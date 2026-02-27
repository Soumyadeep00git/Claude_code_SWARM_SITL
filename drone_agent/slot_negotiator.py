"""
Distributed slot negotiation for formation assignment.

When FORMATION_CMD arrives, drones negotiate which slot each takes based on
proximity (closest drone wins) rather than static drone-ID ordering.

Protocol phases (timeout-driven, ~500ms total):
  0. CORNER_BID   — Closest drones claim perimeter/corner slots
  1. MIDDLE_BID   — Remaining drones claim nearest available slot
  2. TIEBREAK     — Conflicts resolved by random nonce exchange
  3. CONFIRM      — All drones broadcast final assignment
  4. SETTLED      — Slot applied to GlobalPlanner

Degrades gracefully to static (drone_id - 1) assignment on total packet loss.
"""

import random
import hashlib
import logging
from enum import Enum
from math import radians, sqrt

from config import (
    SLOT_NEG_CORNER_PHASE_MS,
    SLOT_NEG_MIDDLE_PHASE_MS,
    SLOT_NEG_TIEBREAK_PHASE_MS,
    SLOT_NEG_CONFIRM_PHASE_MS,
    SLOT_NEG_EQUIDISTANT_M,
    SLOT_NEG_REDUNDANT_SENDS,
)
from drone_agent.global_planner import compute_slot_offset

log = logging.getLogger(__name__)


class NegState(Enum):
    IDLE = "IDLE"
    PHASE_CORNER_BID = "PHASE_CORNER_BID"
    PHASE_MIDDLE_BID = "PHASE_MIDDLE_BID"
    PHASE_TIEBREAK = "PHASE_TIEBREAK"
    PHASE_CONFIRM = "PHASE_CONFIRM"
    SETTLED = "SETTLED"


# ── Corner slot definitions per formation shape ──────────────

def get_corner_slots(formation: str, num_drones: int) -> set[int]:
    """Return the set of corner/perimeter slot indices for a formation."""
    if formation == "LINE":
        return {0, num_drones - 1} if num_drones > 1 else {0}
    elif formation == "V":
        return {0}
    elif formation == "COLUMN":
        return {0, num_drones - 1} if num_drones > 1 else {0}
    elif formation == "DIAMOND":
        return {0, 1, 2, 3} & set(range(num_drones))
    return set()


# ── Phase durations (seconds) ────────────────────────────────

_PHASE_DURATIONS = {
    NegState.PHASE_CORNER_BID: SLOT_NEG_CORNER_PHASE_MS / 1000.0,
    NegState.PHASE_MIDDLE_BID: SLOT_NEG_MIDDLE_PHASE_MS / 1000.0,
    NegState.PHASE_TIEBREAK:   SLOT_NEG_TIEBREAK_PHASE_MS / 1000.0,
    NegState.PHASE_CONFIRM:    SLOT_NEG_CONFIRM_PHASE_MS / 1000.0,
}


class SlotNegotiator:
    """Distributed slot negotiation for formation assignment."""

    def __init__(self, drone_id: int, num_drones: int):
        self.drone_id = drone_id
        self.num_drones = num_drones
        self.state = NegState.IDLE

        # Negotiation identity
        self.neg_id: str = ""
        self.nonce: int = 0

        # Own claim
        self.my_slot: int = -1

        # Peer tracking
        self.bids: dict[int, dict] = {}       # {drone_id: bid_data}
        self.confirms: dict[int, int] = {}    # {drone_id: final_slot}

        # Formation params (set on start)
        self._formation: str = ""
        self._spacing: float = 5.0
        self._heading_rad: float = 0.0
        self._my_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)

        # Timing
        self._phase_start: float = 0.0
        self._sends_this_phase: int = 0
        self._last_send_time: float = 0.0

        # Outbox: list of (msg_type, payload) to send
        self._outbox: list[tuple[str, dict]] = []

    # ── Public API ───────────────────────────────────────────

    def start(self, formation_data: dict, my_ned: tuple[float, float, float],
              now: float) -> None:
        """Begin negotiation from FORMATION_CMD data."""
        self._formation = formation_data.get("formation", "LINE")
        self._spacing = formation_data.get("spacing_m", 5.0)
        self._heading_rad = radians(formation_data.get("heading_deg", 0.0))
        self._my_ned = my_ned
        self.num_drones = formation_data.get("_num_drones", self.num_drones)

        # Generate unique negotiation ID and random tiebreak nonce
        ts = formation_data.get("_neg_ts", now)
        raw = f"{ts}:{self._formation}:{self._spacing}"
        self.neg_id = hashlib.md5(raw.encode()).hexdigest()[:8]
        self.nonce = random.randint(0, 65535)

        # Reset state
        self.my_slot = -1
        self.bids.clear()
        self.confirms.clear()
        self._sends_this_phase = 0
        self._last_send_time = 0.0
        self._outbox.clear()

        self._phase_start = now
        self.state = NegState.PHASE_CORNER_BID
        log.info("Drone %d: slot negotiation started (neg_id=%s, formation=%s)",
                 self.drone_id, self.neg_id, self._formation)

    def handle_message(self, msg_type: str, data: dict) -> None:
        """Process an incoming SLOT_BID / SLOT_TIEBREAK / SLOT_CONFIRM."""
        if self.state == NegState.IDLE:
            return
        # Ignore messages from different negotiations
        if data.get("neg_id") != self.neg_id:
            return
        src = data.get("drone_id", -1)
        if src == self.drone_id:
            return  # Ignore own echoes

        if msg_type == "SLOT_BID":
            self.bids[src] = data
        elif msg_type == "SLOT_TIEBREAK":
            # Update bid with tiebreak nonce
            if src in self.bids:
                self.bids[src]["nonce"] = data.get("nonce", 0)
            else:
                self.bids[src] = data
        elif msg_type == "SLOT_CONFIRM":
            self.confirms[src] = data.get("final_slot", -1)

    def tick(self, now: float) -> int | None:
        """Advance state machine. Returns final slot when SETTLED, else None."""
        if self.state == NegState.IDLE or self.state == NegState.SETTLED:
            return None

        elapsed = now - self._phase_start
        duration = _PHASE_DURATIONS.get(self.state, 0.1)

        # Execute current phase logic
        if self.state == NegState.PHASE_CORNER_BID:
            self._tick_corner_bid(now)
            if elapsed >= duration:
                self._phase_start = now
                self._sends_this_phase = 0
                self._last_send_time = 0.0
                self.state = NegState.PHASE_MIDDLE_BID

        elif self.state == NegState.PHASE_MIDDLE_BID:
            self._tick_middle_bid(now)
            if elapsed >= duration:
                self._phase_start = now
                self._sends_this_phase = 0
                self._last_send_time = 0.0
                self.state = NegState.PHASE_TIEBREAK

        elif self.state == NegState.PHASE_TIEBREAK:
            self._tick_tiebreak(now)
            if elapsed >= duration:
                self._phase_start = now
                self._sends_this_phase = 0
                self._last_send_time = 0.0
                self.state = NegState.PHASE_CONFIRM

        elif self.state == NegState.PHASE_CONFIRM:
            self._tick_confirm(now)
            if elapsed >= duration:
                return self._settle()

        return None

    def pop_messages(self) -> list[tuple[str, dict]]:
        """Drain outbox. Returns list of (msg_type, payload) to send."""
        msgs = list(self._outbox)
        self._outbox.clear()
        return msgs

    @property
    def is_active(self) -> bool:
        return self.state not in (NegState.IDLE, NegState.SETTLED)

    def reset(self) -> None:
        """Abort negotiation, return to IDLE."""
        self.state = NegState.IDLE
        self.my_slot = -1
        self.bids.clear()
        self.confirms.clear()
        self._outbox.clear()

    # ── Phase implementations ────────────────────────────────

    def _tick_corner_bid(self, now: float) -> None:
        """Phase 0: Bid for corner slots."""
        if self._sends_this_phase >= SLOT_NEG_REDUNDANT_SENDS:
            return
        if now - self._last_send_time < 0.03:  # 30ms between sends
            return

        corners = get_corner_slots(self._formation, self.num_drones)
        if not corners:
            # No corners defined — skip to middle
            self._send_bid(-1, float("inf"), False, 0, now)
            return

        # Find closest corner slot
        best_slot, best_dist = self._find_nearest_slot(corners)

        if best_dist < float("inf"):
            self.my_slot = best_slot
            self._send_bid(best_slot, best_dist, True, 0, now)
        else:
            # No GPS — defer
            self._send_bid(-1, float("inf"), False, 0, now)

    def _tick_middle_bid(self, now: float) -> None:
        """Phase 1: Non-corner drones bid for remaining slots."""
        if self._sends_this_phase >= SLOT_NEG_REDUNDANT_SENDS:
            return
        if now - self._last_send_time < 0.03:
            return

        corners = get_corner_slots(self._formation, self.num_drones)

        # Resolve corner winners from collected bids
        corner_winners = self._resolve_corner_winners()
        i_won_corner = (self.my_slot >= 0 and self.my_slot in corners and
                        corner_winners.get(self.my_slot) == self.drone_id)

        if i_won_corner:
            # Re-broadcast winning corner bid
            self._send_bid(self.my_slot, self._distance_to_slot(self.my_slot),
                           True, 1, now)
            return

        # We didn't win a corner — reset and pick from available middle slots
        self.my_slot = -1
        claimed = set(corner_winners.keys())
        available = set(range(self.num_drones)) - claimed

        if not available:
            self._send_bid(-1, float("inf"), False, 1, now)
            return

        best_slot, best_dist = self._find_nearest_slot(available)

        # Check equidistant condition
        if len(available) >= 2:
            dists = sorted(
                (self._distance_to_slot(s), s) for s in available)
            if len(dists) >= 2 and abs(dists[0][0] - dists[1][0]) < SLOT_NEG_EQUIDISTANT_M:
                # Equidistant — defer (bid_slot = -1)
                self._send_bid(-1, best_dist, False, 1, now)
                return

        self.my_slot = best_slot
        self._send_bid(best_slot, best_dist, False, 1, now)

    def _tick_tiebreak(self, now: float) -> None:
        """Phase 2: Resolve conflicts via nonce comparison."""
        if self._sends_this_phase >= SLOT_NEG_REDUNDANT_SENDS:
            return
        if now - self._last_send_time < 0.03:
            return

        if self.my_slot < 0:
            # We deferred — pick from unclaimed using sorted-ID fallback
            claimed = self._get_claimed_slots()
            available = set(range(self.num_drones)) - claimed
            if available:
                # Deterministic fallback: sorted undecided drones pick in ID order
                undecided = self._get_undecided_drones()
                undecided_sorted = sorted(undecided)
                if self.drone_id in undecided_sorted:
                    idx = undecided_sorted.index(self.drone_id)
                    avail_sorted = sorted(available)
                    if idx < len(avail_sorted):
                        self.my_slot = avail_sorted[idx]

        # Check for conflicts: another drone also wants our slot
        if self.my_slot >= 0:
            conflict_drone = self._find_conflict(self.my_slot)
            if conflict_drone is not None:
                other_nonce = self.bids.get(conflict_drone, {}).get("nonce", 0)
                if not self._wins_tiebreak(other_nonce, conflict_drone):
                    # We lose — pick next best
                    log.info("Drone %d: lost tiebreak for slot %d to drone %d",
                             self.drone_id, self.my_slot, conflict_drone)
                    claimed = self._get_claimed_slots()
                    claimed.add(self.my_slot)  # Our slot is taken
                    available = set(range(self.num_drones)) - claimed
                    if available:
                        self.my_slot, _ = self._find_nearest_slot(available)
                    else:
                        self.my_slot = self.drone_id - 1  # Ultimate fallback

                # Broadcast tiebreak
                self._outbox.append(("SLOT_TIEBREAK", {
                    "neg_id": self.neg_id,
                    "drone_id": self.drone_id,
                    "contested_slot": self.my_slot,
                    "nonce": self.nonce,
                }))

        self._sends_this_phase += 1
        self._last_send_time = now

    def _tick_confirm(self, now: float) -> None:
        """Phase 3: Broadcast final slot assignment."""
        if self._sends_this_phase >= SLOT_NEG_REDUNDANT_SENDS:
            return
        if now - self._last_send_time < 0.03:
            return

        # Final fallback: if still undecided, use static assignment
        if self.my_slot < 0:
            self.my_slot = self.drone_id - 1

        self._outbox.append(("SLOT_CONFIRM", {
            "neg_id": self.neg_id,
            "drone_id": self.drone_id,
            "final_slot": self.my_slot,
        }))
        self._sends_this_phase += 1
        self._last_send_time = now

    def _settle(self) -> int:
        """Finalize negotiation. Returns the assigned slot."""
        # Incorporate any confirms we received — check for duplicate slots
        taken = set()
        for did, slot in self.confirms.items():
            taken.add(slot)

        # If our slot is already taken by a confirmed peer, fallback
        if self.my_slot in taken and self.my_slot not in {
                self.confirms.get(self.drone_id)}:
            available = set(range(self.num_drones)) - taken
            if available:
                self.my_slot = min(available)
            # else keep our slot — eventual consistency

        if self.my_slot < 0:
            self.my_slot = self.drone_id - 1

        self.state = NegState.SETTLED
        log.info("Drone %d: negotiation settled → slot %d (neg_id=%s)",
                 self.drone_id, self.my_slot, self.neg_id)
        return self.my_slot

    # ── Helpers ──────────────────────────────────────────────

    def _compute_slot_ned(self, slot: int) -> tuple[float, float, float]:
        """Compute NED target for a given slot."""
        n, e = compute_slot_offset(
            slot, self.num_drones, self._formation,
            self._spacing, self._heading_rad)
        return (n, e, 0.0)

    def _distance_to_slot(self, slot: int) -> float:
        """3D Euclidean distance from own NED position to slot target."""
        sn, se, sd = self._compute_slot_ned(slot)
        mn, me, md = self._my_ned
        return sqrt((sn - mn) ** 2 + (se - me) ** 2 + (sd - md) ** 2)

    def _find_nearest_slot(self, slots: set[int]) -> tuple[int, float]:
        """Find the closest slot from a set. Returns (slot, distance)."""
        best_slot = -1
        best_dist = float("inf")
        for s in sorted(slots):  # Sort for determinism on ties
            d = self._distance_to_slot(s)
            if d < best_dist:
                best_dist = d
                best_slot = s
        return best_slot, best_dist

    def _resolve_corner_winners(self) -> dict[int, int]:
        """From corner bids, determine which drone won each corner.
        Returns {slot: winning_drone_id}."""
        corners = get_corner_slots(self._formation, self.num_drones)

        # Collect all corner bids: {slot: [(distance, nonce, drone_id), ...]}
        slot_bidders: dict[int, list] = {}

        # Add own corner bid
        if self.my_slot >= 0 and self.my_slot in corners:
            d = self._distance_to_slot(self.my_slot)
            slot_bidders.setdefault(self.my_slot, []).append(
                (d, self.nonce, self.drone_id))

        # Add peer corner bids
        for did, bid in self.bids.items():
            slot = bid.get("bid_slot", -1)
            if slot >= 0 and slot in corners and bid.get("is_corner"):
                slot_bidders.setdefault(slot, []).append(
                    (bid.get("distance", float("inf")),
                     bid.get("nonce", 0), did))

        # For each corner, pick winner: closest distance, then highest nonce,
        # then highest drone_id
        winners = {}
        for slot, bidders in slot_bidders.items():
            if not bidders:
                continue
            # Sort: distance ascending, nonce descending, drone_id descending
            bidders.sort(key=lambda x: (x[0], -x[1], -x[2]))
            winners[slot] = bidders[0][2]

        return winners

    def _get_claimed_slots(self) -> set[int]:
        """Return set of slots claimed by drones (from bids)."""
        claimed: set[int] = set()
        for did, bid in self.bids.items():
            slot = bid.get("bid_slot", -1)
            if slot >= 0:
                claimed.add(slot)
        if self.my_slot >= 0:
            claimed.add(self.my_slot)
        return claimed

    def _get_undecided_drones(self) -> set[int]:
        """Return drone IDs that haven't claimed a slot yet."""
        decided = set()
        for did, bid in self.bids.items():
            if bid.get("bid_slot", -1) >= 0:
                decided.add(did)
        if self.my_slot >= 0:
            decided.add(self.drone_id)
        all_drones = set(range(1, self.num_drones + 1))
        return all_drones - decided

    def _find_conflict(self, slot: int) -> int | None:
        """Find another drone that also bids for the same slot."""
        for did, bid in self.bids.items():
            if did != self.drone_id and bid.get("bid_slot") == slot:
                return did
        return None

    def _wins_tiebreak(self, other_nonce: int, other_id: int) -> bool:
        """Return True if we win against another drone.
        Higher nonce wins. Equal nonce: higher drone_id wins."""
        if self.nonce != other_nonce:
            return self.nonce > other_nonce
        return self.drone_id > other_id

    def _send_bid(self, slot: int, distance: float, is_corner: bool,
                  round_num: int, now: float) -> None:
        """Enqueue a SLOT_BID message."""
        self._outbox.append(("SLOT_BID", {
            "neg_id": self.neg_id,
            "drone_id": self.drone_id,
            "round": round_num,
            "bid_slot": slot,
            "distance": distance,
            "is_corner": is_corner,
            "nonce": self.nonce,
        }))
        self._sends_this_phase += 1
        self._last_send_time = now

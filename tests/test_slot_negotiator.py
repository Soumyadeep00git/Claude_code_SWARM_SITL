"""Tests for drone_agent/slot_negotiator.py — distributed slot negotiation."""

import time
import pytest
from math import radians

from drone_agent.slot_negotiator import (
    SlotNegotiator, NegState, get_corner_slots,
)
from drone_agent.global_planner import compute_slot_offset


# ── Helpers ──────────────────────────────────────────────────

def _make_formation_data(formation="LINE", spacing=5.0, heading=0.0,
                         num_drones=5):
    return {
        "formation": formation,
        "spacing_m": spacing,
        "heading_deg": heading,
        "ref_lat": -35.363,
        "ref_lon": 149.165,
        "ref_alt": 10.0,
        "_num_drones": num_drones,
        "_neg_ts": 1000.0,  # Fixed timestamp for deterministic neg_id
    }


def _slot_ned(slot, num_drones=5, formation="LINE", spacing=5.0, heading=0.0):
    """Compute NED position for a slot."""
    n, e = compute_slot_offset(slot, num_drones, formation, spacing,
                               radians(heading))
    return (n, e, 0.0)


def _run_full_negotiation(negotiators, formation_data, positions, max_rounds=60):
    """Run a full negotiation between multiple SlotNegotiators.

    positions: dict {drone_id: (n, e, d)} NED positions.
    Returns dict {drone_id: final_slot}.
    """
    now = 1000.0
    for neg in negotiators:
        did = neg.drone_id
        neg.start(formation_data, positions[did], now)

    results = {}
    for _ in range(max_rounds):
        now += 0.02  # 20ms per tick (50Hz for faster convergence in tests)
        all_msgs = []
        for neg in negotiators:
            result = neg.tick(now)
            if result is not None:
                results[neg.drone_id] = result
            for msg_type, payload in neg.pop_messages():
                all_msgs.append((msg_type, payload))

        # Deliver all messages to all negotiators
        for msg_type, payload in all_msgs:
            for neg in negotiators:
                neg.handle_message(msg_type, payload)

        if len(results) == len(negotiators):
            break

    # Force-settle any stragglers
    for neg in negotiators:
        if neg.drone_id not in results:
            results[neg.drone_id] = neg._settle()

    return results


# ── Test: Corner Slot Definitions ────────────────────────────

class TestCornerSlots:
    def test_line_corners(self):
        assert get_corner_slots("LINE", 5) == {0, 4}
        assert get_corner_slots("LINE", 3) == {0, 2}
        assert get_corner_slots("LINE", 1) == {0}

    def test_v_corners(self):
        assert get_corner_slots("V", 5) == {0}
        assert get_corner_slots("V", 1) == {0}

    def test_column_corners(self):
        assert get_corner_slots("COLUMN", 5) == {0, 4}
        assert get_corner_slots("COLUMN", 2) == {0, 1}

    def test_diamond_corners(self):
        assert get_corner_slots("DIAMOND", 5) == {0, 1, 2, 3}
        assert get_corner_slots("DIAMOND", 3) == {0, 1, 2}
        assert get_corner_slots("DIAMOND", 6) == {0, 1, 2, 3}

    def test_unknown_formation(self):
        assert get_corner_slots("HEXAGON", 5) == set()


# ── Test: Offset Consistency ─────────────────────────────────

class TestOffsetConsistency:
    def test_extracted_matches_original(self):
        """compute_slot_offset produces same results for all shapes."""
        for formation in ("LINE", "V", "COLUMN", "DIAMOND"):
            for slot in range(5):
                n, e = compute_slot_offset(slot, 5, formation, 5.0, 0.0)
                assert isinstance(n, float)
                assert isinstance(e, float)

    def test_rotated_offsets(self):
        """Heading rotation changes offset direction."""
        n0, e0 = compute_slot_offset(0, 5, "LINE", 5.0, 0.0)
        n90, e90 = compute_slot_offset(0, 5, "LINE", 5.0, radians(90))
        # 90-degree rotation should swap axes (approximately)
        assert abs(n0 - e90) < 0.01 or abs(e0 + n90) < 0.01


# ── Test: Corner Bid Phase ───────────────────────────────────

class TestCornerBidPhase:
    def test_closest_drone_claims_corner(self):
        """Drone nearest to a corner slot should claim it."""
        data = _make_formation_data("LINE", spacing=10.0)
        # Slot 0 is at east=-20m, slot 4 at east=+20m (LINE with 5 drones)
        neg = SlotNegotiator(1, 5)
        # Place drone 1 near slot 0 position
        slot0_ned = _slot_ned(0, 5, "LINE", 10.0)
        my_ned = (slot0_ned[0] + 1.0, slot0_ned[1] + 1.0, 0.0)  # 1m away
        neg.start(data, my_ned, 1000.0)
        neg.tick(1000.01)
        msgs = neg.pop_messages()
        assert len(msgs) > 0
        bid = msgs[0][1]
        assert bid["bid_slot"] == 0  # Should claim slot 0 (nearest corner)
        assert bid["is_corner"] is True


# ── Test: Middle Bid Phase ───────────────────────────────────

class TestMiddleBidPhase:
    def test_middle_drone_picks_nearest(self):
        """After corners claimed, middle drone picks nearest available."""
        data = _make_formation_data("LINE", spacing=10.0)
        # Slot 2 (center) is at east=0m
        neg = SlotNegotiator(3, 5)
        my_ned = (0.0, 0.5, 0.0)  # Near center slot
        neg.start(data, my_ned, 1000.0)

        # Simulate corner bids from other drones
        neg.handle_message("SLOT_BID", {
            "neg_id": neg.neg_id, "drone_id": 1, "round": 0,
            "bid_slot": 0, "distance": 1.0, "is_corner": True, "nonce": 100,
        })
        neg.handle_message("SLOT_BID", {
            "neg_id": neg.neg_id, "drone_id": 5, "round": 0,
            "bid_slot": 4, "distance": 1.0, "is_corner": True, "nonce": 200,
        })

        # Advance through corner phase into middle phase
        now = 1000.0
        while neg.state == NegState.PHASE_CORNER_BID:
            now += 0.02
            neg.tick(now)
            neg.pop_messages()

        # Now in MIDDLE_BID phase — tick to generate bid
        neg.tick(now + 0.01)
        msgs = neg.pop_messages()
        bids = [m for t, m in msgs if t == "SLOT_BID"]
        assert len(bids) > 0
        # Should pick slot 2 (nearest to center)
        assert bids[0]["bid_slot"] == 2


# ── Test: Equidistant Defer ──────────────────────────────────

class TestEquidistantDefer:
    def test_equidistant_drones_defer(self):
        """Drones equidistant to all remaining slots should defer."""
        data = _make_formation_data("LINE", spacing=10.0)
        neg = SlotNegotiator(3, 5)
        # Place at origin — equidistant to slots 1 and 3 (symmetric about center)
        # Slots: 0=-20, 1=-10, 2=0, 3=10, 4=20
        # If corners 0,4 are claimed, remaining: 1,2,3
        # At position east=-5: dist to slot1=5m, dist to slot2=5m → equidistant
        my_ned = (0.0, -5.0, 0.0)
        neg.start(data, my_ned, 1000.0)

        # Simulate corner claims
        neg.handle_message("SLOT_BID", {
            "neg_id": neg.neg_id, "drone_id": 1, "round": 0,
            "bid_slot": 0, "distance": 1.0, "is_corner": True, "nonce": 100,
        })
        neg.handle_message("SLOT_BID", {
            "neg_id": neg.neg_id, "drone_id": 5, "round": 0,
            "bid_slot": 4, "distance": 1.0, "is_corner": True, "nonce": 200,
        })

        # Advance to middle phase
        now = 1000.0
        while neg.state == NegState.PHASE_CORNER_BID:
            now += 0.02
            neg.tick(now)
            neg.pop_messages()

        # Tick middle phase
        neg.tick(now + 0.01)
        msgs = neg.pop_messages()
        bids = [m for t, m in msgs if t == "SLOT_BID"]
        # Should defer (bid_slot = -1) because slots 1 and 2 are equidistant
        if bids:
            assert bids[0]["bid_slot"] == -1


# ── Test: Tiebreak ───────────────────────────────────────────

class TestTiebreak:
    def test_higher_nonce_wins(self):
        neg = SlotNegotiator(1, 5)
        neg.nonce = 100
        assert neg._wins_tiebreak(50, 2) is True   # 100 > 50
        assert neg._wins_tiebreak(200, 2) is False  # 100 < 200

    def test_equal_nonce_higher_id_wins(self):
        neg = SlotNegotiator(3, 5)
        neg.nonce = 100
        assert neg._wins_tiebreak(100, 2) is True   # Same nonce, 3 > 2
        assert neg._wins_tiebreak(100, 5) is False   # Same nonce, 3 < 5


# ── Test: No GPS Fallback ────────────────────────────────────

class TestNoGPSFallback:
    def test_no_gps_gets_static_slot(self):
        """Drone with no GPS should end up with a valid slot via fallback."""
        data = _make_formation_data("LINE")
        neg = SlotNegotiator(3, 5)
        neg.start(data, (0.0, 0.0, 0.0), 1000.0)

        # Run through all phases with no peer messages
        now = 1000.0
        result = None
        for _ in range(50):
            now += 0.02
            result = neg.tick(now)
            neg.pop_messages()
            if result is not None:
                break

        assert result is not None
        assert 0 <= result < 5


# ── Test: Timeout Fallback ───────────────────────────────────

class TestTimeoutFallback:
    def test_no_messages_degrades_to_static(self):
        """With zero peer messages, negotiation degrades to static slot."""
        data = _make_formation_data("LINE")
        slot1_ned = _slot_ned(1, 5, "LINE")

        neg = SlotNegotiator(2, 5)
        neg.start(data, slot1_ned, 1000.0)

        # Run all phases with no messages from peers
        now = 1000.0
        result = None
        for _ in range(60):
            now += 0.02
            result = neg.tick(now)
            neg.pop_messages()
            if result is not None:
                break

        assert result is not None
        assert 0 <= result < 5


# ── Test: Failsafe Abort ────────────────────────────────────

class TestFailsafeAbort:
    def test_reset_returns_to_idle(self):
        data = _make_formation_data("LINE")
        neg = SlotNegotiator(1, 5)
        neg.start(data, (0.0, 0.0, 0.0), 1000.0)
        assert neg.is_active
        neg.reset()
        assert not neg.is_active
        assert neg.state == NegState.IDLE
        assert neg.my_slot == -1


# ── Test: Full Negotiation ───────────────────────────────────

class TestFullNegotiation:
    def test_five_drones_get_unique_slots(self):
        """5 negotiators exchanging messages should converge to unique slots."""
        data = _make_formation_data("LINE", spacing=10.0)
        # Place each drone near a different slot position
        positions = {}
        for did in range(1, 6):
            slot_ned = _slot_ned(did - 1, 5, "LINE", 10.0)
            # Offset slightly so each drone is near "its" slot
            positions[did] = (slot_ned[0] + 0.5, slot_ned[1] + 0.5, 0.0)

        negotiators = [SlotNegotiator(did, 5) for did in range(1, 6)]
        results = _run_full_negotiation(negotiators, data, positions)

        assert len(results) == 5
        slots = set(results.values())
        assert len(slots) == 5  # All unique
        assert slots == {0, 1, 2, 3, 4}

    def test_diamond_five_drones(self):
        """Diamond with 5 drones: 4 corners + 1 middle."""
        data = _make_formation_data("DIAMOND", spacing=10.0)
        positions = {}
        for did in range(1, 6):
            slot_ned = _slot_ned(did - 1, 5, "DIAMOND", 10.0)
            positions[did] = (slot_ned[0] + 0.3, slot_ned[1] + 0.3, 0.0)

        negotiators = [SlotNegotiator(did, 5) for did in range(1, 6)]
        results = _run_full_negotiation(negotiators, data, positions)

        assert len(results) == 5
        slots = set(results.values())
        assert len(slots) == 5
        assert slots == {0, 1, 2, 3, 4}

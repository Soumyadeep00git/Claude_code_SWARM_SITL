"""
Diagnostics test runner — exercises algorithm modules with mock data.
Each test returns a standardised result dict for the web UI.

No live drones needed (except test_comms_connectivity).
"""

import time
import math
import logging
from contextlib import contextmanager

log = logging.getLogger(__name__)


@contextmanager
def _suppress_module_logging():
    """Temporarily silence drone_agent loggers so tests don't spam GCS."""
    loggers = [
        logging.getLogger("drone_agent.global_planner"),
        logging.getLogger("drone_agent.failsafe"),
        logging.getLogger("drone_agent.local_planner"),
        logging.getLogger("drone_agent.mppi"),
        logging.getLogger("drone_agent.hybrid_astar"),
        logging.getLogger("mavlink_layer.drone_connection"),
    ]
    old_levels = [lg.level for lg in loggers]
    for lg in loggers:
        lg.setLevel(logging.CRITICAL)
    try:
        yield
    finally:
        for lg, lvl in zip(loggers, old_levels):
            lg.setLevel(lvl)


# ── Test 1: Formation Geometry ─────────────────────────────

def test_formation_geometry():
    """Verify all formation shapes produce correct, non-overlapping offsets."""
    from drone_agent.global_planner import GlobalPlanner

    with _suppress_module_logging():
        return _run_formation_geometry(GlobalPlanner)


def _run_formation_geometry(GlobalPlanner):
    shapes = ["LINE", "V", "COLUMN", "DIAMOND"]
    ref_lat, ref_lon, ref_alt = -35.363, 149.165, 10.0
    spacing = 5.0
    assertions = []
    all_positions = {}

    for n in [3, 5, 7]:
        for shape in shapes:
            for heading in [0, 90, 180]:
                positions = []
                for slot in range(n):
                    gp = GlobalPlanner(drone_id=slot + 1, num_drones=n)
                    gp.set_formation({
                        "formation": shape,
                        "leader_id": 1,
                        "ref_lat": ref_lat, "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "heading_deg": heading, "spacing_m": spacing,
                    })
                    gp.slot = slot
                    off_n, off_e = gp._compute_slot_offset()
                    lat, lon, alt = gp.get_target_position()
                    positions.append({
                        "slot": slot, "drone_id": slot + 1,
                        "offset_n": round(off_n, 4),
                        "offset_e": round(off_e, 4),
                        "lat": lat, "lon": lon, "alt": alt,
                    })

                coords = [(p["offset_n"], p["offset_e"]) for p in positions]
                unique = len(set((round(c[0], 2), round(c[1], 2))
                                 for c in coords))
                assertions.append({
                    "check": f"{shape} N={n} h={heading}: all positions distinct",
                    "passed": unique == n,
                    "value": f"{unique}/{n} unique",
                })

                min_dist = float("inf")
                for i in range(len(coords)):
                    for j in range(i + 1, len(coords)):
                        d = math.hypot(coords[i][0] - coords[j][0],
                                       coords[i][1] - coords[j][1])
                        min_dist = min(min_dist, d)
                # DIAMOND slot 0 is at +spacing, slots 1/2 at ±spacing
                # so min distance can be spacing*sqrt(2) ≈ 0.7*spacing for some shapes
                assertions.append({
                    "check": f"{shape} N={n} h={heading}: min spacing >= {spacing * 0.4:.1f}m",
                    "passed": min_dist >= spacing * 0.4,
                    "value": f"{min_dist:.2f}m",
                })

                key = f"{shape}_N{n}_H{heading}"
                all_positions[key] = positions

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Formation Geometry",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {
            "formations_tested": len(all_positions),
            "positions": all_positions,
        },
    }


# ── Test 2: Failsafe State Machine ────────────────────────

def test_failsafe_transitions():
    """Drive the failsafe state machine through key transitions."""
    with _suppress_module_logging():
        return _run_failsafe_transitions()


def _run_failsafe_transitions():
    from drone_agent.failsafe import FailsafeManager, SwarmState
    from drone_agent.state import DroneState, PeerTable

    class MockConn:
        drone_id = 1
        def rtl(self): pass
        def set_mode(self, m): pass
        def land(self): pass

    class MockPlanner:
        def hold(self): pass
        def do_land(self): pass

    class MockGlobalPlanner:
        formation = "LINE"
        def reslot(self, *a, **kw): pass

    fm = FailsafeManager(1, MockPlanner(), MockConn(),
                         MockGlobalPlanner(),
                         expected_peers={1, 2, 3})
    assertions = []
    transitions = []

    # Setup: valid GPS + armed drone state
    my = DroneState(drone_id=1)
    my.lat, my.lon = -35.363, 149.165
    my.armed = True
    my.battery_pct = 80
    my.mode = "GUIDED"
    peers = PeerTable()

    # Fresh peers so peer-health doesn't trigger first
    for pid in [2, 3]:
        peers.update_peer(pid, {"drone_id": pid, "lat": -35.364,
                                "lon": 149.166}, time.time())
        fm.update_peer_heartbeat(pid)

    # T1: Starts NOMINAL
    assertions.append({
        "check": "Initial state is NOMINAL",
        "passed": fm.state == SwarmState.NOMINAL,
        "value": fm.state.value,
    })

    # T2: GCS contacted then timeout → COMMS_LOST
    fm.update_gcs_heartbeat()
    fm.last_gcs_time = time.time() - 10
    fm.tick(peers, my)
    assertions.append({
        "check": "GCS timeout -> COMMS_LOST",
        "passed": fm.state == SwarmState.COMMS_LOST,
        "value": fm.state.value,
    })
    transitions.append("NOMINAL -> COMMS_LOST")

    # T3: GCS restored → COMMS_RECOVERY
    fm.update_gcs_heartbeat()
    fm.tick(peers, my)
    assertions.append({
        "check": "GCS restored -> COMMS_RECOVERY",
        "passed": fm.state == SwarmState.COMMS_RECOVERY,
        "value": fm.state.value,
    })
    transitions.append("COMMS_LOST -> COMMS_RECOVERY")

    # T4: GUIDED confirmed → NOMINAL
    fm.tick(peers, my)
    assertions.append({
        "check": "GUIDED confirmed -> NOMINAL",
        "passed": fm.state == SwarmState.NOMINAL,
        "value": fm.state.value,
    })
    transitions.append("COMMS_RECOVERY -> NOMINAL")

    # T5: Peer stale → DEGRADED
    peers.last_update[2] = time.time() - 20
    fm._peer_last_contact[2] = time.time() - 20
    fm.tick(peers, my)
    assertions.append({
        "check": "Peer stale -> DEGRADED",
        "passed": fm.state == SwarmState.DEGRADED,
        "value": fm.state.value,
    })
    transitions.append("NOMINAL -> DEGRADED")

    # T6: Peer returns → NOMINAL
    peers.update_peer(2, {"drone_id": 2, "lat": -35.364,
                          "lon": 149.166}, time.time())
    fm.update_peer_heartbeat(2)
    fm.tick(peers, my)
    assertions.append({
        "check": "Peer returns -> NOMINAL",
        "passed": fm.state == SwarmState.NOMINAL,
        "value": fm.state.value,
    })
    transitions.append("DEGRADED -> NOMINAL")

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Failsafe State Machine",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"transitions": transitions},
    }


# ── Test 3: Leader Election ────────────────────────────────

def test_leader_election():
    """Verify leader = min(alive_ids) for various alive sets."""
    with _suppress_module_logging():
        return _run_leader_election()


def _run_leader_election():
    from drone_agent.failsafe import FailsafeManager
    from drone_agent.state import DroneState

    class Mock:
        drone_id = 1
        formation = "NONE"
        def rtl(self): pass
        def set_mode(self, m): pass
        def land(self): pass
        def hold(self): pass
        def do_land(self): pass
        def reslot(self, *a, **kw): pass

    m = Mock()
    fm = FailsafeManager(1, m, m, m, expected_peers={1, 2, 3, 4, 5})

    cases = [
        ({1, 2, 3, 4, 5}, 1),
        ({2, 3, 5}, 2),
        ({4, 5}, 4),
        ({5}, 5),
        ({3}, 3),
    ]

    assertions = []
    for alive, expected in cases:
        fm._alive_peers = alive
        got = fm.elect_leader()
        assertions.append({
            "check": f"alive={sorted(alive)} -> leader={expected}",
            "passed": got == expected,
            "value": got,
        })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Leader Election",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"cases": len(cases)},
    }


# ── Test 4: NED-GPS Conversion ─────────────────────────────

def test_ned_conversion():
    """Round-trip NED -> GPS -> NED accuracy check."""
    from gcs.geo_utils import ned_to_gps

    ref_lat, ref_lon = -35.3632620, 149.1652370
    offsets = [(0, 0), (100, 0), (0, 100), (100, 50),
               (-50, -75), (200, 200)]

    assertions = []
    for north, east in offsets:
        lat, lon = ned_to_gps(ref_lat, ref_lon, north, east)
        # Convert back
        rec_n = (lat - ref_lat) * 111320.0
        rec_e = (lon - ref_lon) * (111320.0 * math.cos(math.radians(ref_lat)))
        err = math.hypot(rec_n - north, rec_e - east)
        assertions.append({
            "check": f"NED({north},{east}) round-trip error < 0.1m",
            "passed": err < 0.1,
            "value": f"{err:.6f}m",
        })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "NED-GPS Conversion",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"offsets_tested": len(offsets)},
    }


# ── Test 5: Local Planner States ───────────────────────────

def test_local_planner_states():
    """Verify task state transitions of LocalPlanner."""
    with _suppress_module_logging():
        return _run_local_planner_states()


def _run_local_planner_states():
    from drone_agent.local_planner import (
        LocalPlanner, IDLE, TAKEOFF, WAYPOINT, VELOCITY, LAND, HOLD,
    )

    class MockConn:
        drone_id = 1
        def set_mode(self, m): pass
        def arm(self): pass
        def takeoff(self, a): pass
        def land(self): pass
        def send_goto_global(self, *a): pass
        def send_velocity_ned(self, *a): pass

    lp = LocalPlanner(MockConn())
    assertions = []

    assertions.append({
        "check": "Initial state is IDLE",
        "passed": lp.task == IDLE, "value": lp.task,
    })

    lp.do_takeoff(10.0)
    assertions.append({
        "check": "do_takeoff() -> TAKEOFF",
        "passed": lp.task == TAKEOFF, "value": lp.task,
    })

    lp.set_waypoint(-35.363, 149.165, 10.0)
    assertions.append({
        "check": "set_waypoint() -> WAYPOINT",
        "passed": lp.task == WAYPOINT, "value": lp.task,
    })

    lp.set_velocity(1.0, 0.0, 0.0)
    assertions.append({
        "check": "set_velocity() -> VELOCITY",
        "passed": lp.task == VELOCITY, "value": lp.task,
    })

    lp.hold()
    assertions.append({
        "check": "hold() -> HOLD",
        "passed": lp.task == HOLD, "value": lp.task,
    })

    lp.do_land()
    assertions.append({
        "check": "do_land() -> LAND",
        "passed": lp.task == LAND, "value": lp.task,
    })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Local Planner States",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"states": [IDLE, TAKEOFF, WAYPOINT, VELOCITY, HOLD, LAND]},
    }


# ── Test 6: Comms Connectivity (live) ──────────────────────

def test_comms_connectivity(collector):
    """Check state freshness from live drones. Requires running swarm."""
    states = collector.get_all_states()

    if not states:
        return {
            "test_name": "Comms Connectivity",
            "status": "error",
            "assertions": [],
            "details": {"message": "No drones reporting. Launch drones first."},
        }

    now = time.time()
    assertions = []
    matrix = {}

    for did, state in states.items():
        ts = collector.timestamps.get(did, 0)
        age = now - ts if ts else float("inf")
        fresh = age < 2.0
        assertions.append({
            "check": f"D{did} last report age < 2.0s",
            "passed": fresh,
            "value": f"{age:.2f}s",
        })
        matrix[str(did)] = {"age_s": round(age, 2), "fresh": fresh}

    stale = collector.get_stale_drones(5.0)
    assertions.append({
        "check": "No stale drones (timeout=5s)",
        "passed": len(stale) == 0,
        "value": f"stale: {stale}" if stale else "none",
    })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Comms Connectivity",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"matrix": matrix, "total_drones": len(states)},
    }


# ── Test 7: P2P Mesh Health (live) ─────────────────────────

def test_p2p_mesh(collector):
    """Check P2P mesh by examining peer awareness reported in drone states.
    Each drone's state includes alive_count and swarm_state.
    Requires running swarm."""
    states = collector.get_all_states()

    if not states:
        return {
            "test_name": "P2P Mesh Health",
            "status": "error",
            "assertions": [],
            "details": {"message": "No drones reporting. Launch drones first."},
        }

    now = time.time()
    n = len(states)
    assertions = []
    mesh = {}

    for did, state in states.items():
        ts = collector.timestamps.get(did, 0)
        age = now - ts if ts else float("inf")
        alive = state.get("alive_count", 0)
        swarm = state.get("swarm_state", "?")
        failsafe = state.get("failsafe_active", False)

        # Each drone should see all peers (alive_count == total drones)
        assertions.append({
            "check": f"D{did} sees {n} peers (alive_count={alive})",
            "passed": alive >= n,
            "value": f"{alive}/{n}",
        })

        # Each drone should be in NOMINAL or DEGRADED (not COMMS_LOST)
        ok_states = {"NOMINAL", "DEGRADED"}
        assertions.append({
            "check": f"D{did} state is NOMINAL/DEGRADED",
            "passed": swarm in ok_states,
            "value": swarm,
        })

        # No failsafe override active
        assertions.append({
            "check": f"D{did} no failsafe override",
            "passed": not failsafe,
            "value": "active" if failsafe else "clear",
        })

        mesh[str(did)] = {
            "alive_count": alive,
            "swarm_state": swarm,
            "failsafe": failsafe,
            "age_s": round(age, 2),
        }

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "P2P Mesh Health",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"mesh": mesh, "total_drones": n},
    }


# ── Test 8: MPPI Controller ───────────────────────────────

def test_mppi_controller():
    """Verify MPPI produces sensible velocity commands and avoids obstacles."""
    with _suppress_module_logging():
        return _run_mppi_test()


def _run_mppi_test():
    import numpy as np
    from drone_agent.mppi import MPPIController
    from config import MPPI_CONFIG

    mppi = MPPIController(MPPI_CONFIG)
    assertions = []

    # T1: Drone heading toward goal (warm up 5 steps to build velocity)
    pos = np.zeros(3, dtype=np.float64)
    vel_state = np.zeros(3, dtype=np.float64)
    goal = np.array([10, 0, 0], dtype=np.float64)
    for _ in range(5):
        state = np.concatenate([pos, vel_state])
        vel_state = mppi.optimize(state, goal, np.empty((0, 3)))
        pos += vel_state * 0.1

    assertions.append({
        "check": "Velocity toward goal (vn > 0.5 after warm-up)",
        "passed": bool(vel_state[0] > 0.5),
        "value": f"vn={vel_state[0]:.3f}",
    })
    assertions.append({
        "check": f"Velocity <= max ({MPPI_CONFIG['MAX_VELOCITY_MS']} m/s)",
        "passed": bool(np.linalg.norm(vel_state[:2]) <= MPPI_CONFIG["MAX_VELOCITY_MS"] + 0.1),
        "value": f"|v|={np.linalg.norm(vel_state[:2]):.3f}",
    })

    # T2: Peer blocking direct path → lateral avoidance
    mppi.reset()
    state = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
    goal = np.array([10, 0, 0], dtype=np.float64)
    peer = np.array([[5, 0, 0]], dtype=np.float64)
    vel = mppi.optimize(state, goal, peer)

    assertions.append({
        "check": "Blocking peer: lateral avoidance (|ve|>0.01 or vn reduced)",
        "passed": bool(abs(vel[1]) > 0.01 or vel[0] < 0.5),
        "value": f"vn={vel[0]:.3f} ve={vel[1]:.3f}",
    })

    # T3: Already at goal → near-zero velocity
    mppi.reset()
    state = np.array([10, 0, 0, 0, 0, 0], dtype=np.float64)
    goal = np.array([10, 0, 0], dtype=np.float64)
    vel = mppi.optimize(state, goal, np.empty((0, 3)))

    assertions.append({
        "check": "At goal: velocity near zero (|v| < 1.0)",
        "passed": bool(np.linalg.norm(vel[:2]) < 1.0),
        "value": f"|v|={np.linalg.norm(vel[:2]):.3f}",
    })

    # T4: Predictive peers — moving peer heading away shouldn't block
    mppi.reset()
    state = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
    goal = np.array([15, 0, 0], dtype=np.float64)
    peer_pos = np.array([[5, 0, 0]], dtype=np.float64)
    peer_vel_away = np.array([[2, 0, 0]], dtype=np.float64)   # moving away
    peer_vel_toward = np.array([[-2, 0, 0]], dtype=np.float64)  # moving toward us

    vel_away = mppi.optimize(state, goal, peer_pos, peer_velocities_ned=peer_vel_away)
    mppi.reset()
    vel_toward = mppi.optimize(state, goal, peer_pos, peer_velocities_ned=peer_vel_toward)

    assertions.append({
        "check": "Predictive: faster when peer moves away vs toward",
        "passed": bool(vel_away[0] > vel_toward[0]),
        "value": f"away_vn={vel_away[0]:.3f} toward_vn={vel_toward[0]:.3f}",
    })

    # T5: Speed incentive — far from goal builds speed quickly (warm up 10 steps)
    mppi.reset()
    pos = np.zeros(3, dtype=np.float64)
    vel_state = np.zeros(3, dtype=np.float64)
    goal = np.array([30, 0, 0], dtype=np.float64)
    for _ in range(10):
        state = np.concatenate([pos, vel_state])
        vel_state = mppi.optimize(state, goal, np.empty((0, 3)))
        pos += vel_state * 0.1
    speed = float(np.linalg.norm(vel_state[:2]))

    assertions.append({
        "check": "Speed incentive: far from goal → speed > 2.0 m/s (after 1s)",
        "passed": bool(speed > 2.0),
        "value": f"|v|={speed:.3f}",
    })

    # T6: Performance check (256 samples × 20 steps)
    state = np.array([0, 0, 0, 1, 0, 0], dtype=np.float64)
    goal = np.array([20, 5, 0], dtype=np.float64)
    peers = np.array([[8, 0, 0], [15, 3, 0]], dtype=np.float64)
    peer_vels = np.array([[0.5, 0.2, 0], [-0.3, 0.1, 0]], dtype=np.float64)
    mppi.reset()

    t0 = time.time()
    for _ in range(10):
        mppi.optimize(state, goal, peers, peer_velocities_ned=peer_vels)
    elapsed_per_call_ms = (time.time() - t0) / 10 * 1000

    assertions.append({
        "check": f"MPPI compute time < 35ms (K={MPPI_CONFIG['K_SAMPLES']}, T={MPPI_CONFIG['HORIZON_STEPS']})",
        "passed": bool(elapsed_per_call_ms < 35.0),
        "value": f"{elapsed_per_call_ms:.2f}ms/call",
    })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "MPPI Controller",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {
            "K": MPPI_CONFIG["K_SAMPLES"],
            "T": MPPI_CONFIG["HORIZON_STEPS"],
        },
    }


# ── Test 9: Hybrid A* Path Planner ───────────────────────

def test_hybrid_astar():
    """Verify Hybrid A* produces collision-free paths."""
    with _suppress_module_logging():
        return _run_hybrid_astar_test()


def _run_hybrid_astar_test():
    import numpy as np
    from drone_agent.hybrid_astar import HybridAStarPlanner
    from config import ASTAR_CONFIG

    planner = HybridAStarPlanner(ASTAR_CONFIG)
    assertions = []

    # T1: Short distance, no obstacles → direct (1 waypoint)
    path = planner.plan(np.array([0, 0]), np.array([5, 0]), np.empty((0, 2)))
    assertions.append({
        "check": "Short distance: direct path (1 waypoint)",
        "passed": len(path) == 1,
        "value": f"{len(path)} waypoints",
    })

    # T2: Long distance, clear path → direct (1 waypoint)
    path = planner.plan(np.array([0, 0]), np.array([30, 0]), np.empty((0, 2)))
    assertions.append({
        "check": "Clear long path: direct (1 waypoint)",
        "passed": len(path) == 1,
        "value": f"{len(path)} waypoints",
    })

    # T3: Obstacle blocking direct path → multi-waypoint detour
    start = np.array([0, 0])
    goal = np.array([30, 0])
    peers = np.array([[15, 0]])  # Peer right in the middle
    path = planner.plan(start, goal, peers)

    assertions.append({
        "check": "Blocked path: generates >1 waypoint",
        "passed": len(path) > 1,
        "value": f"{len(path)} waypoints",
    })

    # Verify path avoids obstacle
    obstacle_radius = ASTAR_CONFIG["OBSTACLE_RADIUS_M"]
    min_clearance = float("inf")
    for wp in path:
        d = np.linalg.norm(np.array(wp) - peers[0])
        min_clearance = min(min_clearance, d)

    assertions.append({
        "check": f"Path clears obstacle (min clearance >= {obstacle_radius * 0.8:.1f}m)",
        "passed": bool(min_clearance >= obstacle_radius * 0.8),
        "value": f"clearance={min_clearance:.2f}m",
    })

    # T4: Performance check
    peers_many = np.array([[10, 2], [20, -2], [15, 5]])
    t0 = time.time()
    for _ in range(10):
        planner.plan(np.array([0, 0]), np.array([40, 0]), peers_many)
    elapsed_per_call_ms = (time.time() - t0) / 10 * 1000

    assertions.append({
        "check": "A* planning time < 20ms per call",
        "passed": bool(elapsed_per_call_ms < 20.0),
        "value": f"{elapsed_per_call_ms:.2f}ms/call",
    })

    failed = [a for a in assertions if not a["passed"]]
    return {
        "test_name": "Hybrid A* Path Planner",
        "status": "pass" if not failed else "fail",
        "assertions": assertions,
        "details": {"grid_cell_m": ASTAR_CONFIG["GRID_CELL_M"]},
    }


# ── Runners ────────────────────────────────────────────────

def run_all_tests(collector=None):
    """Run all diagnostic tests and return list of results."""
    tests = [
        ("formation_geometry", test_formation_geometry),
        ("failsafe_transitions", test_failsafe_transitions),
        ("leader_election", test_leader_election),
        ("ned_conversion", test_ned_conversion),
        ("local_planner_states", test_local_planner_states),
        ("mppi_controller", test_mppi_controller),
        ("hybrid_astar", test_hybrid_astar),
    ]

    results = []
    for test_id, fn in tests:
        t0 = time.time()
        try:
            r = fn()
            r["duration_ms"] = round((time.time() - t0) * 1000, 2)
            r["test_id"] = test_id
        except Exception as e:
            log.exception("Diagnostic test %s failed", test_id)
            r = {
                "test_id": test_id, "test_name": test_id,
                "status": "error",
                "duration_ms": round((time.time() - t0) * 1000, 2),
                "assertions": [],
                "details": {"error": str(e)},
            }
        results.append(r)

    # Live tests
    if collector:
        for test_id, fn in [("comms_connectivity", test_comms_connectivity),
                             ("p2p_mesh", test_p2p_mesh)]:
            t0 = time.time()
            try:
                r = fn(collector)
                r["duration_ms"] = round((time.time() - t0) * 1000, 2)
                r["test_id"] = test_id
            except Exception as e:
                log.exception("Live test %s failed", test_id)
                r = {
                    "test_id": test_id,
                    "test_name": test_id,
                    "status": "error",
                    "duration_ms": round((time.time() - t0) * 1000, 2),
                    "assertions": [],
                    "details": {"error": str(e)},
                }
            results.append(r)

    return results


def run_single_test(test_id: str, collector=None):
    """Run one test by ID and return its result."""
    test_map = {
        "formation_geometry": test_formation_geometry,
        "failsafe_transitions": test_failsafe_transitions,
        "leader_election": test_leader_election,
        "ned_conversion": test_ned_conversion,
        "local_planner_states": test_local_planner_states,
        "mppi_controller": test_mppi_controller,
        "hybrid_astar": test_hybrid_astar,
    }

    live_tests = {
        "comms_connectivity": test_comms_connectivity,
        "p2p_mesh": test_p2p_mesh,
    }
    if test_id in live_tests:
        if collector is None:
            return {"test_id": test_id, "test_name": test_id,
                    "status": "error", "duration_ms": 0, "assertions": [],
                    "details": {"error": "No collector available"}}
        t0 = time.time()
        r = live_tests[test_id](collector)
        r["duration_ms"] = round((time.time() - t0) * 1000, 2)
        r["test_id"] = test_id
        return r

    fn = test_map.get(test_id)
    if fn is None:
        return {"test_id": test_id, "test_name": test_id, "status": "error",
                "duration_ms": 0, "assertions": [],
                "details": {"error": f"Unknown test: {test_id}"}}

    t0 = time.time()
    try:
        r = fn()
        r["duration_ms"] = round((time.time() - t0) * 1000, 2)
        r["test_id"] = test_id
    except Exception as e:
        log.exception("Diagnostic test %s failed", test_id)
        r = {"test_id": test_id, "test_name": test_id, "status": "error",
             "duration_ms": round((time.time() - t0) * 1000, 2),
             "assertions": [], "details": {"error": str(e)}}
    return r

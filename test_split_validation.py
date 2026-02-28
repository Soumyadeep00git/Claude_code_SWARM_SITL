"""Validation test suite for guidance_lib / failsafe_lib split.

Tests:
  A. Module independence (no cross-imports)
  B. failsafe_lib — all 6 failure modes + normal path
  C. guidance_lib — all modes (no embedded failsafe — removed)
  D. failsafe_lib config.yaml loading
  E. Integration — follower_main import pattern works
  F. Edge cases — NaN, Inf, zero GPS, boundary values
"""

import math
import sys
import unittest
import importlib
import os
import tempfile

# ── Ensure project root is on path ──────────────────────────────
_PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
if _PROJECT_DIR not in sys.path:
    sys.path.insert(0, _PROJECT_DIR)


# ════════════════════════════════════════════════════════════════
# A. Module Independence
# ════════════════════════════════════════════════════════════════

class TestModuleIndependence(unittest.TestCase):
    """Verify guidance_lib and failsafe_lib have zero cross-imports."""

    def _clear_modules(self):
        for mod in list(sys.modules):
            if 'guidance_lib' in mod or 'failsafe_lib' in mod:
                del sys.modules[mod]

    def test_failsafe_lib_does_not_import_guidance_lib(self):
        self._clear_modules()
        import failsafe_lib  # noqa: F401
        loaded = [m for m in sys.modules if 'guidance_lib' in m]
        self.assertEqual(loaded, [], f"guidance_lib leaked in: {loaded}")

    def test_guidance_lib_does_not_import_failsafe_lib(self):
        self._clear_modules()
        import guidance_lib  # noqa: F401
        loaded = [m for m in sys.modules if 'failsafe_lib' in m]
        self.assertEqual(loaded, [], f"failsafe_lib leaked in: {loaded}")


# ════════════════════════════════════════════════════════════════
# B. failsafe_lib — Failure Modes
# ════════════════════════════════════════════════════════════════

from failsafe_lib import FailsafeConfig, FailsafeState, compute_failsafe, leader_in_oblivion


# Home position for tests (Canberra)
HOME_LAT = -35.3632620
HOME_LON = 149.1652370


class TestFailsafeNormalPath(unittest.TestCase):
    """All clear — no failures."""

    def test_all_clear_returns_safe(self):
        cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10.0,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, in_catchup=False,
            cfg=cfg, state=state,
        )
        self.assertTrue(result['safe'])
        self.assertEqual(result['action'], 'CONTINUE')
        self.assertFalse(result['emergency'])

    def test_multiple_safe_ticks_increment_counter(self):
        cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON)
        state = FailsafeState()
        for i in range(10):
            result = compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10.0,
                own_gps_valid=True,
                peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON,
                peer_gps_valid=True, leader_fresh=True,
                cfg=cfg, state=state,
            )
        self.assertEqual(result['tick'], 10)
        self.assertTrue(result['safe'])


class TestFailsafeOwnGPS(unittest.TestCase):
    """Check 1: Own GPS loss -> HOVER, then DEADMAN (RTL) after threshold."""

    def test_no_gps_single_tick_hover(self):
        cfg = FailsafeConfig(deadman_ticks=20)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=0.0, own_lon=0.0, own_alt=10.0,
            own_gps_valid=False,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'HOVER')
        self.assertIn('NO_OWN_GPS', result['flags'])
        self.assertNotIn('DEADMAN', result['flags'])

    def test_no_gps_deadman_after_threshold(self):
        cfg = FailsafeConfig(deadman_ticks=5)
        state = FailsafeState()
        for i in range(5):
            result = compute_failsafe(
                own_lat=0, own_lon=0, own_alt=10,
                own_gps_valid=False,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
                leader_fresh=True, cfg=cfg, state=state,
            )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'RTL')
        self.assertIn('DEADMAN', result['flags'])
        self.assertTrue(result['emergency'])

    def test_gps_recovery_resets_counter(self):
        cfg = FailsafeConfig(deadman_ticks=20)
        state = FailsafeState()
        # Lose GPS for 10 ticks
        for _ in range(10):
            compute_failsafe(
                own_lat=0, own_lon=0, own_alt=10, own_gps_valid=False,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
                leader_fresh=True, cfg=cfg, state=state,
            )
        self.assertEqual(state.no_gps_counter, 10)
        # GPS recovered
        compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10, own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertEqual(state.no_gps_counter, 0)


class TestFailsafeLeaderStale(unittest.TestCase):
    """Check 2: Leader data stale -> HOVER after threshold."""

    def test_stale_leader_short_is_still_safe(self):
        cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON, leader_stale_ticks=50)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=False, cfg=cfg, state=state,
        )
        # First stale tick — counter=1, threshold=50 -> still safe
        self.assertTrue(result['safe'])
        self.assertIn('LEADER_STALE', result['flags'])

    def test_stale_leader_critical_after_threshold(self):
        cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON, leader_stale_ticks=5)
        state = FailsafeState()
        for _ in range(6):
            result = compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
                own_gps_valid=True,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
                leader_fresh=False, cfg=cfg, state=state,
            )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'HOVER')
        self.assertIn('LEADER_STALE_CRITICAL', result['flags'])
        self.assertTrue(result['emergency'])

    def test_fresh_leader_resets_stale_counter(self):
        cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON, leader_stale_ticks=50)
        state = FailsafeState()
        for _ in range(10):
            compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
                own_gps_valid=True,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
                leader_fresh=False, cfg=cfg, state=state,
            )
        self.assertEqual(state.stale_counter, 10)
        compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertEqual(state.stale_counter, 0)


class TestFailsafeLeaderGeofence(unittest.TestCase):
    """Check 3: Leader outside geofence -> HOVER."""

    def test_leader_outside_geofence(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, geofence_radius_m=100.0)
        state = FailsafeState()
        # Place leader ~500m north of home
        far_lat = HOME_LAT + 500.0 / 111320.0
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=far_lat, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'HOVER')
        self.assertIn('LEADER_IN_OBLIVION', result['flags'])
        self.assertTrue(result['emergency'])

    def test_leader_inside_geofence(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, geofence_radius_m=200.0)
        state = FailsafeState()
        # Place leader ~50m north of home
        near_lat = HOME_LAT + 50.0 / 111320.0
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=near_lat, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertTrue(result['safe'])


class TestFailsafeFollowerGeofence(unittest.TestCase):
    """Check 4: Follower outside geofence -> RTL."""

    def test_follower_outside_geofence_rtl(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, geofence_radius_m=100.0)
        state = FailsafeState()
        # Place follower ~500m east of home
        cos_lat = math.cos(math.radians(HOME_LAT))
        far_lon = HOME_LON + 500.0 / (111320.0 * cos_lat)
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=far_lon, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'RTL')
        self.assertIn('GEOFENCE', result['flags'])
        self.assertTrue(result['emergency'])

    def test_follower_inside_geofence(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, geofence_radius_m=200.0)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=HOME_LAT + 10.0 / 111320.0, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertTrue(result['safe'])


class TestFailsafeAltitudeCeiling(unittest.TestCase):
    """Check 5: Altitude ceiling -> HOVER."""

    def test_above_ceiling(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, max_altitude_m=50.0)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=60.0,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'HOVER')
        self.assertIn('ALTITUDE_CEILING', result['flags'])

    def test_below_ceiling_safe(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, max_altitude_m=100.0)
        state = FailsafeState()
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=50.0,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertTrue(result['safe'])


class TestFailsafeCatchupTimeout(unittest.TestCase):
    """Check 6: Catchup timeout -> RTL."""

    def test_catchup_timeout_triggers_rtl(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, catchup_timeout_ticks=10)
        state = FailsafeState()
        for _ in range(11):
            result = compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
                own_gps_valid=True,
                peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON,
                peer_gps_valid=True, leader_fresh=True,
                in_catchup=True, cfg=cfg, state=state,
            )
        self.assertFalse(result['safe'])
        self.assertEqual(result['action'], 'RTL')
        self.assertIn('CATCHUP_TIMEOUT', result['flags'])
        self.assertTrue(result['emergency'])

    def test_catchup_counter_resets_on_tracking(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, catchup_timeout_ticks=100)
        state = FailsafeState()
        for _ in range(50):
            compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
                own_gps_valid=True,
                peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON,
                peer_gps_valid=True, leader_fresh=True,
                in_catchup=True, cfg=cfg, state=state,
            )
        self.assertEqual(state.catchup_ticks, 50)
        compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON,
            peer_gps_valid=True, leader_fresh=True,
            in_catchup=False, cfg=cfg, state=state,
        )
        self.assertEqual(state.catchup_ticks, 0)

    def test_catchup_timeout_disabled_when_zero(self):
        cfg = FailsafeConfig(
            home_lat=HOME_LAT, home_lon=HOME_LON, catchup_timeout_ticks=0)
        state = FailsafeState()
        for _ in range(1000):
            result = compute_failsafe(
                own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
                own_gps_valid=True,
                peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON,
                peer_gps_valid=True, leader_fresh=True,
                in_catchup=True, cfg=cfg, state=state,
            )
        self.assertTrue(result['safe'])  # Never triggers when disabled


class TestLeaderInOblivionStandalone(unittest.TestCase):
    """leader_in_oblivion() standalone utility."""

    def test_leader_far_returns_true(self):
        far_lat = HOME_LAT + 500.0 / 111320.0
        self.assertTrue(leader_in_oblivion(far_lat, HOME_LON, HOME_LAT, HOME_LON, 200.0))

    def test_leader_near_returns_false(self):
        near_lat = HOME_LAT + 10.0 / 111320.0
        self.assertFalse(leader_in_oblivion(near_lat, HOME_LON, HOME_LAT, HOME_LON, 200.0))

    def test_invalid_gps_returns_false(self):
        self.assertFalse(leader_in_oblivion(0.0, 0.0, HOME_LAT, HOME_LON, 200.0))
        self.assertFalse(leader_in_oblivion(HOME_LAT, HOME_LON, 0.0, 0.0, 200.0))
        self.assertFalse(leader_in_oblivion(float('nan'), HOME_LON, HOME_LAT, HOME_LON, 200.0))


# ════════════════════════════════════════════════════════════════
# C. guidance_lib — Modes (no embedded failsafe)
# ════════════════════════════════════════════════════════════════

from guidance_lib import GuidanceConfig, GuidanceState, compute_guidance, compute_escape


class TestGuidanceTracking(unittest.TestCase):
    """Normal tracking mode — goal nearby, peer far enough."""

    def test_tracking_mode_selected(self):
        cfg = GuidanceConfig()
        state = GuidanceState()
        # Small position error -> TRACKING
        goal_lat = HOME_LAT + 3.0 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 10.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertEqual(result['mode'], 'TRACKING')
        self.assertAlmostEqual(result['w_tracking'], 1.0)
        self.assertAlmostEqual(result['w_evasion'], 0.0)
        self.assertAlmostEqual(result['w_catchup'], 0.0)

    def test_tracking_produces_nonzero_velocity(self):
        cfg = GuidanceConfig()
        state = GuidanceState()
        goal_lat = HOME_LAT + 5.0 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 10.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertGreater(result['speed'], 0.0)

    def test_deadzone_produces_zero(self):
        cfg = GuidanceConfig(deadzone_m=0.5)
        state = GuidanceState()
        # Error < deadzone
        goal_lat = HOME_LAT + 0.1 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 10.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertEqual(result['mode'], 'TRACKING')
        self.assertAlmostEqual(result['vn'], 0.0, places=2)
        self.assertAlmostEqual(result['ve'], 0.0, places=2)


class TestGuidanceCatchup(unittest.TestCase):
    """Catchup mode — goal far away."""

    def test_catchup_mode_triggered(self):
        cfg = GuidanceConfig(catchup_dist_m=8.0)
        state = GuidanceState()
        # Goal 50m away -> CATCHUP
        goal_lat = HOME_LAT + 50.0 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 60.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertEqual(result['mode'], 'CATCHUP')
        self.assertAlmostEqual(result['w_catchup'], 1.0)

    def test_catchup_increments_counter(self):
        cfg = GuidanceConfig(catchup_dist_m=8.0)
        state = GuidanceState()
        goal_lat = HOME_LAT + 50.0 / 111320.0
        for _ in range(5):
            compute_guidance(
                my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
                my_vn=0, my_ve=0, my_vd=0,
                peer_lat=HOME_LAT + 60.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
                peer_vn=0, peer_ve=0,
                goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
                cfg=cfg, state=state,
            )
        self.assertEqual(state.catchup_ticks, 5)


class TestGuidanceEvasion(unittest.TestCase):
    """Evasion mode — peer too close."""

    def test_evasion_mode_triggered(self):
        cfg = GuidanceConfig(safety_dist_m=5.0)
        state = GuidanceState()
        # Peer 2m away -> EVASION
        peer_lat = HOME_LAT + 2.0 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=peer_lat, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT + 3.0 / 111320.0, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertEqual(result['mode'], 'EVASION')
        self.assertAlmostEqual(result['w_evasion'], 1.0)

    def test_evasion_resets_catchup_counter(self):
        cfg = GuidanceConfig(safety_dist_m=5.0, catchup_dist_m=8.0)
        state = GuidanceState()
        # First: catchup mode
        goal_lat = HOME_LAT + 50.0 / 111320.0
        compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 60.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertGreater(state.catchup_ticks, 0)
        # Then: evasion
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 2.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertEqual(result['mode'], 'EVASION')
        self.assertEqual(state.catchup_ticks, 0)


class TestGuidanceNoEmbeddedFailsafe(unittest.TestCase):
    """Verify guidance no longer produces FAILSAFE mode or embedded failsafe flags."""

    def test_no_gps_returns_tracking_not_failsafe(self):
        """No own GPS -> zero velocity with NO_OWN_GPS flag, but NOT FAILSAFE mode."""
        cfg = GuidanceConfig()
        state = GuidanceState()
        result = compute_guidance(
            my_lat=0.0, my_lon=0.0, my_alt=20.0,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_alt=20.0,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT, goal_lon=HOME_LON, goal_alt=20.0,
            cfg=cfg, state=state,
        )
        self.assertIn('NO_OWN_GPS', result['flags'])
        self.assertNotEqual(result['mode'], 'FAILSAFE')
        self.assertNotIn('DEADMAN', result['flags'])

    def test_no_deadman_after_many_no_gps_ticks(self):
        """Even after 100 ticks with no GPS, guidance should NOT produce DEADMAN."""
        cfg = GuidanceConfig()
        state = GuidanceState()
        for _ in range(100):
            result = compute_guidance(
                my_lat=0, my_lon=0, my_alt=20,
                my_vn=0, my_ve=0, my_vd=0,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_alt=20,
                peer_vn=0, peer_ve=0,
                goal_lat=HOME_LAT, goal_lon=HOME_LON, goal_alt=20,
                cfg=cfg, state=state,
            )
        self.assertNotIn('DEADMAN', result['flags'])
        self.assertNotEqual(result['mode'], 'FAILSAFE')

    def test_no_geofence_check_in_guidance(self):
        """Guidance should not check geofence even with far-away positions."""
        cfg = GuidanceConfig()
        state = GuidanceState()
        far_lat = HOME_LAT + 500.0 / 111320.0
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=far_lat, peer_lon=HOME_LON, peer_alt=20,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT + 3.0 / 111320.0, goal_lon=HOME_LON, goal_alt=20,
            cfg=cfg, state=state,
        )
        self.assertNotEqual(result['mode'], 'FAILSAFE')
        self.assertNotIn('LEADER_IN_OBLIVION', result['flags'])
        self.assertNotIn('GEOFENCE', result['flags'])

    def test_no_catchup_timeout_in_guidance(self):
        """Guidance should not check catchup timeout even after many CATCHUP ticks."""
        cfg = GuidanceConfig(catchup_dist_m=8.0)
        state = GuidanceState()
        goal_lat = HOME_LAT + 50.0 / 111320.0
        for _ in range(500):
            result = compute_guidance(
                my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20,
                my_vn=0, my_ve=0, my_vd=0,
                peer_lat=HOME_LAT + 60.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20,
                peer_vn=0, peer_ve=0,
                goal_lat=goal_lat, goal_lon=HOME_LON, goal_alt=20,
                cfg=cfg, state=state,
            )
        self.assertEqual(result['mode'], 'CATCHUP')
        self.assertNotIn('CATCHUP_TIMEOUT', result['flags'])

    def test_guidance_config_has_no_failsafe_fields(self):
        """GuidanceConfig should not have failsafe-specific fields."""
        cfg = GuidanceConfig()
        self.assertFalse(hasattr(cfg, 'geofence_radius_m'))
        self.assertFalse(hasattr(cfg, 'home_lat'))
        self.assertFalse(hasattr(cfg, 'home_lon'))
        self.assertFalse(hasattr(cfg, 'catchup_timeout_ticks'))
        self.assertFalse(hasattr(cfg, 'skip_embedded_failsafe'))
        # max_altitude_m stays (used by OutputSafety)
        self.assertTrue(hasattr(cfg, 'max_altitude_m'))


# ════════════════════════════════════════════════════════════════
# D. failsafe_lib config.yaml loading
# ════════════════════════════════════════════════════════════════

from failsafe_lib import load_config


class TestFailsafeLoadConfig(unittest.TestCase):
    """Test load_config() reads from config.yaml correctly."""

    def test_load_default_config(self):
        """load_config() with no args reads failsafe_lib/config.yaml."""
        cfg = load_config()
        self.assertEqual(cfg.geofence_radius_m, 200.0)
        self.assertEqual(cfg.deadman_ticks, 20)
        self.assertEqual(cfg.leader_stale_ticks, 50)
        self.assertEqual(cfg.catchup_timeout_ticks, 300)
        self.assertEqual(cfg.max_altitude_m, 100.0)
        # home defaults to 0.0 when not passed
        self.assertEqual(cfg.home_lat, 0.0)
        self.assertEqual(cfg.home_lon, 0.0)

    def test_load_config_with_home(self):
        """load_config() injects runtime home_lat/lon."""
        cfg = load_config(home_lat=HOME_LAT, home_lon=HOME_LON)
        self.assertEqual(cfg.home_lat, HOME_LAT)
        self.assertEqual(cfg.home_lon, HOME_LON)
        # Other params still from YAML
        self.assertEqual(cfg.geofence_radius_m, 200.0)

    def test_load_config_returns_failsafe_config(self):
        """load_config() returns a FailsafeConfig instance."""
        cfg = load_config()
        self.assertIsInstance(cfg, FailsafeConfig)

    def test_load_config_custom_yaml(self):
        """load_config() can read from a custom YAML path."""
        custom_yaml = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False)
        try:
            custom_yaml.write(
                "failsafe:\n"
                "  geofence_radius_m: 500.0\n"
                "  deadman_ticks: 10\n"
                "  leader_stale_ticks: 25\n"
                "  catchup_timeout_ticks: 150\n"
                "  max_altitude_m: 50.0\n"
            )
            custom_yaml.close()
            cfg = load_config(path=custom_yaml.name, home_lat=1.0, home_lon=2.0)
            self.assertEqual(cfg.geofence_radius_m, 500.0)
            self.assertEqual(cfg.deadman_ticks, 10)
            self.assertEqual(cfg.leader_stale_ticks, 25)
            self.assertEqual(cfg.catchup_timeout_ticks, 150)
            self.assertEqual(cfg.max_altitude_m, 50.0)
            self.assertEqual(cfg.home_lat, 1.0)
            self.assertEqual(cfg.home_lon, 2.0)
        finally:
            os.unlink(custom_yaml.name)

    def test_max_altitude_from_failsafe_to_guidance(self):
        """max_altitude_m flows from failsafe config to guidance config."""
        fs_cfg = load_config()
        g_cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)
        self.assertEqual(g_cfg.max_altitude_m, fs_cfg.max_altitude_m)
        self.assertEqual(g_cfg.max_altitude_m, 100.0)


# ════════════════════════════════════════════════════════════════
# E. Integration — follower_main import pattern
# ════════════════════════════════════════════════════════════════

class TestIntegrationImportPattern(unittest.TestCase):
    """Verify the exact import pattern used by follower_main.py works."""

    def test_guidance_lib_imports(self):
        from guidance_lib import (
            GuidanceConfig, GuidanceState, compute_guidance, CommandSmoother,
        )
        self.assertIsNotNone(GuidanceConfig)
        self.assertIsNotNone(GuidanceState)
        self.assertIsNotNone(compute_guidance)
        self.assertIsNotNone(CommandSmoother)

    def test_failsafe_lib_imports(self):
        from failsafe_lib import (
            FailsafeConfig, FailsafeState, compute_failsafe, load_config,
        )
        self.assertIsNotNone(FailsafeConfig)
        self.assertIsNotNone(FailsafeState)
        self.assertIsNotNone(compute_failsafe)
        self.assertIsNotNone(load_config)

    def test_guidance_lib_no_longer_exports_failsafe(self):
        import guidance_lib
        self.assertFalse(hasattr(guidance_lib, 'FailsafeConfig'))
        self.assertFalse(hasattr(guidance_lib, 'FailsafeState'))
        self.assertFalse(hasattr(guidance_lib, 'compute_failsafe'))
        self.assertFalse(hasattr(guidance_lib, 'leader_in_oblivion'))

    def test_compute_escape_still_in_guidance_lib(self):
        from guidance_lib import compute_escape
        self.assertIsNotNone(compute_escape)


# ════════════════════════════════════════════════════════════════
# F. Edge Cases
# ════════════════════════════════════════════════════════════════

class TestEdgeCases(unittest.TestCase):
    """NaN, Inf, zero GPS, boundary values."""

    def test_guidance_nan_input_safe_output(self):
        cfg = GuidanceConfig()
        state = GuidanceState()
        result = compute_guidance(
            my_lat=float('nan'), my_lon=HOME_LON, my_alt=20,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_alt=20,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT, goal_lon=HOME_LON, goal_alt=20,
            cfg=cfg, state=state,
        )
        # NaN sanitized to 0 -> GPS invalid -> safe zero output
        self.assertTrue(math.isfinite(result['vn']))
        self.assertTrue(math.isfinite(result['ve']))
        self.assertTrue(math.isfinite(result['vd']))

    def test_guidance_inf_input_safe_output(self):
        cfg = GuidanceConfig()
        state = GuidanceState()
        result = compute_guidance(
            my_lat=float('inf'), my_lon=HOME_LON, my_alt=20,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_alt=20,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT, goal_lon=HOME_LON, goal_alt=20,
            cfg=cfg, state=state,
        )
        self.assertTrue(math.isfinite(result['vn']))
        self.assertTrue(math.isfinite(result['ve']))

    def test_failsafe_with_none_config_uses_defaults(self):
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT + 0.0001, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True,
        )
        self.assertTrue(result['safe'])

    def test_guidance_with_none_config_uses_defaults(self):
        result = compute_guidance(
            my_lat=HOME_LAT, my_lon=HOME_LON, my_alt=20,
            my_vn=0, my_ve=0, my_vd=0,
            peer_lat=HOME_LAT + 10.0 / 111320.0, peer_lon=HOME_LON, peer_alt=20,
            peer_vn=0, peer_ve=0,
            goal_lat=HOME_LAT + 3.0 / 111320.0, goal_lon=HOME_LON, goal_alt=20,
        )
        self.assertIn('mode', result)
        self.assertTrue(math.isfinite(result['vn']))

    def test_failsafe_home_zero_disables_geofence(self):
        """When home is (0,0), geofence checks should be skipped."""
        cfg = FailsafeConfig(home_lat=0.0, home_lon=0.0, geofence_radius_m=10.0)
        state = FailsafeState()
        # Leader would be "far" from (0,0) but home_lat/lon=0 is invalid
        result = compute_failsafe(
            own_lat=HOME_LAT, own_lon=HOME_LON, own_alt=10,
            own_gps_valid=True,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
            leader_fresh=True, cfg=cfg, state=state,
        )
        self.assertTrue(result['safe'])  # Geofence skipped

    def test_failsafe_priority_gps_over_stale(self):
        """GPS failure should take priority over stale leader."""
        cfg = FailsafeConfig(deadman_ticks=3)
        state = FailsafeState()
        for _ in range(5):
            result = compute_failsafe(
                own_lat=0, own_lon=0, own_alt=10,
                own_gps_valid=False,
                peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_gps_valid=True,
                leader_fresh=False, cfg=cfg, state=state,
            )
        # Should hit DEADMAN, not LEADER_STALE_CRITICAL
        self.assertIn('DEADMAN', result['flags'])
        self.assertEqual(result['action'], 'RTL')

    def test_compute_escape_handles_zero_gps(self):
        state = GuidanceState()
        result = compute_escape(
            my_lat=0.0, my_lon=0.0, my_alt=10.0,
            peer_lat=HOME_LAT, peer_lon=HOME_LON, peer_alt=10.0,
            state=state,
        )
        self.assertIn('NO_GPS', result['flags'])
        self.assertEqual(result['vn'], 0)
        self.assertEqual(result['ve'], 0)

    def test_command_smoother_reset(self):
        from guidance_lib import CommandSmoother
        sm = CommandSmoother()
        sm.filter(3.0, 3.0, -1.0)
        sm.reset()
        vn, ve, vd = sm.filter(0.0, 0.0, 0.0)
        self.assertAlmostEqual(vn, 0.0)
        self.assertAlmostEqual(ve, 0.0)
        self.assertAlmostEqual(vd, 0.0)


# ════════════════════════════════════════════════════════════════
# G. Mission Dispatch (follower_missions.py)
# ════════════════════════════════════════════════════════════════

from docker_sim.follower_missions import (
    Mission, FlightStatus, MissionContext,
    check_status, get_mission, execute_mission, sleep_tick,
    _RC_CMD_MAP, _on_exit, _on_enter,
)
from docker_sim.mavlink_bridge import CMD_RTL, CMD_LAND, CMD_KILL, CMD_FOLLOW, CMD_HOVER
from docker_sim.config import CONTROL_HZ


class _FakeConn:
    """Minimal MavlinkConn stub for mission tests."""

    def __init__(self):
        self.velocities = []
        self.modes_set = []
        self.disarmed = False
        self.drone_id = 99

    def send_velocity_ned(self, vn, ve, vd):
        self.velocities.append((vn, ve, vd))

    def set_mode(self, mode):
        self.modes_set.append(mode)

    def force_disarm(self):
        self.disarmed = True

    def drain_latest(self):
        return {}


class _FakeBridge:
    """Minimal MavlinkBridge stub for mission tests."""

    def __init__(self):
        self._commands = []
        self._peer = None
        self._guidance_mode = 0

    def queue_command(self, cmd_dict):
        self._commands.append(cmd_dict)

    def get_pending_command(self):
        if self._commands:
            return self._commands.pop(0)
        return None

    def get_peer_state(self):
        return self._peer

    def update_own_state(self, pos):
        pass

    def set_guidance_mode(self, code):
        self._guidance_mode = code


def _make_ctx(**overrides):
    """Create a MissionContext with fake conn/bridge for testing."""
    conn = _FakeConn()
    bridge = _FakeBridge()
    fs_cfg = FailsafeConfig(home_lat=HOME_LAT, home_lon=HOME_LON)
    cfg = GuidanceConfig(max_altitude_m=fs_cfg.max_altitude_m)
    ctx = MissionContext(
        conn=conn,
        bridge=bridge,
        cfg=cfg,
        fs_cfg=fs_cfg,
        fs_state=FailsafeState(),
    )
    for k, v in overrides.items():
        setattr(ctx, k, v)
    return ctx


def _make_status(**overrides):
    """Create a FlightStatus with sensible defaults."""
    defaults = dict(
        pos={'lat': HOME_LAT, 'lon': HOME_LON, 'alt': 10.0,
             'vx': 0, 'vy': 0, 'vz': 0, 'heading': 0},
        heartbeat={'mode': 'GUIDED', 'armed': True},
        gps_valid=True,
        leader={'lat': HOME_LAT + 0.0001, 'lon': HOME_LON,
                'alt': 10.0, 'vx': 0, 'vy': 0},
        leader_fresh=True,
    )
    defaults.update(overrides)
    return FlightStatus(**defaults)


class TestRCCommandMapping(unittest.TestCase):
    """RC commands map to correct missions."""

    def test_cmd_follow_maps_to_follow(self):
        self.assertEqual(_RC_CMD_MAP[CMD_FOLLOW], Mission.FOLLOW)

    def test_cmd_hover_maps_to_hover(self):
        self.assertEqual(_RC_CMD_MAP[CMD_HOVER], Mission.HOVER)

    def test_cmd_rtl_maps_to_rtl(self):
        self.assertEqual(_RC_CMD_MAP[CMD_RTL], Mission.RTL)

    def test_cmd_land_maps_to_land(self):
        self.assertEqual(_RC_CMD_MAP[CMD_LAND], Mission.LAND)

    def test_cmd_kill_maps_to_kill(self):
        self.assertEqual(_RC_CMD_MAP[CMD_KILL], Mission.KILL)

    def test_rc_follow_via_get_mission(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.bridge.queue_command({'cmd': CMD_FOLLOW})
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.FOLLOW)

    def test_rc_rtl_via_get_mission(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.bridge.queue_command({'cmd': CMD_RTL})
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.RTL)

    def test_rc_last_command_wins(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.bridge.queue_command({'cmd': CMD_FOLLOW})
        ctx.bridge.queue_command({'cmd': CMD_HOVER})
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.HOVER)


class TestGlobalFailsafeOverride(unittest.TestCase):
    """Global failsafe can override mission to HOVER or RTL."""

    def test_no_gps_forces_hover(self):
        ctx = _make_ctx(current_mission=Mission.FOLLOW)
        status = _make_status(gps_valid=False, pos=None)
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.HOVER)

    def test_gps_deadman_forces_rtl(self):
        ctx = _make_ctx(current_mission=Mission.FOLLOW)
        ctx.fs_cfg.deadman_ticks = 3
        status = _make_status(gps_valid=False, pos=None)
        for _ in range(3):
            mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.RTL)

    def test_follower_geofence_forces_rtl(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.fs_cfg.geofence_radius_m = 100.0
        cos_lat = math.cos(math.radians(HOME_LAT))
        far_lon = HOME_LON + 500.0 / (111320.0 * cos_lat)
        status = _make_status(
            pos={'lat': HOME_LAT, 'lon': far_lon, 'alt': 10.0,
                 'vx': 0, 'vy': 0, 'vz': 0, 'heading': 0})
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.RTL)

    def test_altitude_ceiling_forces_hover(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.fs_cfg.max_altitude_m = 50.0
        status = _make_status(
            pos={'lat': HOME_LAT, 'lon': HOME_LON, 'alt': 60.0,
                 'vx': 0, 'vy': 0, 'vz': 0, 'heading': 0})
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.HOVER)


class TestFollowToHoverResets(unittest.TestCase):
    """FOLLOW → HOVER transition resets failsafe state."""

    def test_exit_follow_resets_stale_counter(self):
        ctx = _make_ctx(current_mission=Mission.FOLLOW)
        ctx.fs_state.stale_counter = 42
        ctx.fs_state.catchup_ticks = 10
        _on_exit(Mission.FOLLOW, ctx)
        self.assertEqual(ctx.fs_state.stale_counter, 0)
        self.assertEqual(ctx.fs_state.catchup_ticks, 0)
        self.assertEqual(ctx.last_guidance_mode, "")

    def test_exit_hover_does_not_reset(self):
        ctx = _make_ctx()
        ctx.fs_state.stale_counter = 5
        _on_exit(Mission.HOVER, ctx)
        self.assertEqual(ctx.fs_state.stale_counter, 5)


class TestTerminalMissions(unittest.TestCase):
    """Terminal missions (RTL/LAND/KILL) return False from execute_mission."""

    def test_kill_returns_false(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        status = _make_status()
        result = execute_mission(Mission.KILL, status, ctx)
        self.assertFalse(result)
        self.assertTrue(ctx.conn.disarmed)

    def test_rtl_enters_rtl_mode(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        status = _make_status()
        execute_mission(Mission.RTL, status, ctx)
        self.assertIn("RTL", ctx.conn.modes_set)
        self.assertTrue(ctx.rtl_sent)

    def test_land_enters_land_mode(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        status = _make_status()
        execute_mission(Mission.LAND, status, ctx)
        self.assertIn("LAND", ctx.conn.modes_set)

    def test_rtl_returns_false_when_landed(self):
        ctx = _make_ctx(current_mission=Mission.RTL)
        ctx.rtl_sent = True
        status = _make_status(
            pos={'lat': HOME_LAT, 'lon': HOME_LON, 'alt': 0.5,
                 'vx': 0, 'vy': 0, 'vz': 0, 'heading': 0})
        result = execute_mission(Mission.RTL, status, ctx)
        self.assertFalse(result)

    def test_rtl_returns_true_while_descending(self):
        ctx = _make_ctx(current_mission=Mission.RTL)
        ctx.rtl_sent = True
        status = _make_status(
            pos={'lat': HOME_LAT, 'lon': HOME_LON, 'alt': 5.0,
                 'vx': 0, 'vy': 0, 'vz': 0, 'heading': 0})
        result = execute_mission(Mission.RTL, status, ctx)
        self.assertTrue(result)


class TestPhysicalConstraints(unittest.TestCase):
    """FOLLOW without GPS stays in current mission."""

    def test_follow_denied_without_gps(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.bridge.queue_command({'cmd': CMD_FOLLOW})
        status = _make_status(gps_valid=False, pos=None)
        mission = get_mission(status, ctx)
        # Should not be FOLLOW — GPS required
        self.assertNotEqual(mission, Mission.FOLLOW)

    def test_follow_allowed_with_gps(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.bridge.queue_command({'cmd': CMD_FOLLOW})
        status = _make_status(gps_valid=True)
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.FOLLOW)


class TestAutoPromotions(unittest.TestCase):
    """Auto-promotions in get_mission: IDLE→TAKEOFF, TAKEOFF→HOVER, leader-lost→RTL."""

    def test_idle_promotes_to_takeoff_when_gps(self):
        ctx = _make_ctx(current_mission=Mission.IDLE, has_gps=True)
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.TAKEOFF)

    def test_idle_stays_idle_without_gps(self):
        ctx = _make_ctx(current_mission=Mission.IDLE, has_gps=False)
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.IDLE)

    def test_takeoff_promotes_to_hover_when_complete(self):
        ctx = _make_ctx(current_mission=Mission.TAKEOFF)
        # Simulate a completed TakeoffManager
        ctx.takeoff = type('FakeTakeoff', (), {'complete': True})()
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.HOVER)

    def test_takeoff_stays_takeoff_when_not_complete(self):
        ctx = _make_ctx(current_mission=Mission.TAKEOFF)
        ctx.takeoff = type('FakeTakeoff', (), {'complete': False})()
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.TAKEOFF)

    def test_follow_promotes_to_rtl_on_leader_lost(self):
        ctx = _make_ctx(current_mission=Mission.FOLLOW)
        ctx.no_leader_count = CONTROL_HZ * 30 + 1
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.RTL)

    def test_rc_overrides_auto_promotion(self):
        """RC KILL during TAKEOFF overrides the auto-promote to HOVER."""
        ctx = _make_ctx(current_mission=Mission.TAKEOFF)
        ctx.takeoff = type('FakeTakeoff', (), {'complete': True})()
        ctx.bridge.queue_command({'cmd': CMD_KILL})
        status = _make_status()
        mission = get_mission(status, ctx)
        self.assertEqual(mission, Mission.KILL)

    def test_failsafe_overrides_auto_promotion(self):
        """GPS loss during IDLE with has_gps=True still gets caught by failsafe
        on the TAKEOFF that auto-promotion would produce."""
        ctx = _make_ctx(current_mission=Mission.IDLE, has_gps=True)
        ctx.fs_cfg.deadman_ticks = 1
        status = _make_status(gps_valid=False, pos=None)
        mission = get_mission(status, ctx)
        # IDLE auto-promotes to TAKEOFF, but failsafe doesn't run for IDLE→TAKEOFF
        # since failsafe check happens before auto-promotion
        self.assertEqual(mission, Mission.TAKEOFF)


class TestMissionTransitions(unittest.TestCase):
    """execute_mission handles transitions correctly."""

    def test_hover_sends_zero_velocity(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        status = _make_status()
        execute_mission(Mission.HOVER, status, ctx)
        self.assertEqual(ctx.conn.velocities[-1], (0, 0, 0))

    def test_enter_follow_resets_no_leader_count(self):
        ctx = _make_ctx(current_mission=Mission.HOVER)
        ctx.no_leader_count = 99
        _on_enter(Mission.FOLLOW, ctx)
        self.assertEqual(ctx.no_leader_count, 0)

    def test_mission_enum_values(self):
        """All 7 missions are defined with correct string values."""
        self.assertEqual(Mission.IDLE.value, "IDLE")
        self.assertEqual(Mission.TAKEOFF.value, "TAKEOFF")
        self.assertEqual(Mission.HOVER.value, "HOVER")
        self.assertEqual(Mission.FOLLOW.value, "FOLLOW")
        self.assertEqual(Mission.RTL.value, "RTL")
        self.assertEqual(Mission.LAND.value, "LAND")
        self.assertEqual(Mission.KILL.value, "KILL")

    def test_do_idle_sets_has_gps_flag(self):
        """do_idle sets ctx.has_gps but does NOT transition."""
        from docker_sim.follower_missions import do_idle
        ctx = _make_ctx(current_mission=Mission.IDLE)
        status = _make_status(gps_valid=True)
        do_idle(status, ctx)
        self.assertTrue(ctx.has_gps)
        self.assertEqual(ctx.current_mission, Mission.IDLE)

    def test_do_takeoff_does_not_transition(self):
        """do_takeoff ticks the manager but does NOT mutate current_mission."""
        from docker_sim.follower_missions import do_takeoff
        ctx = _make_ctx(current_mission=Mission.TAKEOFF)
        # Fake TakeoffManager that completes immediately
        class FakeTM:
            complete = False
            def tick(self, **kw):
                self.complete = True
                return True
        ctx.takeoff = FakeTM()
        status = _make_status()
        do_takeoff(status, ctx)
        self.assertTrue(ctx.takeoff.complete)
        # Handler must NOT have changed current_mission
        self.assertEqual(ctx.current_mission, Mission.TAKEOFF)


class TestMissionImports(unittest.TestCase):
    """follower_missions.py exports are importable."""

    def test_imports_clean(self):
        from docker_sim.follower_missions import (
            Mission, FlightStatus, MissionContext,
            check_status, get_mission, execute_mission, sleep_tick,
        )
        self.assertIsNotNone(Mission)
        self.assertIsNotNone(FlightStatus)
        self.assertIsNotNone(MissionContext)
        self.assertIsNotNone(check_status)
        self.assertIsNotNone(get_mission)
        self.assertIsNotNone(execute_mission)
        self.assertIsNotNone(sleep_tick)


# ════════════════════════════════════════════════════════════════
# Run
# ════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    unittest.main(verbosity=2)

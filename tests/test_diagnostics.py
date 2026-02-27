"""Tests that wrap gcs/diagnostics.py — verify all algorithm diagnostics pass."""

import pytest
from gcs import diagnostics as diag


def _assert_diagnostic(result):
    """Common assertion: diagnostic returned 'pass' with no failing checks."""
    assert result["status"] == "pass", (
        f"{result['test_name']} failed: "
        + ", ".join(
            f"{a['check']}={a['value']}"
            for a in result.get("assertions", [])
            if not a["passed"]
        )
    )


def test_diag_formation_geometry():
    _assert_diagnostic(diag.test_formation_geometry())


def test_diag_failsafe_transitions():
    _assert_diagnostic(diag.test_failsafe_transitions())


def test_diag_leader_election():
    _assert_diagnostic(diag.test_leader_election())


def test_diag_ned_conversion():
    _assert_diagnostic(diag.test_ned_conversion())


def test_diag_local_planner_states():
    _assert_diagnostic(diag.test_local_planner_states())


def test_diag_mppi_controller():
    _assert_diagnostic(diag.test_mppi_controller())


def test_diag_hybrid_astar():
    _assert_diagnostic(diag.test_hybrid_astar())

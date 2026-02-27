"""Tests for gcs/command_dispatcher.py — command routing logic."""

from unittest.mock import MagicMock
from gcs.command_dispatcher import CommandDispatcher


class TestSendToAll:
    def _make_dispatcher(self, num_drones=5, known_drones=None):
        udp = MagicMock()
        dispatcher = CommandDispatcher(udp, num_drones, known_drones=known_drones)
        return dispatcher, udp

    def test_send_to_all_empty_set(self):
        """drone_ids=set() should send to nobody (empty set is falsy but explicit)."""
        disp, udp = self._make_dispatcher()
        disp._send_to_all("TAKEOFF_CMD", {"alt": 10}, drone_ids=set())
        udp.send.assert_not_called()

    def test_send_to_all_none_uses_known(self):
        """drone_ids=None should fall back to _known_drones."""
        known = {2, 4}
        disp, udp = self._make_dispatcher(known_drones=known)
        disp._send_to_all("TAKEOFF_CMD", {"alt": 10}, drone_ids=None)
        sent_ids = {call.args[2] for call in udp.send.call_args_list}
        # Should send to ports for drones 2 and 4
        assert udp.send.call_count == 2

    def test_send_to_all_fallback_range(self):
        """When both drone_ids and _known_drones are None, fall back to 1..N."""
        disp, udp = self._make_dispatcher(num_drones=3, known_drones=None)
        disp._send_to_all("LAND_CMD", {}, drone_ids=None)
        assert udp.send.call_count == 3

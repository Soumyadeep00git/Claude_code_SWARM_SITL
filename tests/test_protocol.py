"""Tests for comms/protocol.py — message serialization."""

import json
import time
import pytest
from comms.protocol import (
    make_msg, parse_msg, encode_msg, ALL_TYPES,
    TAKEOFF_CMD, LAND_CMD, STATE_REPORT, FORMATION_CMD,
    PEER_HEARTBEAT, PROXIMITY_ALERT, RL_MODE_CMD,
)


class TestMakeMsg:
    def test_returns_bytes(self):
        raw = make_msg(TAKEOFF_CMD, 0, {"alt": 10})
        assert isinstance(raw, bytes)

    def test_valid_json(self):
        raw = make_msg(STATE_REPORT, 1, {"lat": -35.363, "lon": 149.165})
        msg = json.loads(raw.decode("utf-8"))
        assert "type" in msg
        assert "src" in msg
        assert "ts" in msg
        assert "data" in msg

    def test_type_preserved(self):
        raw = make_msg(FORMATION_CMD, 0, {"formation": "V"})
        msg = json.loads(raw)
        assert msg["type"] == "FORMATION_CMD"

    def test_src_preserved(self):
        raw = make_msg(TAKEOFF_CMD, 0, {"alt": 10})
        msg = json.loads(raw)
        assert msg["src"] == 0

    def test_data_preserved(self):
        data = {"alt": 15.5, "drone_id": 3}
        raw = make_msg(TAKEOFF_CMD, 0, data)
        msg = json.loads(raw)
        assert msg["data"]["alt"] == 15.5
        assert msg["data"]["drone_id"] == 3

    def test_timestamp_is_recent(self):
        before = time.time()
        raw = make_msg(LAND_CMD, 0, {})
        after = time.time()
        msg = json.loads(raw)
        assert before <= msg["ts"] <= after

    def test_all_message_types_serialize(self):
        for msg_type in ALL_TYPES:
            raw = make_msg(msg_type, 1, {"test": True})
            msg = json.loads(raw)
            assert msg["type"] == msg_type


class TestParseMsg:
    def test_round_trip(self):
        data = {"lat": -35.363, "lon": 149.165, "alt": 10.0}
        raw = make_msg(STATE_REPORT, 1, data)
        msg = parse_msg(raw)
        assert msg is not None
        assert msg["type"] == STATE_REPORT
        assert msg["src"] == 1
        assert msg["data"]["lat"] == -35.363

    def test_malformed_json(self):
        assert parse_msg(b"not json at all") is None

    def test_missing_fields(self):
        raw = json.dumps({"type": "TAKEOFF_CMD"}).encode()
        assert parse_msg(raw) is None

    def test_missing_data_field(self):
        raw = json.dumps({"type": "TAKEOFF_CMD", "src": 0}).encode()
        assert parse_msg(raw) is None

    def test_empty_bytes(self):
        assert parse_msg(b"") is None

    def test_invalid_utf8(self):
        assert parse_msg(b"\xff\xfe") is None

    def test_extra_fields_preserved(self):
        msg = {"type": "STATE_REPORT", "src": 1, "ts": 0, "data": {}, "extra": 42}
        raw = json.dumps(msg).encode()
        parsed = parse_msg(raw)
        assert parsed is not None
        assert parsed["extra"] == 42


class TestEncodeMsg:
    def test_encode_msg_roundtrip(self):
        msg = {"type": "STATE_REPORT", "src": 1, "ts": 1234567890.0,
               "data": {"lat": -35.363, "lon": 149.165, "alt": 10.0}}
        raw = encode_msg(msg)
        assert isinstance(raw, bytes)
        parsed = parse_msg(raw)
        assert parsed is not None
        assert parsed["type"] == msg["type"]
        assert parsed["src"] == msg["src"]
        assert parsed["ts"] == msg["ts"]
        assert parsed["data"]["lat"] == msg["data"]["lat"]

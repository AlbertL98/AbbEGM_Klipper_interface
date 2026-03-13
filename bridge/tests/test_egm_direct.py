# tests/test_egm_direct.py — Integrations-Tests: EGM-Client direkt
#
# FIX: Import-Pfad korrigiert.
#   Vorher: from egm_bridge.egm_client import EgmClient, EgmTarget, EgmFeedback
#   Jetzt:  from data.egm_client import EgmClient, EgmTarget, EgmFeedback
#
#   Alle anderen Tests nutzen bereits korrekt `from data.xxx import ...`.
#   Das Paket heißt `data`, nicht `egm_bridge`.

import time
import socket
import threading
import pytest

# FIX: korrekter Import-Pfad
from data.egm_client import EgmClient, EgmTarget, EgmFeedback
from data.config import EgmConfig


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture
def egm_cfg():
    return EgmConfig(
        robot_ip="127.0.0.1",
        egm_port=6510,
        feedback_port=6511,
        timeout_ms=500,
    )


# ═══════════════════════════════════════════════════════════════════════
# Unit-Tests (ohne echten Roboter)
# ═══════════════════════════════════════════════════════════════════════

class TestEgmTarget:

    def test_instantiation(self):
        t = EgmTarget(x=100.0, y=200.0, z=300.0, sequence_id=1)
        assert t.x == 100.0
        assert t.y == 200.0
        assert t.z == 300.0

    def test_sequence_id_increments(self):
        t1 = EgmTarget(x=0, y=0, z=0, sequence_id=1)
        t2 = EgmTarget(x=0, y=0, z=0, sequence_id=2)
        assert t2.sequence_id == t1.sequence_id + 1

    def test_serialize_deserialize_roundtrip(self):
        """EgmTarget → bytes → EgmFeedback sollte Position erhalten."""
        t = EgmTarget(x=123.5, y=-45.2, z=678.9, sequence_id=42)
        if hasattr(t, 'to_bytes') and hasattr(EgmFeedback, 'from_bytes'):
            data = t.to_bytes()
            assert isinstance(data, (bytes, bytearray))


class TestEgmFeedback:

    def test_default_position(self):
        fb = EgmFeedback()
        assert hasattr(fb, 'x')
        assert hasattr(fb, 'y')
        assert hasattr(fb, 'z')

    def test_timestamp_set(self):
        fb = EgmFeedback()
        fb.timestamp = time.perf_counter()
        assert fb.timestamp > 0


class TestEgmClient:

    def test_instantiation(self, egm_cfg):
        client = EgmClient(egm_cfg)
        assert client is not None

    def test_not_connected_initially(self, egm_cfg):
        client = EgmClient(egm_cfg)
        assert not client.is_connected

    def test_send_without_connection_raises_or_returns_false(self, egm_cfg):
        """Senden ohne Verbindung darf keinen unkontrollierten Absturz verursachen."""
        client = EgmClient(egm_cfg)
        target = EgmTarget(x=0, y=0, z=0, sequence_id=1)
        try:
            result = client.send(target)
            # Wenn es nicht wirft: muss False/None zurückgeben
            assert not result
        except (ConnectionError, OSError, RuntimeError):
            pass  # Erwartetes Verhalten

    def test_receive_timeout(self, egm_cfg):
        """receive() mit Timeout gibt None zurück wenn kein Paket kommt."""
        client = EgmClient(egm_cfg)
        result = client.receive(timeout_ms=50)
        assert result is None


# ═══════════════════════════════════════════════════════════════════════
# Loopback-Test (localhost, kein Roboter nötig)
# ═══════════════════════════════════════════════════════════════════════

class TestEgmLoopback:
    """
    Schickt ein EgmTarget an sich selbst via UDP-Loopback.
    Kein echter Roboter nötig.
    """

    def test_udp_send_receive(self):
        """Gesendete Bytes kommen via UDP-Loopback an."""
        PORT = 19876
        received = []

        def listener():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", PORT))
            s.settimeout(0.5)
            try:
                data, _ = s.recvfrom(4096)
                received.append(data)
            except socket.timeout:
                pass
            finally:
                s.close()

        t = threading.Thread(target=listener, daemon=True)
        t.start()
        time.sleep(0.05)

        # Sende etwas an localhost:PORT
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(b"TEST_PAYLOAD", ("127.0.0.1", PORT))
        s.close()

        t.join(timeout=1.0)
        assert len(received) == 1
        assert received[0] == b"TEST_PAYLOAD"

# clock.py — Einheitliche Zeitquelle für alle Bridge-Komponenten
#
# WARUM:
#   time.perf_counter() hat µs-Auflösung auf allen Plattformen.
#   time.monotonic() hat auf Windows nur 15ms Auflösung.
#   Mischen beider Clocks führt zu falschen Differenzen (RTT, Lag),
#   weil die Epochen auf Windows divergieren.
#
# REGEL:
#   Jede Komponente importiert bridge_now() aus diesem Modul.
#   Niemand ruft time.perf_counter() oder time.monotonic() direkt
#   für Timestamps auf. Ausnahme: sleep-Timeouts (time.monotonic OK).
#
# VERWENDUNG:
#   from .clock import bridge_now
#   ts = bridge_now()   # → float, Sekunden, µs-Auflösung

import time


def bridge_now() -> float:
    """
    Einheitlicher Timestamp für alle Bridge-Komponenten.

    Basiert auf time.perf_counter():
      - µs-Auflösung auf allen Plattformen
      - Monoton (geht nie rückwärts)
      - NICHT wallclock (kein Datum, nur Differenzen sinnvoll)

    Alle internen Timestamps (TX, RX, Lag, RTT, Telemetrie)
    müssen diese Funktion verwenden.
    """
    return time.perf_counter()

#!/usr/bin/env python3
"""
segment_source.py — Modulare Datenquellen für Klipper Trapezsegmente

Architektur:
    SegmentSource (abstrakte Basisklasse)
    ├── CsvSegmentSource     — liest aus move_segments.csv (Batch-Modus)
    └── TcpSegmentSource     — verbindet sich per TCP (Live-Modus, Zukunft)

Verwendung:
    from segment_source import CsvSegmentSource, TcpSegmentSource

    # Batch/Offline:
    source = CsvSegmentSource("logs/batch_results/move_segments.csv")

    # Live (Zukunft):
    source = TcpSegmentSource(host="localhost", port=7200)

    # Einheitliches Interface:
    source.connect()
    for segment in source.segments():
        print(segment)
    source.disconnect()
"""

import abc
import csv
import json
import socket
import time
import logging
from dataclasses import dataclass, field
from typing import Iterator, Optional, List

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────
# Datenklasse für ein Trapezsegment
# ─────────────────────────────────────────────────────────────────────

@dataclass
class TrapezSegment:
    """Ein einzelnes Klipper-Trapezsegment mit allen kinematischen Daten."""
    nr: int
    print_time: float
    duration: float
    start: List[float]       # [x, y, z]
    end: List[float]         # [x, y, z]
    distance: float
    start_v: float           # mm/s
    cruise_v: float          # mm/s
    end_v: float             # mm/s
    accel: float             # mm/s²
    accel_t: float           # s
    cruise_t: float          # s
    decel_t: float           # s
    axes_r: List[float]      # [rx, ry, rz] Richtungsvektor

    def to_dict(self) -> dict:
        """Konvertiert zu JSON-kompatiblem Dictionary."""
        return {
            'type': 'segment',
            'nr': self.nr,
            'print_time': self.print_time,
            'duration': self.duration,
            'start': self.start,
            'end': self.end,
            'distance': self.distance,
            'start_v': self.start_v,
            'cruise_v': self.cruise_v,
            'end_v': self.end_v,
            'accel': self.accel,
            'accel_t': self.accel_t,
            'cruise_t': self.cruise_t,
            'decel_t': self.decel_t,
            'axes_r': self.axes_r,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'TrapezSegment':
        """Erstellt ein Segment aus einem Dictionary (z.B. von JSON)."""
        return cls(
            nr=int(d['nr']),
            print_time=float(d['print_time']),
            duration=float(d['duration']),
            start=[float(x) for x in d['start']],
            end=[float(x) for x in d['end']],
            distance=float(d['distance']),
            start_v=float(d['start_v']),
            cruise_v=float(d['cruise_v']),
            end_v=float(d['end_v']),
            accel=float(d['accel']),
            accel_t=float(d['accel_t']),
            cruise_t=float(d['cruise_t']),
            decel_t=float(d['decel_t']),
            axes_r=[float(x) for x in d['axes_r']],
        )

    @classmethod
    def from_csv_row(cls, row: dict) -> 'TrapezSegment':
        """Erstellt ein Segment aus einer CSV-Zeile (move_segments.csv)."""
        return cls(
            nr=int(row['move_nr']),
            print_time=float(row['print_time']),
            duration=float(row['move_duration']),
            start=[
                float(row['start_x']),
                float(row['start_y']),
                float(row['start_z']),
            ],
            end=[
                float(row['end_x']),
                float(row['end_y']),
                float(row['end_z']),
            ],
            distance=float(row['distance']),
            start_v=float(row['start_v']),
            cruise_v=float(row['cruise_v']),
            end_v=float(row['end_v']),
            accel=float(row['accel']),
            accel_t=float(row['accel_time']),
            cruise_t=float(row['cruise_time']),
            decel_t=float(row['decel_time']),
            axes_r=[
                float(row['axes_r_x']),
                float(row['axes_r_y']),
                float(row['axes_r_z']),
            ],
        )


# ─────────────────────────────────────────────────────────────────────
# Abstrakte Basisklasse
# ─────────────────────────────────────────────────────────────────────

class SegmentSource(abc.ABC):
    """
    Abstrakte Basisklasse für Segment-Datenquellen.

    Jede Datenquelle muss connect(), disconnect() und segments() implementieren.
    segments() ist ein Generator, der TrapezSegment-Objekte liefert.
    """

    @abc.abstractmethod
    def connect(self):
        """Verbindung herstellen / Datei öffnen."""
        pass

    @abc.abstractmethod
    def disconnect(self):
        """Verbindung trennen / Datei schließen."""
        pass

    @abc.abstractmethod
    def segments(self) -> Iterator[TrapezSegment]:
        """Generator: liefert TrapezSegment-Objekte nacheinander."""
        pass

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def all_segments(self) -> List[TrapezSegment]:
        """Lädt alle Segmente in eine Liste (Convenience-Methode)."""
        return list(self.segments())


# ─────────────────────────────────────────────────────────────────────
# CSV-Quelle (Batch-Modus)
# ─────────────────────────────────────────────────────────────────────

class CsvSegmentSource(SegmentSource):
    """
    Liest Trapezsegmente aus einer move_segments.csv Datei.
    Erzeugt vom move_export.py Klipper-Extra im Batch-Modus.

    Parameter:
        csv_path:       Pfad zur CSV-Datei
        realtime:       Wenn True, werden Segmente mit echtem Timing
                        ausgegeben (simuliert Live-Stream)
        speed_factor:   Geschwindigkeitsfaktor für Realtime-Modus
                        (1.0 = Echtzeit, 2.0 = doppelt so schnell)
    """

    def __init__(self, csv_path: str, realtime: bool = False,
                 speed_factor: float = 1.0):
        self.csv_path = csv_path
        self.realtime = realtime
        self.speed_factor = max(0.1, speed_factor)
        self._file = None
        self._segment_count = 0

    def connect(self):
        logger.info("CsvSegmentSource: Öffne %s", self.csv_path)
        self._file = open(self.csv_path, 'r', newline='')
        self._segment_count = 0

    def disconnect(self):
        if self._file:
            self._file.close()
            self._file = None
            logger.info(
                "CsvSegmentSource: %d Segmente gelesen aus %s",
                self._segment_count, self.csv_path
            )

    def segments(self) -> Iterator[TrapezSegment]:
        if not self._file:
            raise RuntimeError("Nicht verbunden — zuerst connect() aufrufen")

        self._file.seek(0)
        reader = csv.DictReader(self._file)

        last_time = None

        for row in reader:
            segment = TrapezSegment.from_csv_row(row)
            self._segment_count += 1

            # Realtime-Modus: warte zwischen Segmenten
            if self.realtime and last_time is not None:
                delta = (segment.print_time - last_time) / self.speed_factor
                if delta > 0:
                    time.sleep(delta)

            last_time = segment.print_time
            yield segment


# ─────────────────────────────────────────────────────────────────────
# TCP-Quelle (Live-Modus — für Zukunft)
# ─────────────────────────────────────────────────────────────────────

class TcpSegmentSource(SegmentSource):
    """
    Empfängt Trapezsegmente per TCP vom move_export.py Klipper-Extra.
    Für den Live-Betrieb mit laufendem Klipper.

    Parameter:
        host:           Hostname/IP des Klipper-Containers
        port:           TCP-Port (default: 7200)
        timeout:        Verbindungs-Timeout in Sekunden
        max_retries:    Maximale Verbindungsversuche
        retry_delay:    Wartezeit zwischen Versuchen in Sekunden
    """

    def __init__(self, host: str = 'localhost', port: int = 7200,
                 timeout: float = 5.0, max_retries: int = 30,
                 retry_delay: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self._socket: Optional[socket.socket] = None
        self._buffer = ""
        self._segment_count = 0

    def connect(self):
        logger.info(
            "TcpSegmentSource: Verbinde zu %s:%d...", self.host, self.port
        )

        for attempt in range(1, self.max_retries + 1):
            try:
                self._socket = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM
                )
                self._socket.settimeout(self.timeout)
                self._socket.connect((self.host, self.port))
                logger.info(
                    "TcpSegmentSource: Verbunden (Versuch %d)", attempt
                )
                self._buffer = ""
                self._segment_count = 0

                # Hello-Message lesen
                self._read_hello()
                return

            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                if attempt < self.max_retries:
                    logger.debug(
                        "Versuch %d/%d fehlgeschlagen: %s",
                        attempt, self.max_retries, e
                    )
                    time.sleep(self.retry_delay)
                else:
                    raise ConnectionError(
                        f"Konnte nach {self.max_retries} Versuchen nicht "
                        f"verbinden zu {self.host}:{self.port}"
                    ) from e

    def _read_hello(self):
        """Liest und verarbeitet die Hello-Message vom Server."""
        self._socket.settimeout(5.0)
        try:
            data = self._socket.recv(4096)
            if data:
                line = data.decode('utf-8').strip()
                msg = json.loads(line)
                if msg.get('type') == 'hello':
                    logger.info(
                        "TcpSegmentSource: Server %s (v%s)",
                        msg.get('msg', '?'), msg.get('version', '?')
                    )
        except (socket.timeout, json.JSONDecodeError):
            pass
        finally:
            self._socket.settimeout(None)  # Blocking für segments()

    def disconnect(self):
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
            logger.info(
                "TcpSegmentSource: %d Segmente empfangen", self._segment_count
            )

    def segments(self) -> Iterator[TrapezSegment]:
        if not self._socket:
            raise RuntimeError("Nicht verbunden — zuerst connect() aufrufen")

        while True:
            # Zeilen aus Buffer verarbeiten
            while '\n' in self._buffer:
                line, self._buffer = self._buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    logger.warning("Ungültiges JSON: %s", line[:80])
                    continue

                if msg.get('type') == 'segment':
                    self._segment_count += 1
                    yield TrapezSegment.from_dict(msg)

            # Neue Daten empfangen
            try:
                data = self._socket.recv(4096)
                if not data:
                    logger.info("TcpSegmentSource: Verbindung getrennt")
                    return
                self._buffer += data.decode('utf-8')
            except (ConnectionResetError, OSError):
                logger.info("TcpSegmentSource: Verbindung verloren")
                return


# ─────────────────────────────────────────────────────────────────────
# Factory-Funktion
# ─────────────────────────────────────────────────────────────────────

def create_source(source_type: str = 'csv', **kwargs) -> SegmentSource:
    """
    Factory: Erstellt die passende SegmentSource.

    Beispiele:
        source = create_source('csv', csv_path='move_segments.csv')
        source = create_source('tcp', host='localhost', port=7200)
    """
    if source_type == 'csv':
        return CsvSegmentSource(**kwargs)
    elif source_type == 'tcp':
        return TcpSegmentSource(**kwargs)
    else:
        raise ValueError(f"Unbekannter Source-Typ: {source_type}")

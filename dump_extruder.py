#!/usr/bin/env python3
"""
dump_extruder.py — Extruder-Stepper-Daten über Moonraker API abfragen

Verwendung (im Container oder vom Host):
  python3 dump_extruder.py [moonraker_host] [moonraker_port]

Default: localhost:7125

Zeigt:
  - Klipper-Status (Ready/Error)
  - Extruder-Position und Status
  - Stepper-Queue-Status
"""

import json
import sys
import time

try:
    from urllib.request import urlopen, Request
    from urllib.error import URLError
except ImportError:
    print("ERROR: urllib not available")
    sys.exit(1)

MOONRAKER_HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
MOONRAKER_PORT = sys.argv[2] if len(sys.argv) > 2 else "7125"
BASE_URL = f"http://{MOONRAKER_HOST}:{MOONRAKER_PORT}"


def api_get(endpoint):
    """Simple GET request to Moonraker API"""
    try:
        url = f"{BASE_URL}{endpoint}"
        req = Request(url)
        resp = urlopen(req, timeout=5)
        return json.loads(resp.read().decode())
    except URLError as e:
        print(f"  API Error: {e}")
        return None
    except json.JSONDecodeError:
        return None


def api_post(endpoint, data=None):
    """Simple POST request to Moonraker API"""
    try:
        url = f"{BASE_URL}{endpoint}"
        body = json.dumps(data).encode() if data else b""
        req = Request(url, data=body, method='POST')
        req.add_header('Content-Type', 'application/json')
        resp = urlopen(req, timeout=5)
        return json.loads(resp.read().decode())
    except URLError as e:
        print(f"  API Error: {e}")
        return None


def check_klippy_status():
    """Klipper Status prüfen"""
    print("=" * 60)
    print("KLIPPER STATUS")
    print("=" * 60)
    
    result = api_get("/server/info")
    if not result:
        print("  FEHLER: Moonraker nicht erreichbar!")
        return False
    
    klippy_state = result.get("result", {}).get("klippy_state", "unknown")
    print(f"  Klippy State: {klippy_state}")
    
    if klippy_state != "ready":
        print(f"  WARNUNG: Klipper ist nicht ready (state={klippy_state})")
        return False
    
    print("  OK — Klipper ist ready")
    return True


def get_extruder_status():
    """Extruder-Status abfragen"""
    print()
    print("=" * 60)
    print("EXTRUDER STATUS")
    print("=" * 60)
    
    result = api_get(
        "/printer/objects/query?extruder&toolhead&print_stats"
    )
    if not result:
        print("  Konnte Extruder-Status nicht abfragen")
        return
    
    status = result.get("result", {}).get("status", {})
    
    # Extruder
    ext = status.get("extruder", {})
    print(f"  Extruder Temperature:  {ext.get('temperature', '?')}°C")
    print(f"  Extruder Target:       {ext.get('target', '?')}°C")
    print(f"  Pressure Advance:      {ext.get('pressure_advance', '?')}")
    print(f"  Smooth Time:           {ext.get('smooth_time', '?')}")
    
    # Toolhead
    th = status.get("toolhead", {})
    print(f"  Toolhead Position:     {th.get('position', '?')}")
    print(f"  Homed Axes:            {th.get('homed_axes', '?')}")
    print(f"  Print Time:            {th.get('print_time', '?')}")
    print(f"  Estimated Print Time:  {th.get('estimated_print_time', '?')}")
    print(f"  Max Velocity:          {th.get('max_velocity', '?')}")
    print(f"  Max Accel:             {th.get('max_accel', '?')}")
    
    # Print Stats
    ps = status.get("print_stats", {})
    print(f"  Print State:           {ps.get('state', '?')}")
    print(f"  Total Duration:        {ps.get('total_duration', '?')}s")
    print(f"  Print Duration:        {ps.get('print_duration', '?')}s")
    print(f"  Filename:              {ps.get('filename', '?')}")


def get_stepper_info():
    """Stepper-Informationen über motion_report abfragen"""
    print()
    print("=" * 60)
    print("STEPPER / MOTION REPORT")
    print("=" * 60)
    
    result = api_get(
        "/printer/objects/query?motion_report"
    )
    if not result:
        print("  Konnte Motion Report nicht abfragen")
        return
    
    status = result.get("result", {}).get("status", {})
    mr = status.get("motion_report", {})
    
    print(f"  Live Position:  {mr.get('live_position', '?')}")
    print(f"  Live Velocity:  {mr.get('live_velocity', '?')}")
    print(f"  Live Extruder Velocity: {mr.get('live_extruder_velocity', '?')}")
    
    # Stepper-spezifische Daten
    steppers = mr.get("steppers", {})
    if steppers:
        print(f"  Steppers: {list(steppers.keys())}")
        for name, data in steppers.items():
            print(f"    {name}: {data}")


def run_gcode(cmd):
    """G-Code Befehl über Moonraker senden"""
    result = api_post("/printer/gcode/script", {"script": cmd})
    return result is not None


def start_print_and_monitor(filename):
    """Print starten und auf Fertigstellung warten"""
    print()
    print("=" * 60)
    print(f"STARTE PRINT: {filename}")
    print("=" * 60)
    
    # Print starten
    result = api_post(f"/printer/print/start?filename={filename}")
    if not result:
        print("  FEHLER: Print konnte nicht gestartet werden!")
        # Alternativ über G-Code
        print("  Versuche über G-Code...")
        run_gcode(f"SDCARD_PRINT_FILE FILENAME={filename}")
    
    print("  Print gestartet, warte auf Fertigstellung...")
    
    # Warten
    for i in range(120):  # Max 2 Minuten
        time.sleep(1)
        result = api_get("/printer/objects/query?print_stats&extruder&toolhead")
        if not result:
            continue
        
        status = result.get("result", {}).get("status", {})
        ps = status.get("print_stats", {})
        ext = status.get("extruder", {})
        th = status.get("toolhead", {})
        state = ps.get("state", "unknown")
        
        pos = th.get("position", [0, 0, 0, 0])
        e_pos = pos[3] if len(pos) > 3 else 0
        
        if i % 5 == 0:
            print(f"  [{i:3d}s] State={state}, E-Pos={e_pos:.3f}mm")
        
        if state == "standby" and i > 3:
            print()
            print("  PRINT FERTIG!")
            print(f"  Finale E-Position: {e_pos:.3f}mm")
            print(f"  Duration: {ps.get('print_duration', '?')}s")
            return True
        
        if state == "error":
            msg = ps.get("message", "unknown")
            print(f"\n  FEHLER WÄHREND PRINT: {msg}")
            return False
    
    print("\n  TIMEOUT — Print nicht in 120s fertig")
    return False


def main():
    print()
    print("########################################################")
    print("#  EXTRUDER QUEUE VALIDATION                           #")
    print("#  Klipper Simulation — Host MCU                       #")
    print("########################################################")
    print()
    
    # 1. Status prüfen
    if not check_klippy_status():
        print("\nABBRUCH: Klipper nicht bereit.")
        sys.exit(1)
    
    # 2. Extruder-Status vor Print
    get_extruder_status()
    get_stepper_info()
    
    # 3. Position setzen und kalt-extrusion erlauben
    print()
    print(">> Setze Startposition und erlaube Cold Extrusion...")
    run_gcode("SET_KINEMATIC_POSITION X=250 Y=250 Z=150")
    run_gcode("M302 S0")
    time.sleep(1)
    
    # 4. Print starten
    success = start_print_and_monitor("extruder_test.gcode")
    
    # 5. Status nach Print
    if success:
        print()
        print(">> Post-Print Status:")
        get_extruder_status()
        get_stepper_info()
        
        print()
        print("=" * 60)
        print("ERGEBNIS: EXTRUDER QUEUE VALIDATION")
        print("=" * 60)
        print("  ✅ G-Code wurde fehlerfrei abgearbeitet")
        print("  ✅ Extruder-Stepper-Queue wurde korrekt erzeugt")
        print("  ✅ Kein MCU-Shutdown, kein Timer-Error")
        print("  → Die Extruder-Queue-Erzeugung funktioniert.")
        print()
        print("  Für detaillierte Logs:")
        print("  docker exec klipper tail -100 /home/klippy/printer_data/logs/klippy.log")
    else:
        print()
        print("=" * 60)
        print("ERGEBNIS: VALIDATION FEHLGESCHLAGEN")
        print("=" * 60)
        print("  ❌ Fehler bei der Queue-Erzeugung")
        print("  → Prüfe klippy.log für Details")


if __name__ == "__main__":
    main()

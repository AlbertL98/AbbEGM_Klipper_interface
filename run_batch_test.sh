#!/bin/bash
# =============================================================================
# run_batch_test.sh — Klipper Batch-Mode Test Runner (Vollautomatisch)
#
# Führt aus:
#   1. Dictionary prüfen/kompilieren
#   2. move_export.py Extra installieren
#   3. Batch-Run (G-Code → MCU-Kommandos + Trapez-CSV)
#   4. MCU-Kommandos decodieren
#   5. Step-CSV extrahieren
#   6. Alles ins gemountete logs-Verzeichnis kopieren
#
# Verwendung:
#   docker exec klipper bash /home/klippy/run_batch_test.sh [gcode-datei]
#
# Outputs (in ./logs/batch_results/ auf Windows):
#   - mcu_commands.txt        Alle MCU-Kommandos (Klartext)
#   - extruder_steps.csv      Extruder queue_step Daten
#   - move_segments.csv       Trapez-Segmente (CodingKernPlan Punkt 1!)
#   - batch_test.log          Klippy Log
#   - test_summary.txt        Zusammenfassung
# =============================================================================
set -e

# --- Konfiguration ---
KLIPPER_DIR="/home/klippy/klipper"
KLIPPY_ENV="/home/klippy/klippy-env/bin/python"
DICT_FILE="$KLIPPER_DIR/out/klipper.dict"

CONFIG="/home/klippy/printer_data/config/printer_batch.cfg"
GCODE="${1:-/home/klippy/printer_data/gcodes/extruder_batch_test.gcode}"

# Output — im gemounteten Volume, direkt von Windows/PyCharm aus erreichbar
OUT_DIR="/home/klippy/printer_data/logs/batch_results"

# Farben
G='\033[0;32m'; R='\033[0;31m'; Y='\033[1;33m'; B='\033[1;34m'; N='\033[0m'

echo -e "${B}══════════════════════════════════════════════${N}"
echo -e "${B}  Klipper Batch-Mode Test Runner${N}"
echo -e "${B}══════════════════════════════════════════════${N}"
echo ""

# --- Schritt 0: Vorbereiten ---
mkdir -p "$OUT_DIR"
cd "$KLIPPER_DIR"

# --- Schritt 1: Dictionary ---
echo -e "${Y}[1/6] Dictionary prüfen...${N}"
if [ ! -f "$DICT_FILE" ]; then
    echo "  Kompiliere Linux-MCU Dictionary..."
    make clean 2>/dev/null || true
    cat > .config << 'MKCONF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_LINUX=y
CONFIG_BOARD_DIRECTORY="linux"
CONFIG_CLOCK_FREQ=50000000
CONFIG_LINUX_SELECT=y
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_GPIO_HARD_PWM=y
CONFIG_INLINE_STEPPER_HACK=y
MKCONF
    make olddefconfig 2>&1 | tail -3
    make -j$(nproc) 2>&1 | tail -5
fi

if [ ! -f "$DICT_FILE" ]; then
    echo -e "${R}  FEHLER: Dictionary nicht gefunden!${N}"
    exit 1
fi
echo -e "${G}  OK: $(ls -lh "$DICT_FILE" | awk '{print $5}')${N}"

# --- Schritt 2: move_export.py installieren ---
echo ""
echo -e "${Y}[2/6] move_export.py Extra installieren...${N}"
EXTRAS_DIR="$KLIPPER_DIR/klippy/extras"
CUSTOM_DIR="/home/klippy/custom_extras"

if [ -f "$CUSTOM_DIR/move_export.py" ]; then
    ln -sf "$CUSTOM_DIR/move_export.py" "$EXTRAS_DIR/move_export.py" 2>/dev/null || true
    echo -e "${G}  OK: Verlinkt aus custom_extras${N}"
elif [ -f "$EXTRAS_DIR/move_export.py" ]; then
    echo -e "${G}  OK: Bereits vorhanden${N}"
else
    echo -e "${R}  FEHLER: move_export.py nicht gefunden!${N}"
    echo "  Bitte move_export.py nach $CUSTOM_DIR/ oder $EXTRAS_DIR/ kopieren."
    exit 1
fi

# --- Schritt 3: Prüfe Inputs ---
echo ""
echo -e "${Y}[3/6] Inputs prüfen...${N}"
if [ ! -f "$CONFIG" ]; then
    echo -e "${R}  FEHLER: Config nicht gefunden: $CONFIG${N}"
    exit 1
fi
echo "  Config: $CONFIG"

if [ ! -f "$GCODE" ]; then
    echo -e "${R}  FEHLER: G-Code nicht gefunden: $GCODE${N}"
    echo "  Verwendung: $0 /pfad/zur/datei.gcode"
    exit 1
fi
echo "  G-Code: $GCODE"
echo "  Output: $OUT_DIR/"

# --- Schritt 4: Batch-Run ---
echo ""
echo -e "${Y}[4/6] Batch-Run starten...${N}"

SEGMENTS_CSV="/home/klippy/printer_data/logs/move_segments.csv"
rm -f "$SEGMENTS_CSV" 2>/dev/null || true

$KLIPPY_ENV "$KLIPPER_DIR/klippy/klippy.py" \
    "$CONFIG" \
    -i "$GCODE" \
    -o "$OUT_DIR/test.serial" \
    -v \
    -d "$DICT_FILE" \
    -l "$OUT_DIR/batch_test.log" \
    2>&1 | tee "$OUT_DIR/klippy_stdout.txt"

EXIT_CODE=${PIPESTATUS[0]}

if [ $EXIT_CODE -ne 0 ]; then
    echo -e "${R}  FEHLER: Batch-Run fehlgeschlagen (exit $EXIT_CODE)${N}"
    echo "  Siehe: $OUT_DIR/batch_test.log"
    tail -30 "$OUT_DIR/batch_test.log" 2>/dev/null
    exit 1
fi

PRINT_TIME=$(grep -oP 'print time \K[0-9.]+s' "$OUT_DIR/batch_test.log" 2>/dev/null || echo "?")
echo -e "${G}  OK: Batch-Run erfolgreich (print time: $PRINT_TIME)${N}"

# --- Schritt 5: MCU-Kommandos decodieren ---
echo ""
echo -e "${Y}[5/6] MCU-Kommandos decodieren...${N}"

$KLIPPY_ENV "$KLIPPER_DIR/klippy/parsedump.py" \
    "$DICT_FILE" \
    "$OUT_DIR/test.serial" \
    > "$OUT_DIR/mcu_commands.txt" 2>/dev/null

TOTAL_CMDS=$(wc -l < "$OUT_DIR/mcu_commands.txt")
echo -e "${G}  OK: $TOTAL_CMDS MCU-Kommandos${N}"

# --- Schritt 6: CSVs extrahieren ---
echo ""
echo -e "${Y}[6/6] CSVs extrahieren...${N}"

# Extruder Steps (oid=9)
echo "oid,interval,count,add" > "$OUT_DIR/extruder_steps.csv"
grep "queue_step oid=9" "$OUT_DIR/mcu_commands.txt" | \
    sed 's/.*oid=\([^ ]*\) interval=\([^ ]*\) count=\([^ ]*\) add=\([^ ]*\)/\1,\2,\3,\4/' \
    >> "$OUT_DIR/extruder_steps.csv" 2>/dev/null || true
ESTEPS=$(($(wc -l < "$OUT_DIR/extruder_steps.csv") - 1))
echo "  Extruder Steps:   $ESTEPS"

# Alle Steps
echo "oid,interval,count,add" > "$OUT_DIR/all_steps.csv"
grep "queue_step" "$OUT_DIR/mcu_commands.txt" | \
    sed 's/.*oid=\([^ ]*\) interval=\([^ ]*\) count=\([^ ]*\) add=\([^ ]*\)/\1,\2,\3,\4/' \
    >> "$OUT_DIR/all_steps.csv" 2>/dev/null || true
ALLSTEPS=$(($(wc -l < "$OUT_DIR/all_steps.csv") - 1))
echo "  Alle Steps:       $ALLSTEPS"

# Richtungswechsel
echo "oid,dir" > "$OUT_DIR/dir_changes.csv"
grep "set_next_step_dir" "$OUT_DIR/mcu_commands.txt" | \
    sed 's/.*oid=\([^ ]*\) dir=\([^ ]*\)/\1,\2/' \
    >> "$OUT_DIR/dir_changes.csv" 2>/dev/null || true
DIRS=$(($(wc -l < "$OUT_DIR/dir_changes.csv") - 1))
echo "  Richtungswechsel: $DIRS"

# move_segments.csv (vom move_export Extra)
SEGS="n/a"
if [ -f "$SEGMENTS_CSV" ]; then
    cp "$SEGMENTS_CSV" "$OUT_DIR/move_segments.csv"
    SEGS=$(($(wc -l < "$OUT_DIR/move_segments.csv") - 1))
    echo -e "${G}  Trapez-Segmente:  $SEGS (CodingKernPlan!)${N}"
else
    echo -e "${Y}  Trapez-Segmente:  nicht erzeugt (move_export nicht aktiv?)${N}"
fi

# --- Zusammenfassung ---
echo ""
echo -e "${B}══════════════════════════════════════════════${N}"
echo -e "${B}  Ergebnis${N}"
echo -e "${B}══════════════════════════════════════════════${N}"

cat > "$OUT_DIR/test_summary.txt" << SUMMARY
Klipper Batch Test — $(date)
================================================
G-Code:            $(basename "$GCODE")
Print Time:        $PRINT_TIME
MCU-Kommandos:     $TOTAL_CMDS
Extruder Steps:    $ESTEPS
Alle Steps:        $ALLSTEPS
Richtungswechsel:  $DIRS
Trapez-Segmente:   $SEGS

Dateien:
  mcu_commands.txt     — MCU-Kommandos (Klartext)
  extruder_steps.csv   — Extruder queue_step Daten
  all_steps.csv        — Alle Stepper queue_step Daten
  dir_changes.csv      — Richtungswechsel
  move_segments.csv    — Trapez-Segmente (Positionen, Geschwindigkeiten, Zeiten)
  batch_test.log       — Klippy Log (verbose)
SUMMARY

cat "$OUT_DIR/test_summary.txt"

echo ""
echo -e "${G}Windows-Pfad: ./logs/batch_results/${N}"
echo -e "${G}Öffne in PyCharm: logs/batch_results/move_segments.csv${N}"
echo -e "${B}══════════════════════════════════════════════${N}"

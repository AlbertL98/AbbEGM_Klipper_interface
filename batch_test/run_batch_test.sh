#!/bin/bash
# =============================================================================
# run_batch_test.sh — Klipper Batch-Mode Test Runner
# Ausführen im Docker-Container: bash /home/klippy/run_batch_test.sh
# =============================================================================
set -e

KLIPPER_DIR="/home/klippy/klipper"
KLIPPY_ENV="/home/klippy/klippy-env/bin/python"
WORK_DIR="/home/klippy/batch_test_output"

CONFIG="/home/klippy/printer_batch.cfg"
GCODE="/home/klippy/extruder_batch_test.gcode"

# Output-Dateien
SERIAL_OUT="$WORK_DIR/test.serial"
LOG_OUT="$WORK_DIR/batch_test.log"
READABLE_OUT="$WORK_DIR/mcu_commands.txt"
STEPS_CSV="$WORK_DIR/extruder_steps.csv"

# Farben
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=============================================="
echo " Klipper Batch-Mode Test"
echo "=============================================="

# --- Schritt 0: Vorbereitung ---
mkdir -p "$WORK_DIR"
cd "$KLIPPER_DIR"

# --- Schritt 1: Dictionary prüfen / generieren ---
DICT_FILE="$KLIPPER_DIR/out/klipper.dict"

if [ ! -f "$DICT_FILE" ]; then
    echo -e "${YELLOW}>> Dictionary nicht gefunden. Versuche zu kompilieren...${NC}"
    
    # Linux-MCU-Config setzen
    make clean 2>/dev/null || true
    cat > .config << 'MAKECONFIG'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_LINUX=y
CONFIG_BOARD_DIRECTORY="linux"
CONFIG_CLOCK_FREQ=50000000
CONFIG_LINUX_SELECT=y
CONFIG_USB_VENDOR_ID=0x1d50
CONFIG_USB_DEVICE_ID=0x614e
CONFIG_USB_SERIAL_NUMBER="klipper"
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_GPIO_HARD_PWM=y
CONFIG_INLINE_STEPPER_HACK=y
MAKECONFIG
    
    make olddefconfig 2>&1 | tail -5
    make -j$(nproc) 2>&1 | tail -10
    
    if [ ! -f "$DICT_FILE" ]; then
        echo -e "${RED}>> FEHLER: Dictionary konnte nicht generiert werden!${NC}"
        echo "   Prüfe ob build-essential, gcc, etc. installiert sind."
        echo ""
        echo "   Alternative: Dictionary manuell herunterladen von"
        echo "   https://github.com/Klipper3d/klipper/releases"
        exit 1
    fi
    echo -e "${GREEN}>> Dictionary generiert: $DICT_FILE${NC}"
else
    echo -e "${GREEN}>> Dictionary gefunden: $DICT_FILE${NC}"
fi

# --- Schritt 1b: Dictionary inspizieren ---
echo ""
echo ">> Dictionary-Info:"
python3 -c "
import json
with open('$DICT_FILE') as f:
    d = json.load(f)
print(f'   Version: {d.get(\"version\", \"?\")}'  )
print(f'   Build:   {d.get(\"build_versions\", \"?\")}'  )
cmds = d.get('commands', {})
print(f'   Commands: {len(cmds)}')
# Prüfe ob queue_step vorhanden
has_queue_step = any('queue_step' in str(v) for v in cmds.values())
print(f'   queue_step: {\"JA\" if has_queue_step else \"NEIN\"}'  )
" 2>/dev/null || echo "   (Dictionary-Inspektion übersprungen)"

# --- Schritt 2: Batch-Run ---
echo ""
echo ">> Starte Batch-Run..."
echo "   Config: $CONFIG"
echo "   G-Code: $GCODE"
echo ""

$KLIPPY_ENV "$KLIPPER_DIR/klippy/klippy.py" \
    "$CONFIG" \
    -i "$GCODE" \
    -o "$SERIAL_OUT" \
    -v \
    -d "$DICT_FILE" \
    -l "$LOG_OUT" \
    2>&1 | tee "$WORK_DIR/klippy_stdout.txt"

EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo -e "${RED}>> FEHLER: Batch-Run fehlgeschlagen (exit $EXIT_CODE)${NC}"
    echo "   Log: $LOG_OUT"
    echo ""
    echo ">> Letzte 30 Zeilen des Logs:"
    tail -30 "$LOG_OUT" 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}>> Batch-Run erfolgreich!${NC}"

# --- Schritt 3: Serial → Klartext ---
echo ""
echo ">> Übersetze MCU-Kommandos..."

if [ -f "$SERIAL_OUT" ]; then
    $KLIPPY_ENV "$KLIPPER_DIR/klippy/parsedump.py" \
        "$DICT_FILE" \
        "$SERIAL_OUT" \
        > "$READABLE_OUT" 2>/dev/null
    
    TOTAL_CMDS=$(wc -l < "$READABLE_OUT")
    QUEUE_STEPS=$(grep -c "queue_step" "$READABLE_OUT" 2>/dev/null || echo "0")
    DIR_CHANGES=$(grep -c "set_next_step_dir" "$READABLE_OUT" 2>/dev/null || echo "0")
    
    echo -e "${GREEN}>> MCU-Kommandos: $TOTAL_CMDS gesamt${NC}"
    echo "   queue_step:          $QUEUE_STEPS"
    echo "   set_next_step_dir:   $DIR_CHANGES  (Retraction-Beweis)"
else
    echo -e "${YELLOW}>> Keine Serial-Datei erzeugt (evtl. leere Queue)${NC}"
fi

# --- Schritt 4: Extrahiere queue_step als CSV ---
echo ""
echo ">> Extrahiere Step-Daten..."

if [ -f "$READABLE_OUT" ]; then
    echo "# timestamp,oid,interval,count,add" > "$STEPS_CSV"
    grep "queue_step" "$READABLE_OUT" | \
        sed 's/.*clock=\([^ ]*\).*oid=\([^ ]*\).*interval=\([^ ]*\).*count=\([^ ]*\).*add=\([^ ]*\).*/\1,\2,\3,\4,\5/' \
        >> "$STEPS_CSV" 2>/dev/null || true
    
    CSV_LINES=$(($(wc -l < "$STEPS_CSV") - 1))
    echo "   $CSV_LINES Step-Einträge in $STEPS_CSV"
fi

# --- Schritt 5: Log-Analyse ---
echo ""
echo ">> Log-Analyse:"
if [ -f "$LOG_OUT" ]; then
    ERRORS=$(grep -ci "error" "$LOG_OUT" 2>/dev/null || echo "0")
    WARNINGS=$(grep -ci "warning" "$LOG_OUT" 2>/dev/null || echo "0")
    echo "   Errors:   $ERRORS"
    echo "   Warnings: $WARNINGS"
    
    # Move-Timing aus Log extrahieren (wenn verbose)
    MOVES=$(grep -c "move_time" "$LOG_OUT" 2>/dev/null || echo "0")
    echo "   Move entries: $MOVES"
fi

# --- Zusammenfassung ---
echo ""
echo "=============================================="
echo " Ergebnis"
echo "=============================================="
echo " Output-Verzeichnis: $WORK_DIR/"
echo ""
echo " Dateien:"
ls -lh "$WORK_DIR/" 2>/dev/null | grep -v "^total"
echo ""

if [ "$QUEUE_STEPS" -gt 0 ] 2>/dev/null; then
    echo -e "${GREEN} ✓ BEWEIS: Klipper hat $QUEUE_STEPS Step-Kommandos generiert!${NC}"
    echo -e "${GREEN}   Die Move-Queue wurde aufgebaut und verarbeitet.${NC}"
else
    echo -e "${YELLOW} ⚠ Keine queue_step Kommandos gefunden.${NC}"
    echo "   Prüfe Log und MCU-Kommandos manuell."
fi

echo ""
echo ">> Nächste Schritte:"
echo "   1) cat $READABLE_OUT | head -50    # MCU-Kommandos ansehen"
echo "   2) cat $STEPS_CSV                  # Step-CSV ansehen"
echo "   3) cat $LOG_OUT | grep -i move     # Move-Timing im Log"
echo "=============================================="

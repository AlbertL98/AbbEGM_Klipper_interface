# Klipper Batch-Modus: Schritt-für-Schritt Anleitung

## Übersicht

Der Batch-Modus (`klippy.py -i gcode -o serial -d dict`) ist Klippers offizieller Weg, G-Code **ohne Hardware** durch den kompletten Motion-Planner zu schicken. Dabei wird:

1. G-Code geparsed
2. Lookahead + Junction Deviation berechnet
3. Trapezprofile (accel/cruise/decel) erstellt
4. MCU-Kommandos generiert (binär in `.serial`-Datei)
5. Diese Kommandos können dann mit `parsedump.py` in lesbaren Text übersetzt werden

**Voraussetzung:** Ein "Data Dictionary" (`klipper.dict`) — das ist ein JSON-File, das die MCU-Kommandos beschreibt. Du brauchst das passende Dictionary für deine MCU-Architektur.

---

## Schritt 1: Dictionary generieren (im Docker-Container)

Da du bereits `klipper_mcu` (Linux Host MCU) in deinem Container hast, kannst du das Dictionary direkt aus dem kompilierten Binary extrahieren. Der Linux-Host-MCU hat ein eingebautes Dictionary.

```bash
# Im Container:
cd /home/klippy/klipper

# Option A: Dictionary aus der laufenden klipper_mcu extrahieren
# Die klipper_mcu gibt ihr Dictionary aus, wenn man es richtig abfragt.
# Am einfachsten: Klipper für "linux" kompilieren und Dictionary nehmen.

# Konfiguration für Linux-Host-MCU setzen:
make clean
cp /dev/null .config
cat > .config << 'EOF'
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
EOF

# Kompilieren (generiert out/klipper.dict)
make olddefconfig
make
```

Wenn `make` durchläuft, liegt das Dictionary unter `out/klipper.dict`.

**Alternative**: Falls `make` im Container nicht klappt (fehlende Toolchain), kannst du das Dictionary auch aus einem Klipper GitHub Release herunterladen — dort gibt es Archive `klipper-dict-*.tar.gz` mit vorcompilierten Dictionaries für verschiedene Plattformen.

---

## Schritt 2: printer.cfg für Batch-Modus anpassen

Deine aktuelle `printer.cfg` hat ein Problem für den Batch-Modus: Sie referenziert deinen Custom-Extracode `[egm_bridge]` und `[dummy_thermistor]`. Im Batch-Modus werden Extras nur geladen, wenn sie im Python-Pfad sind.

Erstelle eine **separate** `printer_batch.cfg` — minimal und sauber:

```ini
# printer_batch.cfg — NUR für Batch-Test, keine Hardware nötig
[mcu]
serial: /tmp/klipper_host_mcu

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
max_z_velocity: 50
max_z_accel: 500

[stepper_x]
step_pin: PA0
dir_pin: PA1
enable_pin: !PA2
microsteps: 16
rotation_distance: 40
endstop_pin: PA3
position_endstop: 0
position_max: 500
homing_speed: 50

[stepper_y]
step_pin: PA4
dir_pin: PA5
enable_pin: !PA6
microsteps: 16
rotation_distance: 40
endstop_pin: PA7
position_endstop: 0
position_max: 500
homing_speed: 50

[stepper_z]
step_pin: PB0
dir_pin: PB1
enable_pin: !PB2
microsteps: 16
rotation_distance: 8
endstop_pin: PB3
position_endstop: 0
position_max: 300
homing_speed: 25

[extruder]
step_pin: PB4
dir_pin: PB5
enable_pin: !PB6
microsteps: 16
rotation_distance: 4.637
nozzle_diameter: 0.400
filament_diameter: 1.750
max_extrude_only_velocity: 60
max_extrude_only_accel: 800
max_extrude_cross_section: 50
heater_pin: PB7
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PB8
control: watermark
min_temp: 0
max_temp: 300
min_extrude_temp: 0
pressure_advance: 0.0
pressure_advance_smooth_time: 0.040

[virtual_sdcard]
path: /home/klippy/printer_data/gcodes

[force_move]
enable_force_move: True
```

**Wichtige Änderungen gegenüber deiner printer.cfg:**
- `[dummy_thermistor]` entfernt (das ist dein Custom-Extra, nicht im Standard-Klipper)
- `[egm_bridge]` entfernt (ebenfalls Custom)
- `sensor_type` auf einen Standard-Typ gesetzt (`EPCOS 100K B57560G104F`)
- `sensor_pin` hinzugefügt (Batch braucht eine gültige Config-Syntax)
- `min_extrude_temp: 0` direkt gesetzt (damit M302 nicht nötig ist)
- `[display_status]` und `[pause_resume]` entfernt (nicht nötig für Batch)

---

## Schritt 3: G-Code anpassen

Dein `extruder_test.gcode` nutzt `M302 S0` — das brauchen wir nicht mehr, weil `min_extrude_temp: 0` in der Config steht. Auch `SET_HEATER_TEMPERATURE` ist überflüssig im Batch-Modus.

Vereinfachter G-Code (`extruder_batch_test.gcode`):

```gcode
; Batch-Mode Extruder Test
SET_KINEMATIC_POSITION X=250 Y=250 Z=150
M83
G1 E1.0 F60
G1 E2.0 F120
G1 E-0.8 F180
G1 E5.0 F600
G4 P500
M400
```

---

## Schritt 4: Batch-Run ausführen

```bash
cd /home/klippy/klipper

# Batch-Run: G-Code → binäre MCU-Kommandos
/home/klippy/klippy-env/bin/python ./klippy/klippy.py \
    /home/klippy/printer_batch.cfg \
    -i /home/klippy/extruder_batch_test.gcode \
    -o /home/klippy/test_output.serial \
    -v \
    -d out/klipper.dict \
    -l /home/klippy/batch_test.log
```

**Flags:**
- `-i` = Input G-Code
- `-o` = Output binäre Serial-Datei
- `-v` = Verbose (mehr Log-Output)
- `-d` = Data Dictionary
- `-l` = Log-Datei

**Erwartetes Ergebnis:** Exit Code 0, keine ERROR-Zeilen im Log.

---

## Schritt 5: Serial-Output in lesbaren Text übersetzen

```bash
/home/klippy/klippy-env/bin/python ./klippy/parsedump.py \
    out/klipper.dict \
    /home/klippy/test_output.serial \
    > /home/klippy/test_output_readable.txt
```

Die Datei `test_output_readable.txt` enthält dann die MCU-Kommandos im Klartext. Du wirst dort z.B. sehen:

- `queue_step` Kommandos für den Extruder-Stepper (Step-Intervals + Counts)
- `set_next_step_dir` für Richtungswechsel (Retraction)
- `stepper_get_position` für Positionsabfragen

**Das ist dein Beweis**, dass Klipper die E-Moves vollständig durch den Planner geschickt und MCU-Kommandos generiert hat.

---

## Schritt 6: Daten analysieren / CSV exportieren

Die `parsedump`-Ausgabe ist bereits recht lesbar. Für eine CSV-Analyse kannst du die `queue_step`-Kommandos extrahieren:

```bash
# Alle queue_step Kommandos für Extruder extrahieren
grep "queue_step" /home/klippy/test_output_readable.txt \
    > /home/klippy/extruder_steps.csv
```

Für tiefere Analyse (Trapezprofile, Timing) ist die **Log-Datei** (`batch_test.log`) informativer — dort stehen die Move-Timings mit `-v` Flag.

---

## Zusammenfassung: Was beweist der Batch-Test?

| Was                        | Beweis?                                        |
|----------------------------|------------------------------------------------|
| G-Code Parsing             | ✅ Kein Error → alle Befehle erkannt           |
| Move-Queue aufgebaut       | ✅ Serial-Output enthält queue_step Kommandos  |
| Trapezprofil berechnet     | ✅ Log zeigt Timing (print_time, move_time)    |
| Extruder E-Steps generiert | ✅ queue_step für Extruder-OID sichtbar        |
| Retraction funktioniert    | ✅ set_next_step_dir Wechsel sichtbar          |
| Pressure Advance           | ✅ PA-modifizierte Steps (wenn PA > 0)         |

---

## Bekannte Stolpersteine

1. **Dictionary-Mismatch**: Das Dictionary MUSS zur gleichen Klipper-Version passen, mit der `klippy.py` läuft. Sonst: "Unknown command" Errors.

2. **Pin-Namen**: Im Batch-Modus werden Pins nicht wirklich initialisiert, aber sie müssen syntaktisch gültig sein für das gewählte Dictionary. Linux-MCU kennt nur `gpio0`, `gpio1`, etc. — nicht `PA0`. Du brauchst also das **Linux-Dictionary** und musst ggf. die Pin-Namen in `printer_batch.cfg` anpassen (oder ein ATmega-Dictionary nehmen, das `PA0` etc. kennt).

3. **Custom Extras**: Alles was nicht Standard-Klipper ist (`[egm_bridge]`, `[dummy_thermistor]`) muss entweder raus aus der Config oder im Python-Pfad liegen.

4. **sensor_type / heater**: Im Batch-Modus wird der Heater nicht wirklich angesteuert, aber die Config muss trotzdem einen gültigen `sensor_type` haben.

---

## Nächster Schritt: Richtung CodingKernPlan

Wenn der Batch-Test läuft, hast du bewiesen, dass Klipper's Planner deine G-Codes verarbeitet. Der nächste Schritt wäre:

- **Einen `move_finalize_callback` in `toolhead.py` einbauen** (CodingKernPlan Punkt 1) — und diesen ebenfalls im Batch-Modus testen
- **Die Callback-Daten (Trapezprofile) als CSV exportieren** — das wäre schon deutlich aussagekräftiger als rohe MCU-Steps
- **Custom Kinematics** bauen (CodingKernPlan Punkt 2) — und damit das Pin/MCU-Problem dauerhaft lösen

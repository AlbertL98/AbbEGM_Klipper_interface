#!/bin/sh
set -e

chmod 755 /home/klippy

# Custom Klipper Extras verlinken
CUSTOM_EXTRAS="/home/klippy/custom_extras"
KLIPPER_EXTRAS="/home/klippy/klipper/klippy/extras"

if [ -d "$CUSTOM_EXTRAS" ]; then
    echo ">> Verlinke Custom Extras..."
    for pyfile in "$CUSTOM_EXTRAS"/*.py; do
        [ -f "$pyfile" ] || continue
        filename=$(basename "$pyfile")
        ln -sf "$pyfile" "$KLIPPER_EXTRAS/$filename"
        echo "   $filename"
    done
fi

# Default Configs
if [ ! -f /home/klippy/printer_data/config/moonraker.conf ]; then
    echo ">> Erstelle moonraker.conf..."
    cat > /home/klippy/printer_data/config/moonraker.conf << 'EOF'
[server]
host: 0.0.0.0
port: 7125
klippy_uds_address: /tmp/klippy_uds

[authorization]
trusted_clients:
    0.0.0.0/0
cors_domains:
    *

[octoprint_compat]
[history]
[file_manager]
enable_object_processing: True

[machine]
provider: none
EOF
    chown klippy:klippy /home/klippy/printer_data/config/moonraker.conf
fi

if [ ! -f /home/klippy/printer_data/config/printer.cfg ]; then
    echo ">> Erstelle printer.cfg (Simulations-Modus)..."
    cat > /home/klippy/printer_data/config/printer.cfg << 'EOF'
[mcu]
serial: /tmp/klipper_host_mcu

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
max_z_velocity: 50
max_z_accel: 500

[stepper_x]
step_pin: gpio0
dir_pin: gpio1
enable_pin: !gpio2
microsteps: 16
rotation_distance: 40
endstop_pin: gpio3
position_endstop: 0
position_max: 500
homing_speed: 50

[stepper_y]
step_pin: gpio4
dir_pin: gpio5
enable_pin: !gpio6
microsteps: 16
rotation_distance: 40
endstop_pin: gpio7
position_endstop: 0
position_max: 500
homing_speed: 50

[stepper_z]
step_pin: gpio8
dir_pin: gpio9
enable_pin: !gpio10
microsteps: 16
rotation_distance: 8
endstop_pin: gpio11
position_endstop: 0
position_max: 300
homing_speed: 25

[extruder]
step_pin: gpio12
dir_pin: gpio13
enable_pin: !gpio14
microsteps: 16
rotation_distance: 4.637
nozzle_diameter: 0.400
filament_diameter: 1.750
max_extrude_only_velocity: 60
max_extrude_only_accel: 800
max_extrude_cross_section: 50
heater_pin: gpio15
sensor_type: EPCOS 100K B57560G104F
sensor_pin: gpio16
control: watermark
min_temp: 0
max_temp: 300
min_extrude_temp: 0
pressure_advance: 0.0
pressure_advance_smooth_time: 0.040

[virtual_sdcard]
path: /home/klippy/printer_data/gcodes

[display_status]
[pause_resume]

[force_move]
enable_force_move: True

[move_export]
output_path: /home/klippy/printer_data/logs/move_segments.csv
tcp_port: 7200
tcp_enabled: true
log_to_console: true
EOF
    chown klippy:klippy /home/klippy/printer_data/config/printer.cfg
fi

nginx -s stop 2>/dev/null || true
sleep 1

# --- Simulierte MCU starten (als klippy-User, vor Supervisor) ---
echo ">> Starte simulierte Linux-MCU..."
rm -f /tmp/klipper_host_mcu
su -s /bin/bash klippy -c \
    'socat PTY,link=/tmp/klipper_host_mcu,rawer EXEC:/usr/local/bin/klipper_mcu,pty,rawer &'
sleep 2

if [ -e /tmp/klipper_host_mcu ]; then
    echo ">> MCU PTY bereit: /tmp/klipper_host_mcu"
else
    echo ">> WARNUNG: MCU PTY nicht gefunden!"
fi

echo ">> Starte Supervisor..."
exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/supervisord.conf

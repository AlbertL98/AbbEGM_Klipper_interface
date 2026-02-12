#!/bin/bash
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
    echo ">> Erstelle printer.cfg..."
    cat > /home/klippy/printer_data/config/printer.cfg << 'EOF'
[mcu]
serial: /tmp/klipper_host_mcu

[printer]
kinematics: none
max_velocity: 300
max_accel: 3000

[virtual_sdcard]
path: /home/klippy/printer_data/gcodes

[display_status]
[pause_resume]

[egm_bridge]
debug_mode: true
log_moves: false
EOF
    chown klippy:klippy /home/klippy/printer_data/config/printer.cfg
fi

nginx -s stop 2>/dev/null || true
sleep 1

echo ">> Starte Supervisor..."
exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/supervisord.conf

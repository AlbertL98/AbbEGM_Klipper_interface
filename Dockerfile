# ---- Ubuntu + Klipper + Moonraker + Mainsail ----
# Mit simulierter Linux-MCU für Docker/WSL2 (kein echtes GPIO nötig)
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# System-Updates und Basis-Pakete
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    git \
    build-essential \
    libffi-dev \
    libncurses-dev \
    avrdude \
    gcc-avr \
    binutils-avr \
    avr-libc \
    stm32flash \
    dfu-util \
    libnewlib-arm-none-eabi \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    libusb-1.0-0 \
    libcurl4-openssl-dev \
    libssl-dev \
    liblmdb-dev \
    libsodium-dev \
    zlib1g-dev \
    iproute2 \
    sudo \
    wget \
    curl \
    nano \
    supervisor \
    nginx \
    socat \
    && rm -rf /var/lib/apt/lists/*

# !! Nginx Autostart deaktivieren !!
RUN rm -f /etc/init.d/nginx \
    && update-rc.d -f nginx remove 2>/dev/null || true \
    && echo "daemon off;" >> /etc/nginx/nginx.conf

# Klipper-User erstellen
RUN useradd -ms /bin/bash klippy && \
    echo "klippy ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# --- KLIPPER ---
USER klippy
WORKDIR /home/klippy

RUN git clone https://github.com/Klipper3d/klipper.git

RUN python3 -m venv klippy-env && \
    /home/klippy/klippy-env/bin/pip install --upgrade pip && \
    /home/klippy/klippy-env/bin/pip install -r /home/klippy/klipper/scripts/klippy-requirements.txt

# --- GPIO SIMULATION PATCH ---
# Patcht die Linux-MCU Hardware-Dateien damit sie ohne echtes
# /dev/gpiochip, /sys/bus/iio etc. funktionieren (für Docker/WSL2)
COPY gpio_patch.py /tmp/gpio_patch.py

# --- LINUX-MCU KOMPILIEREN (mit GPIO-Patch) ---
WORKDIR /home/klippy/klipper

# Erst patchen, dann als Linux-MCU bauen
RUN python3 /tmp/gpio_patch.py && \
    make clean && \
    echo "CONFIG_LOW_LEVEL_OPTIONS=y" > .config && \
    echo "CONFIG_MACH_LINUX=y" >> .config && \
    echo 'CONFIG_BOARD_DIRECTORY="linux"' >> .config && \
    echo "CONFIG_CLOCK_FREQ=50000000" >> .config && \
    echo "CONFIG_LINUX_SELECT=y" >> .config && \
    echo "CONFIG_HAVE_GPIO=y" >> .config && \
    echo "CONFIG_HAVE_GPIO_ADC=y" >> .config && \
    echo "CONFIG_HAVE_GPIO_SPI=y" >> .config && \
    echo "CONFIG_HAVE_GPIO_I2C=y" >> .config && \
    echo "CONFIG_HAVE_GPIO_HARD_PWM=y" >> .config && \
    echo "CONFIG_INLINE_STEPPER_HACK=y" >> .config && \
    make olddefconfig && \
    make -j$(nproc)

# Binary an bekannten Ort kopieren
USER root
RUN cp /home/klippy/klipper/out/klipper.elf /usr/local/bin/klipper_mcu && \
    chmod +x /usr/local/bin/klipper_mcu

USER klippy
WORKDIR /home/klippy

# --- MOONRAKER ---
RUN git clone https://github.com/Arksine/moonraker.git

RUN python3 -m venv moonraker-env && \
    /home/klippy/moonraker-env/bin/pip install --upgrade pip && \
    /home/klippy/moonraker-env/bin/pip install -r /home/klippy/moonraker/scripts/moonraker-requirements.txt

# --- MAINSAIL ---
RUN mkdir -p /home/klippy/mainsail && \
    cd /home/klippy/mainsail && \
    wget -q https://github.com/mainsail-crew/mainsail/releases/latest/download/mainsail.zip && \
    python3 -c "import zipfile; zipfile.ZipFile('mainsail.zip').extractall('.')" && \
    rm mainsail.zip

# Verzeichnisse
RUN mkdir -p /home/klippy/printer_data/config \
             /home/klippy/printer_data/logs \
             /home/klippy/printer_data/gcodes \
             /home/klippy/printer_data/systemd \
             /home/klippy/printer_data/moonraker-db \
             /home/klippy/printer_data/comms

# Zurück zu root für System-Configs
USER root

# Supervisor, Nginx, Startscript kopieren
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY nginx.conf /etc/nginx/sites-available/default
COPY start.sh /start.sh
RUN chmod +x /start.sh

RUN chown -R klippy:klippy /home/klippy

EXPOSE 7125 80 7200

CMD ["/start.sh"]

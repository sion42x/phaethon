#!/bin/bash
#
# Read Phaethon (Nucleo) sensors with correct labels.
# Works around humility sensors not supporting pressure/humidity kinds,
# which causes it to skip those types and shift INA260 IDs by -2.
#
# Remapping (humility display → actual):
#   env temp         → env temp        (BMP280, correct)
#   nas_power power  → env pressure    (BMP280, mislabeled)
#   nas_power current→ env humidity    (BMP280, mislabeled)
#   nas_power voltage→ nas_power power (INA260, mislabeled)
#   (missing)        → nas_power current (INA260 ID 7)
#   (missing)        → nas_power voltage (INA260 ID 8)
#
# Usage: ./scripts/phaethon-sensors.sh [-s] [-a ARCHIVE]

set -euo pipefail

ARCHIVE_PATH="${ARCHIVE:-}"
LOOP=false

while getopts "sa:" opt; do
    case $opt in
        s) LOOP=true ;;
        a) ARCHIVE_PATH="$OPTARG" ;;
        *) echo "Usage: $0 [-s] [-a ARCHIVE]" >&2; exit 1 ;;
    esac
done

if [[ -z "$ARCHIVE_PATH" ]]; then
    echo "Error: set \$ARCHIVE or use -a <path>" >&2
    exit 1
fi

HUMILITY="humility -a $ARCHIVE_PATH"

print_sensors() {
    # Capture humility sensors output (fast — single probe read)
    local raw
    raw=$($HUMILITY sensors 2>&1 | grep -v "^humility:")

    # Print header
    printf "%-16s %-10s %12s\n" "NAME" "KIND" "VALUE"
    printf "%-16s %-10s %12s\n" "----" "----" "-----"

    # Parse each line and remap
    echo "$raw" | tail -n +2 | while IFS= read -r line; do
        # Extract fields: NAME KIND VALUE (rest is counters)
        local name kind value
        name=$(echo "$line" | awk '{print $1}')
        kind=$(echo "$line" | awk '{print $2}')
        value=$(echo "$line" | awk '{print $3}')

        case "${name}_${kind}" in
            cpu_temp)
                printf "%-16s %-10s %12s  C\n" "cpu" "temp" "$value" ;;
            ambient_temp)
                printf "%-16s %-10s %12s  C\n" "ambient" "temp" "$value" ;;
            nvme_temp)
                printf "%-16s %-10s %12s  C\n" "nvme" "temp" "$value" ;;
            env_temp)
                printf "%-16s %-10s %12s  C\n" "env" "temp" "$value" ;;
            nas_power_power)
                # Actually BMP280 pressure (sensor ID 4)
                printf "%-16s %-10s %12s  hPa\n" "env" "pressure" "$value" ;;
            nas_power_current)
                # Actually BMP280 humidity (sensor ID 5)
                printf "%-16s %-10s %12s  %%RH\n" "env" "humidity" "$value" ;;
            nas_power_voltage)
                # Actually INA260 power (sensor ID 6)
                printf "%-16s %-10s %12s  mW\n" "nas_power" "power" "$value" ;;
            *)
                printf "%-16s %-10s %12s\n" "$name" "$kind" "$value" ;;
        esac
    done

    # INA260 current (ID 7) and voltage (ID 8) are not shown by
    # humility at all. Do 2 hiffy calls only if INA260 is connected.
    # Skip for now since INA260 is disconnected — uncomment when ready:
    # val7=$($HUMILITY hiffy --call Sensor.get --arguments id=7 2>&1 | sed 's/.*=> //')
    # val8=$($HUMILITY hiffy --call Sensor.get --arguments id=8 2>&1 | sed 's/.*=> //')
    # printf "%-16s %-10s %12s  mA\n" "nas_power" "current" "$val7"
    # printf "%-16s %-10s %12s  mV\n" "nas_power" "voltage" "$val8"
}

if $LOOP; then
    while true; do
        clear
        echo "Phaethon Sensors ($(date '+%H:%M:%S'))  [Ctrl-C to stop]"
        echo
        print_sensors
        sleep 1
    done
else
    print_sensors
fi

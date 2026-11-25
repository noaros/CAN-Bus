#!/usr/bin/env bash
# Flash an ELF to the ST-LINK whose target has the given device ID.
#
#   tools/flash.sh <device-id> <elf>      e.g. tools/flash.sh 0x419 build/f429.elf
#
# With both Nucleos plugged in there are two ST-LINKs, and STM32_Programmer_CLI
# would otherwise use whichever it lists first, flashing one board with the
# other's firmware. Device IDs: 0x419 = STM32F42x/43x, 0x483 = STM32H72x/73x.
set -euo pipefail

want=$1
elf=$2
cli=${FLASHER:-STM32_Programmer_CLI}

strip() { sed 's/\x1b\[[0-9;]*m//g; s/\r//g'; }

serials=$("$cli" -l st-link | strip | sed -n 's/.*ST-LINK SN *: *\([0-9A-Fa-f]*\).*/\1/p')
if [ -z "$serials" ]; then
    echo "flash.sh: no ST-LINK found" >&2
    exit 1
fi

for sn in $serials; do
    id=$("$cli" -c port=SWD sn="$sn" mode=HOTPLUG | strip |
         sed -n 's/.*Device ID *: *\(0x[0-9A-Fa-f]*\).*/\1/p')
    if [ "${id,,}" = "${want,,}" ]; then
        echo "flash.sh: ST-LINK $sn has device $id, flashing $elf"
        exec "$cli" -c port=SWD sn="$sn" -w "$elf" -rst
    fi
done

echo "flash.sh: no connected ST-LINK has a target with device ID $want" >&2
exit 1

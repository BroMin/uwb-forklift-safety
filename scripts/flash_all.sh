#!/bin/bash

SERIALS=("760144486" "760197326")
HEX="/home/hyeongmin/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1/BuildOutput/CLI/FreeRTOS/DWM3001CDK/Debug/DWM3001CDK-CLI-FreeRTOS.hex"

for SN in "${SERIALS[@]}"; do
    echo "플래시 중: 보드 $SN"
    cat > /tmp/flash_$SN.jlink << JLINK
si 1
speed 4000
device nrf52833_xxaa
loadfile $HEX
r
g
exit
JLINK
    JLinkExe -SelectEmuBySN $SN -CommandFile /tmp/flash_$SN.jlink
    echo "완료: 보드 $SN"
done

echo "전체 플래시 완료!"

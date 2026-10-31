#!/bin/bash
# Invoked by build_windows.ps1; no network downloads and no flashing.
set -euo pipefail
repo=$1
sdk=$2
profile=${3:-Default}
test -f "$sdk/pico_sdk_init.cmake" || { echo 'Pico SDK path is invalid'; exit 1; }
if [[ "$sdk" != *' '* ]]; then
    export PICO_SDK_PATH="$sdk"
else
alias_path="/tmp/csc02-pico-sdk-$(printf '%s' "$sdk" | sha256sum | cut -c1-16)"
if [ -L "$alias_path" ]; then
    test "$(readlink "$alias_path")" = "$sdk" || { echo 'SDK alias mismatch'; exit 1; }
elif [ -e "$alias_path" ]; then
    echo 'SDK alias is occupied'; exit 1
else
    ln -s "$sdk" "$alias_path"
fi
export PICO_SDK_PATH="$alias_path"
fi
options=()
case "$profile" in
    Default) ;;
    Mqtt) test -f "$repo/config/wifi_secrets.h" || { echo 'Create your local config/wifi_secrets.h from its example first'; exit 1; }; options=(NET=lwip MQTT=1);;
    PassiveTest) options=(TEST=1);;
    SingleCore) options=(SMP=0);;
    Uart) options=(CONSOLE=uart WIFI=none);;
    S2Bench) options=(S2_ONLY=1 TEST=1);;
    S4Bench) options=(S4_ONLY=1);;
    IRRawAdc) options=(IR_RAW_MODE=2);;
    IRRawDigital) options=(IR_RAW_MODE=1);;
    IRCalibrated) options=(IR_CALIBRATED_BENCH=1);;
    IRBarcodeCapture) options=(IR_CALIBRATED_BENCH=1 IR_BARCODE_CAPTURE=1);;
    IRContinuous) options=(IR_MOTOR_SCAN=1 IR_LINE_TRACE=1 IR_LINE_ONLY=1 IR_CONTINUOUS=1 IR_BARCODE_CAPTURE=1 S2_COMMAND_TIMEOUT_MS=150);;
    IRLineOnly) options=(IR_MOTOR_SCAN=1 IR_LINE_TRACE=1 IR_LINE_ONLY=1 IR_BARCODE_CAPTURE=1 S2_COMMAND_TIMEOUT_MS=150);;
    IRLineTrace) options=(IR_MOTOR_SCAN=1 IR_LINE_TRACE=1 IR_BARCODE_CAPTURE=1 S2_COMMAND_TIMEOUT_MS=150);;
    IRMotorScan) options=(IR_MOTOR_SCAN=1 IR_BARCODE_CAPTURE=1 S2_COMMAND_TIMEOUT_MS=150);;
    *) echo 'Unknown build profile';exit 1;;
esac
make -C "$repo/build_make" -j8 "${options[@]}"

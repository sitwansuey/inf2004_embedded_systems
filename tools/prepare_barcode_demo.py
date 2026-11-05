"""Build and verify this robot's motor-enabled 30% BarcodeDemo. Never flashes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sdk", type=Path, required=True)
    p.add_argument("--arm-bin", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--allow-example-network", action="store_true", help="produce a clearly labelled network-incomplete candidate")
    p.add_argument("--wheel-calibration", action="store_true", help="secured raised-wheel bench; bounded ARM only, no line/ADC or autonomous START")
    p.add_argument("--wheel-pi", action="store_true", help="initial closed-loop PI trial, requires --wheel-calibration")
    p.add_argument("--course-pi", action="store_true", help="barcode course candidate with measured wheel PI; separate course validation required")
    a = p.parse_args()
    if a.wheel_pi and not a.wheel_calibration:
        p.error("--wheel-pi requires --wheel-calibration")
    if a.course_pi and a.wheel_calibration:
        p.error("--course-pi and --wheel-calibration are separate profiles")
    pi = a.wheel_pi or a.course_pi
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(a.arm_bin.resolve()), "C:/msys64/ucrt64/bin", "C:/msys64/usr/bin", env["PATH"]])
    flags = ["-D_PICO_RP2040_", "-DCNF_SMP=1", "-DAPP_TEST=0", "-DAPP_S2_ONLY=0", "-DAPP_S4_ONLY=0",
             "-DTM_CONSOLE_UART=0", "-DTM_CONSOLE_USB_CDC=1", "-DBOARD_UART0_FREE=1",
             "-DBOARD_BARCODE_DEMO=1", "-DTM_MQTT=1", "-DTM_WIFI_CYW43=1", "-DTM_NET_LWIP=1",
             "-DS2_ENCODER_ENABLED=1", "-DS2_COMMAND_TIMEOUT_MS=150"]
    if a.wheel_calibration:
        flags.remove("-DBOARD_BARCODE_DEMO=1")
        flags += ["-DBOARD_WHEEL_CAL_BENCH=1"]
        flags += ["-DBOARD_WHEEL_CAL_PI=" + str(int(a.wheel_pi))]
    if a.course_pi:
        flags += ["-DBOARD_BARCODE_WHEEL_PI=1"]
    command = ["arm-none-eabi-gcc", "-dM", "-E", "-x", "c", *flags, "-Iinclude", "-Iconfig", "-Ikernel/knlinc",
               "-include", "app_program/robot_config.h", "-include", "app_program/subsystem2/subsystem2.h", "-include", "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c",
               "-include", "device/adc/sysdepend/rp2040/adc_cnf_sysdep.h", "-"]
    pad_check = (b"#if DEVCONF_ADC_PIN_INIT_0 || DEVCONF_ADC_PIN_INIT_1 || DEVCONF_ADC_PIN_INIT_2 || DEVCONF_ADC_PIN_INIT_3\n#error Wheel bench must not initialize ADC pads\n#endif\n" if a.wheel_calibration else
        b"#if !DEVCONF_ADC_PIN_INIT_0 || !DEVCONF_ADC_PIN_INIT_1 || DEVCONF_ADC_PIN_INIT_2 || DEVCONF_ADC_PIN_INIT_3\n#error Incorrect mixed ADC/digital pad ownership\n#endif\n")
    dump = subprocess.check_output(command, cwd=ROOT, env=env, input=pad_check).decode()
    macros = {n: v.strip() for n, v in re.findall(r"^#define (\w+) (.+)$", dump, re.M)}
    while macros.get("BOARD_PID_ENABLED") in macros:
        macros["BOARD_PID_ENABLED"] = macros[macros["BOARD_PID_ENABLED"]]
    expected = {"BOARD_MOTORS_DISABLED": 0, "BOARD_PID_ENABLED": 0, "BOARD_DRIVE_POWER_CAP": 30,
                "S2_MOTOR1_TRIM_PERCENT": 100, "S2_MOTOR2_TRIM_PERCENT": 100, "BOARD_RIGHT_PWM_PERMILLE": 1000,
                "BOARD_IR_MODE": 3, "BOARD_IR_ADC_CALIBRATED": 1, "BOARD_IR_CENTER_WHITE": 1,
                "BOARD_BARCODE_DIGITAL_BLACK_LEVEL": 1,
                "APP_S5_ENABLED": 0, "S2_COMMAND_TIMEOUT_MS": 150, "BOARD_TERRAIN_CALIBRATED": 0,
                "BOARD_PID_CALIBRATED": 0, "BOARD_BYPASS_VALIDATED": 0, "BOARD_BARCODE_AUTO_START": 1,
                "CYW43_TASK_PRIORITY": 18, "BOARD_BARCODE_MIN_RUN_MS": 0, "BOARD_BARCODE_RETRY": 1,
                "BOARD_BARCODE_DEFAULT_FORWARD": 1,
                "BOARD_TICKS_PER_REV": 631, "BOARD_WHEEL_CIRCUMFERENCE_MM": 220, "BOARD_TRACK_WIDTH_MM": 113}
    if a.wheel_calibration:
        for name in ("BOARD_IR_ADC_CALIBRATED", "BOARD_BARCODE_DIGITAL_BLACK_LEVEL"):
            expected.pop(name)
        expected.update(BOARD_WHEEL_CAL_BENCH=1, BOARD_IR_MODE=0, BOARD_IR_CENTER_WHITE=0,
                        BOARD_BARCODE_AUTO_START=0, BOARD_BARCODE_MIN_RUN_MS=20,
                        BOARD_BARCODE_RETRY=0, BOARD_BARCODE_DEFAULT_FORWARD=0)
    if pi:
        expected.update(BOARD_PID_ENABLED=1, BOARD_PID_MAX_CPS=1667,
                        BOARD_PID_KP_MILLI=20, BOARD_PID_KI_MILLI=10,
                        BOARD_PID_KD_MILLI=0, BOARD_PID_MAX_POWER=30,
                        S2_MOTOR1_TRIM_PERCENT=74, S2_MOTOR2_TRIM_PERCENT=78,
                        BOARD_PID_LEFT_FF_OFFSET=3, BOARD_PID_RIGHT_FF_OFFSET=4,
                        BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE=1000,
                        BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE=970)
    expected["BOARD_BARCODE_SCAN_STEERING"]=int(a.course_pi)
    for n, v in expected.items():
        assert int(macros[n].rstrip("uUlL")) == v, (n, macros.get(n))
    if not a.wheel_calibration:
        assert macros["BOARD_IR_ADC_WHITE"].replace(" ", "") == "{187U,189U,0U}"
        assert macros["BOARD_IR_ADC_BLACK"].replace(" ", "") == "{3214U,3177U,1U}"
    (out / "EFFECTIVE_MACROS.txt").write_text(dump, encoding="utf-8")
    secrets = ROOT / "config/wifi_secrets.h"
    settings = secrets.read_text(encoding="utf-8")
    placeholders = any(x in settings for x in ('"your-network-name"', '"your-network-password"'))
    if placeholders and not a.allow_example_network:
        p.error("Configure config/wifi_secrets.h first, or explicitly allow a network-incomplete candidate")

    # Exercise the production PWM driver, using values from the actual ARM preprocessing.
    host = out / "host"
    for name in ("subsystem2", "shared", "tk"):
        (host / name).mkdir(parents=True, exist_ok=True)
    fixture = ROOT / "app_program/subsystem2/tests/host"
    for name in ("app.h", "board_pins.h", "subsystem2_host_test.c", "tk/syslib.h"):
        shutil.copy2(fixture / name, host / name)
    for name in ("subsystem2.c", "subsystem2.h", "subsystem2_test.c", "s2_data.c", "s2_data.h", "speed_pid.h", "wheel_steering.h"):
        shutil.copy2(ROOT / "app_program/subsystem2" / name, host / "subsystem2" / name)
    for name in ("edr.c", "edr.h", "encoder_snapshot.h"):
        shutil.copy2(ROOT / "app_program/shared" / name, host / "shared" / name)
    constants = ("BOARD_MOTORS_DISABLED", "BOARD_PID_ENABLED", "BOARD_DRIVE_POWER_CAP", "S2_MOTOR1_TRIM_PERCENT", "S2_MOTOR2_TRIM_PERCENT")
    if pi:
        constants += ("BOARD_PID_MAX_CPS", "BOARD_PID_KP_MILLI", "BOARD_PID_KI_MILLI", "BOARD_PID_KD_MILLI", "BOARD_PID_MAX_POWER", "BOARD_PID_LEFT_FF_OFFSET", "BOARD_PID_RIGHT_FF_OFFSET", "BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE", "BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE")
    (host / "robot_config.h").write_text("".join(f"#define {n} {macros[n]}\n" for n in constants), encoding="utf-8")
    pwm_test = '''#define main existing_suite
#include "subsystem2_host_test.c"
#undef main
int main(void) {
 setup();
 for(unsigned p=1;p<=30;++p) {
  assert(movement_valid(S2_MOVE_FORWARD,p));
  assert(motor_apply(S2_MOVE_FORWARD,p,now_ms)==E_OK);
  assert(duties[8]==6250U*p/100U && duties[9]==0 && duties[10]==0 && duties[11]==6250U*p/100U);
 }
 assert(!movement_valid(S2_MOVE_FORWARD,31U));
 assert(motor_apply(S2_MOVE_FORWARD,100U,now_ms)==E_OK);
 assert(duties[8]==1875U && duties[11]==1875U);
 motor_stop();assert_off();
 puts("Production motor: equal 30% PWM=1875/6250, all 1..30 duties, 31% rejected, internal cap and STOP PASS");return 0;
}
'''
    if pi:
        for pin, trim in ((8,74), (11,78)):
            pwm_test = pwm_test.replace(f'duties[{pin}]==6250U*p/100U', f'duties[{pin}]==6250U*p*{trim}U/10000U')
            pwm_test = pwm_test.replace(f'duties[{pin}]==1875U', f'duties[{pin}]==6250U*30U*{trim}U/10000U')
        pwm_test = pwm_test.replace('equal 30% PWM=1875/6250', 'measured trims74/78; raw cap retained')
        pwm_test = pwm_test.replace(' puts("Production motor:', '''
 setup();run_task();stopped_epoch=stop_epoch;
 for(unsigned command=S2_MOVE_FORWARD;command<=S2_MOVE_PIVOT_LEFT;++command) {
  motor_stop();
  uint16_t word;
  assert(edr_motor_pack(command,30,S2_MOTION_MOVING,&word));
  pid_initialized=FALSE;
  now_ms+=100;assert(motor_regulate(word)==E_OK);
  now_ms+=20;assert(motor_regulate(word)==E_OK);
  if(command!=S2_MOVE_PIVOT_LEFT)
   assert(wheel_pid[0].previous_error+pid_speed[0]==500);
  if(command!=S2_MOVE_PIVOT_RIGHT)
   assert(wheel_pid[1].previous_error+pid_speed[1]==(command==S2_MOVE_FORWARD?485:500));
  for(unsigned pin=8;pin<=11;++pin)assert(duties[pin]<=1875U);
 }
 motor_stop();assert_off();
 puts("Production PI: forward target 500/485; reverse/turn references unchanged; cap and STOP PASS");
 uint16_t fine_word;
 assert(edr_motor_pack(S2_MOVE_FORWARD,30,S2_MOTION_MOVING,&fine_word));
 pid_initialized=FALSE;now_ms+=100;assert(motor_regulate(fine_word)==E_OK);
 pid_speed[0]=700;pid_speed[1]=700;
 now_ms+=20;assert(motor_regulate(fine_word)==E_OK);
 assert(wheel_pid[0].previous_error==-25 && wheel_pid[1].previous_error==-40);
 assert(duties[8]==1530U && duties[11]==1637U);
 motor_stop();assert_off();
 puts("Production PI: fractional corrections reach actual PWM (24.495/26.192 percent), STOP PASS");
 puts("Production motor:''')
    (host / "demo_pwm.c").write_text(pwm_test, encoding="utf-8")
    with (out / "PWM_TEST.log").open("w", encoding="utf-8") as log:
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-DAPP_TEST=0",
                        "-DAPP_S2_ONLY=0", "-DS2_ENCODER_ENABLED=1", "-DS2_COMMAND_TIMEOUT_MS=150", "-include", host / "robot_config.h",
                        "-I", host, host / "demo_pwm.c", host / "subsystem2/subsystem2_test.c", host / "subsystem2/s2_data.c",
                        host / "shared/edr.c", "-o", host / "demo_pwm.exe"], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run([host / "demo_pwm.exe"], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)

    with (out / "BUILD.log").open("w", encoding="utf-8") as log:
        subprocess.run(["mingw32-make", "-C", "build_make", "-j1", "SHELL=C:/msys64/usr/bin/sh.exe",
                        "PICO_SDK_PATH=" + a.sdk.resolve().as_posix(), "E2U=tools/elf2uf2.exe", "NET=lwip", "MQTT=1",
                        "BARCODE_DEMO=" + ("0" if a.wheel_calibration else "1"),
                        "WHEEL_CAL=" + ("1" if a.wheel_calibration else "0"),
                        "WHEEL_CAL_PI=" + str(int(a.wheel_pi)),
                        "BARCODE_WHEEL_PI=" + str(int(a.course_pi)),
                        "S2_COMMAND_TIMEOUT_MS=150"], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    stem = ROOT / ("build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_tmo150_" + ("wheelcalpid" + str(int(a.wheel_pi)) if a.wheel_calibration else "barcodewheelpi30" if a.course_pi else "barcodedemo30"))
    uf2, elf = stem.with_suffix(".uf2").read_bytes(), stem.with_suffix(".elf").read_bytes()
    assert len(uf2) % 512 == 0 and elf[:6] == b"\x7fELF\x01\x01"
    flash = bytearray()
    for i in range(len(uf2) // 512):
        b = uf2[i*512:(i+1)*512]
        m0,m1,flags,addr,size,index,count,family = struct.unpack_from("<8I", b)
        assert (m0,m1,flags,addr,size,index,count,family) == (0x0A324655,0x9E5D5157,0x2000,0x10000000+i*256,256,i,len(uf2)//512,0xE48BFF56)
        assert addr+256 <= 0x10200000 and struct.unpack_from("<I",b,508)[0] == 0x0AB16F30
        flash += b[32:288]
    phoff = struct.unpack_from("<I",elf,28)[0]
    entsize, count = struct.unpack_from("<HH",elf,42)
    compared = 0
    for i in range(count):
        typ,offset,va,pa,filesz,memsz,fl,align = struct.unpack_from("<8I",elf,phoff+i*entsize)
        if typ == 1 and 0x10000000 <= pa < 0x10200000:
            assert flash[pa-0x10000000:pa-0x10000000+filesz] == elf[offset:offset+filesz]
            compared += filesz
    markers = ((b"WHEEL CAL: bounded forward ARM only", b"wheels raised", b"[WHEEL CAL ARM]", b"csc02/barcode/map", b"edr16/v2") if a.wheel_calibration else
        (b"BARCODE DEMO30: motors ENABLED", b"DIGITAL GP28", b"invalid capture=discard/retry", b"no decoded/mapped instruction=FORWARD", b"[BARCODE REJECT]", b"[BARCODE AUTO START]", b"csc02/barcode/map", b"edr16/v2"))
    for marker in markers:
        assert marker in flash, marker
    assert b"MOTOR LOCK ON" not in flash and compared > 0
    assert b"Robopico/diagnostics/barcode/v1" in flash
    if not a.wheel_calibration:
        assert b"BARCODE CAPTURE: active scan survives both-black" in flash
        assert b"BARCODE RESYNC: retry capture slides up to8" in flash
        assert b"BARCODE WIDTH FIT: compensated separation" in flash
    symbols = subprocess.check_output(["arm-none-eabi-nm", "--defined-only", stem.with_suffix(".elf")], env=env).decode()
    # The optimizer may inline motor_regulate; its persistent PI state remains.
    assert (" wheel_pid" in symbols) == pi
    if a.wheel_pi:
        assert b"[WHEEL PI]" in flash
    if a.course_pi:
        assert b"measured wheel PI" in flash and b"[WHEEL PI]" not in flash
        assert b"BARCODE WHEEL PI: fractional PWM" in flash
        assert b"BARCODE LINE CORRECTION: forward-only analog P" in flash
    if a.wheel_calibration:
        assert "wheel_calibration_task" in symbols and b"[BARCODE AUTO START]" not in flash
    else:
        assert "vehicle_task" in symbols and "barcode_boot_step" in symbols
    artifact = out / ("CSC02_BARCODE_DEMO30_NETWORK_NOT_CONFIGURED.uf2" if placeholders else "CSC02_BARCODE_DEMO30_DIGITAL_GP28_MQTT_AUTO.uf2")
    if a.wheel_calibration:
        assert not placeholders
        artifact = out / "CSC02_WHEEL_CAL_MQTT_30PCT.uf2"
        if a.wheel_pi:
            artifact = out / "CSC02_WHEEL_PI_TRIAL_MQTT_30PCT.uf2"
    if a.course_pi:
        artifact = out / "CSC02_BARCODE_WHEEL_PI_30PCT_MQTT_AUTO.uf2"
    artifact.write_bytes(uf2)
    files = subprocess.check_output(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT).decode().split("\0")
    hashes = {n: digest(ROOT / n) for n in sorted(set(files)) if n and (ROOT/n).is_file()}
    result = {"artifact": str(artifact), "sha256": digest(artifact), "effective_configuration": expected,
              "elf_flash_bytes_compared": compared, "PWM_test": "PASS", "network_placeholders_detected": placeholders,
              "credentials_sha256": digest(secrets), "auto_start": "received map + fresh ADC/encoder + white GP28 stable for 3000ms, once per boot",
              "source_hashes": hashes, "flashed": False, "pushed": False, "physical_turn_accuracy_validated": False}
    result["barcode_diagnostics"] = {"topic":"Robopico/diagnostics/barcode/v1", "version":1,
        "bytes":192,"qos":1,"retained":False,"heartbeat_ms":1000,
        "change_observation_ms":200,"queue_records":4,"pulse_widths":29,
        "EDR_cursor_preserved_on_snapshot_expiry":True}
    if not a.wheel_calibration:
        result["junction_capture_policy"]="active retry capture continues through both-black; overlap latched until clearance; next junction uses stored symbol"
        result["barcode_resynchronisation"]={"retry_profiles_only":True,"max_prefix_pairs":8,
            "window_runs":29,"absolute_timeout_ms":10000,"full_framing_and_trailing_white_required":True,
            "wire_format_changed":False,"physical_validation":"pending"}
        result["barcode_width_fit"]={"unit_estimator":"mean of six narrow elements",
            "separation":"after bounded edge compensation","gaps":"joint fit with adjacent character biases",
            "width_tolerance_percent":25,"max_edge_bias_percent_of_unit":50,
            "full_framing_required":True,"recorded_F_regression":"PASS forward/reverse",
            "physical_validation":"pending"}
    if a.wheel_calibration:
        result["auto_start"] = "disabled; explicit forward ARM lease 1..3000ms only; START refused"
        assert b"WHEEL CAL: forward lease maximum3000ms" in flash
        result["maximum_forward_lease_ms"] = 3000
    if pi:
        result["PID_output_precision"] = "milli-percent corrections converted directly to PWM ticks"
        result["physical_PID_calibrated"] = False
    if a.course_pi:
        result["wheel_PI_validation"] = "wireless3000ms floor check1576/1583 counts, user confirmed much straighter; full course/turn validation pending"
        result["forward_balance_trial"] = {"left_target_permille":1000,"right_target_permille":970,
            "physical_validation":"one wireless3000ms straight floor pass1576/1583; user confirmed much straighter; course pending"}
        result["physical_PID_calibrated"] = False
        result["barcode_scan_line_correction"] = {"enabled":True,"max_steering_permille":100,
            "normalized_darkness_deadband":80,"P_gain":0.12,"error_filter":"3previous+new /4 per fresh sample",
            "motion":"forward only; both wheel targets positive; reset on pause/fault/both-black"}
    (out / "BUILD_RESULT.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k:v for k,v in result.items() if k not in ("source_hashes", "credentials_sha256")}, indent=2))


if __name__ == "__main__":
    main()

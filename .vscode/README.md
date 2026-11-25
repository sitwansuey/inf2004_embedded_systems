# Pico W IntelliSense

Open the repository root as the VS Code workspace.
`c_cpp_properties.json` provides repository include paths and preprocessor
definitions for the editor. It does not build firmware.

| Configuration | `compilerPath` | IntelliSense mode |
|---|---|---|
| `Win32` | `${env:USERPROFILE}/.pico-sdk/toolchain/15_2_Rel1/bin/arm-none-eabi-gcc.exe` | `windows-gcc-arm` |
| `Linux` | `/usr/bin/arm-none-eabi-gcc` | `linux-gcc-arm` |

VS Code selects the configuration named after your platform: `Win32` on
Windows, `Linux` on Linux or in a WSL window. If the compiler is installed
elsewhere, change that configuration's `compilerPath` with **C/C++: Edit
Configurations (UI)**; it must point to the Arm GCC, not a host compiler.
Both configurations pass `-mcpu=cortex-m0plus`, `-mthumb` and `-ffreestanding`,
and select `gnu11` for C and `gnu++14` for C++.

The compiler supplies standard headers and built-in definitions; the explicit
IntelliSense mode models Arm data sizes.

`picoIncludePath` covers `include/`, `config/`, `kernel/knlinc/`,
`kernel/tkernel/` and `app_program/`. It does not include external Pico SDK
headers; add the include paths used by your SDK build when inspecting files
that depend on those headers.

After changing the configuration, run **C/C++: Reset IntelliSense Database**,
then **Developer: Reload Window** if old include errors remain. Open
`kernel/tkernel/time_calls.c` before `time_calls.h`: the header has no includes
of its own and takes its types from the source file.

`picoDefines` models the dual-core MQTT mission build: the target macro
`_PICO_RP2040_`, `CNF_SMP=1`, `APP_TEST=0`, both subsystem-only switches off,
encoders enabled, `S2_COMMAND_TIMEOUT_MS=1000`, USB console, no UART mirror,
Wi-Fi, lwIP, MQTT and `BOARD_UART0_FREE=1`. Adjust the editor definitions when
inspecting another build. Changing them does not change Make options.

See Microsoft's [cross-compiling guide](https://code.visualstudio.com/docs/cpp/configure-intellisense-crosscompilation).

Build through `build_make/` or `tools/build_windows.ps1` as described in the
[root README](../README.md). See [TEAM.md](../TEAM.md) for hardware ownership.

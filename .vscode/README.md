# Pico W IntelliSense

Open the repository root as the VS Code workspace. `c_cpp_properties.json`
supplies the same project include roots and Pico target definition used by the
Make build. This lets `time_calls.c` find `kernel.h`, `config.h` and the target
headers without adding SDK directories to kernel source code.

- **Windows (`Win32`)** uses the installed Pico extension toolchain at
  `%USERPROFILE%/.pico-sdk/toolchain/15_2_Rel1/bin/arm-none-eabi-gcc.exe`.
- **Linux/WSL (`Linux`)** uses `/usr/bin/arm-none-eabi-gcc`.

VS Code automatically chooses the matching platform profile. If your toolchain
is elsewhere, change that profile's `compilerPath` through **C/C++: Edit
Configurations (UI)**. The path must point to ARM GCC. The compiler supplies
standard headers and built-in definitions; the explicit mode models ARM data
sizes. See Microsoft's [cross-compiling configuration guide](https://code.visualstudio.com/docs/cpp/configure-intellisense-crosscompilation).

The default editor profile mirrors dual-core production configuration
(`CNF_SMP=1`, `APP_TEST=0`, encoders disabled). Change `picoDefines` if inspecting
a different Make profile. Firmware still builds through `build_make`; this
configuration does not change the firmware or the shared data contract.

After adding/updating the configuration, run **C/C++: Reset IntelliSense
Database**, then **Developer: Reload Window** if VS Code still shows cached
include errors. Open `time_calls.c` before its private header so the extension
uses the source file's include context for `time_calls.h`.

These paths configure repository/kernel headers. Files using external Pico SDK
drivers additionally need the include directories for the SDK installation used
by their build; keep those specific to the selected target and SDK version.

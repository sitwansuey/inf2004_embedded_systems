# Buddy 4: Pico SDK I2C, timeouts, tick and GPIO helpers.
# Included after pico_rp2040.mk so the WiFi source set is already known.
S4_SDK := $(PICO_SDK_PATH)/src
ifeq ($(wildcard $(S4_SDK)/rp2_common/hardware_i2c/i2c.c),)
$(error Buddy 4 needs PICO_SDK_PATH pointing to Pico SDK 2.2.0, including for UART builds)
endif

S4_INCLUDES := common/pico_base_headers common/pico_binary_info \
               common/pico_time common/hardware_claim \
               rp2040/pico_platform rp2040/hardware_regs \
               rp2040/hardware_structs \
               rp2_common/pico_platform_compiler \
               rp2_common/pico_platform_sections \
               rp2_common/pico_platform_panic rp2_common/pico_platform_common \
               rp2_common/hardware_base rp2_common/hardware_irq \
               rp2_common/hardware_sync rp2_common/hardware_sync_spin_lock \
               rp2_common/hardware_clocks rp2_common/hardware_resets \
               rp2_common/hardware_i2c rp2_common/hardware_gpio \
               rp2_common/hardware_timer rp2_common/hardware_ticks \
               rp2_common/hardware_watchdog

CFLAGS += -DPICO_NO_HARDWARE=0 -DNDEBUG -ffunction-sections -fdata-sections \
          -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
          $(foreach path,$(S4_INCLUDES),-isystem "$(S4_SDK)/$(path)/include")
LFLAGS += -Wl,--gc-sections

S4_SDK_OBJS := mtkernel_3/subsystem4_sdk/i2c.o \
               mtkernel_3/subsystem4_sdk/timeout_helper.o \
               mtkernel_3/subsystem4_sdk/ticks.o
ifeq ($(WIFI),none)
# WiFi already compiles gpio.c and supplies time_us_64/clock_get_hz. Do not
# import duplicate symbols or SDK IRQ dispatch/startup when it is enabled.
S4_SDK_OBJS += mtkernel_3/subsystem4_sdk/gpio.o \
               mtkernel_3/subsystem4_sdk/timer.o
endif
OBJS += $(S4_SDK_OBJS)
C_DEPS += $(S4_SDK_OBJS:.o=.d)

mtkernel_3/subsystem4_sdk/i2c.o: $(S4_SDK)/rp2_common/hardware_i2c/i2c.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/subsystem4_sdk/timeout_helper.o: $(S4_SDK)/common/pico_time/timeout_helper.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/subsystem4_sdk/ticks.o: $(S4_SDK)/rp2_common/hardware_ticks/ticks.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/subsystem4_sdk/gpio.o: $(S4_SDK)/rp2_common/hardware_gpio/gpio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/subsystem4_sdk/timer.o: $(S4_SDK)/rp2_common/hardware_timer/timer.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

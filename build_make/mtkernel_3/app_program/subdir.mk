################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
ifeq ($(S2_ONLY),1)
TEMP_SRCS = ../app_program/app_main.c ../app_program/subsystem2/subsystem2.c \
            ../app_program/subsystem2/subsystem2_test.c \
            ../app_program/subsystem2/s2_data.c ../app_program/shared/edr.c
else ifeq ($(S4_ONLY),1)
TEMP_SRCS = ../app_program/app_main.c $(wildcard ../app_program/subsystem4/*.c) \
            ../app_program/shared/edr.c
else
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
endif
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

ifneq ($(S2_ONLY),1)
include ../app_program/subsystem4/sdk.mk

mtkernel_3/app_program/subsystem4/%.o: CFLAGS += -Wall -Wextra -Werror
endif

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include sources/src/subdir.mk
-include freertos/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: TDIII_Parcial.axf

# Tool invocations
TDIII_Parcial.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Juan Diego\Documents\MCUXpressoIDE_11.3.0_5222\workspace\lpc_chip_175x_6x\Debug" -L"C:\Users\Juan Diego\Documents\MCUXpressoIDE_11.3.0_5222\workspace\lpc_board_nxp_lpcxpresso_1769\Debug" -Xlinker -Map="TDIII_Parcial.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T TDIII_Parcial_Debug.ld -o "TDIII_Parcial.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) TDIII_Parcial.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "TDIII_Parcial.axf"; # arm-none-eabi-objcopy -v -O binary "TDIII_Parcial.axf" "TDIII_Parcial.bin" ; # checksum -p LPC1769 -d "TDIII_Parcial.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include sources/src/subdir.mk
-include freertos/src/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := TDIII_Parcial
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: TDIII_Parcial.axf

# Tool invocations
TDIII_Parcial.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\Debug" -L"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\Debug" -Xlinker -Map="TDIII_Parcial.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T TDIII_Parcial_Debug.ld -o "TDIII_Parcial.axf" $(OBJS) $(USER_OBJS) $(LIBS) -llpc_board_nxp_lpcxpresso_1769 -llpc_chip_175x_6x
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) TDIII_Parcial.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "TDIII_Parcial.axf"; # arm-none-eabi-objcopy -v -O binary "TDIII_Parcial.axf" "TDIII_Parcial.bin" ; # checksum -p LPC1769 -d "TDIII_Parcial.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets

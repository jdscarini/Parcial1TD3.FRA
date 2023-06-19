################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include freertos/src/subdir.mk
-include example/src/subdir.mk
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


BUILD_ARTIFACT_NAME := ParcialTD3
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: ParcialTD3.axf

# Tool invocations
ParcialTD3.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\Debug" -L"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\Debug" -Xlinker -Map="ParcialTD3.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "ParcialTD3_Debug.ld" -o "ParcialTD3.axf" $(OBJS) $(USER_OBJS) $(LIBS) -llpc_board_nxp_lpcxpresso_1769 -llpc_chip_175x_6x
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ParcialTD3.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "ParcialTD3.axf"; # arm-none-eabi-objcopy -v -O binary "ParcialTD3.axf" "ParcialTD3.bin" ; # checksum -p LPC1769 -d "ParcialTD3.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets

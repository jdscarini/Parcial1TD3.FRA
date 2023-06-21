################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/board.c \
../src/board_sysinit.c \
../src/lpc_phy_smsc87x0.c 

C_DEPS += \
./src/board.d \
./src/board_sysinit.d \
./src/lpc_phy_smsc87x0.d 

OBJS += \
./src/board.o \
./src/board_sysinit.o \
./src/lpc_phy_smsc87x0.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M3 -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\inc" -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/board.d ./src/board.o ./src/board_sysinit.d ./src/board_sysinit.o ./src/lpc_phy_smsc87x0.d ./src/lpc_phy_smsc87x0.o

.PHONY: clean-src


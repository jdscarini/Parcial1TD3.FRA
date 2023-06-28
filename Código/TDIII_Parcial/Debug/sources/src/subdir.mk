################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sources/src/cr_startup_lpc175x_6x.c \
../sources/src/lcd1602_i2c.c \
../sources/src/main.c \
../sources/src/sysinit.c 

C_DEPS += \
./sources/src/cr_startup_lpc175x_6x.d \
./sources/src/lcd1602_i2c.d \
./sources/src/main.d \
./sources/src/sysinit.d 

OBJS += \
./sources/src/cr_startup_lpc175x_6x.o \
./sources/src/lcd1602_i2c.o \
./sources/src/main.o \
./sources/src/sysinit.o 


# Each subdirectory must supply rules for building sources it contributes
sources/src/%.o: ../sources/src/%.c sources/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M3 -I"C:\Users\Juan Diego\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\inc" -I"C:\Users\Juan Diego\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\inc" -I"C:\Users\Juan Diego\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\TDIII_Parcial\sources\inc" -I"C:\Users\Juan Diego\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\TDIII_Parcial\freertos\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-sources-2f-src

clean-sources-2f-src:
	-$(RM) ./sources/src/cr_startup_lpc175x_6x.d ./sources/src/cr_startup_lpc175x_6x.o ./sources/src/lcd1602_i2c.d ./sources/src/lcd1602_i2c.o ./sources/src/main.d ./sources/src/main.o ./sources/src/sysinit.d ./sources/src/sysinit.o

.PHONY: clean-sources-2f-src


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../example/src/ParcialTD3.c \
../example/src/cr_startup_lpc175x_6x.c \
../example/src/sysinit.c 

C_DEPS += \
./example/src/ParcialTD3.d \
./example/src/cr_startup_lpc175x_6x.d \
./example/src/sysinit.d 

OBJS += \
./example/src/ParcialTD3.o \
./example/src/cr_startup_lpc175x_6x.o \
./example/src/sysinit.o 


# Each subdirectory must supply rules for building sources it contributes
example/src/%.o: ../example/src/%.c example/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M3 -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\inc" -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\inc" -I"D:\Juan\Documents\GitHub\Parcial1TD3.FRA\Código\Prueba lectura de pulsos\example\inc" -I"D:\Juan\Documents\GitHub\Parcial1TD3.FRA\Código\Prueba lectura de pulsos\freertos\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-example-2f-src

clean-example-2f-src:
	-$(RM) ./example/src/ParcialTD3.d ./example/src/ParcialTD3.o ./example/src/cr_startup_lpc175x_6x.d ./example/src/cr_startup_lpc175x_6x.o ./example/src/sysinit.d ./example/src/sysinit.o

.PHONY: clean-example-2f-src


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/src/FreeRTOSCommonHooks.c \
../freertos/src/croutine.c \
../freertos/src/event_groups.c \
../freertos/src/heap_3.c \
../freertos/src/list.c \
../freertos/src/port.c \
../freertos/src/queue.c \
../freertos/src/stream_buffer.c \
../freertos/src/tasks.c \
../freertos/src/timers.c 

C_DEPS += \
./freertos/src/FreeRTOSCommonHooks.d \
./freertos/src/croutine.d \
./freertos/src/event_groups.d \
./freertos/src/heap_3.d \
./freertos/src/list.d \
./freertos/src/port.d \
./freertos/src/queue.d \
./freertos/src/stream_buffer.d \
./freertos/src/tasks.d \
./freertos/src/timers.d 

OBJS += \
./freertos/src/FreeRTOSCommonHooks.o \
./freertos/src/croutine.o \
./freertos/src/event_groups.o \
./freertos/src/heap_3.o \
./freertos/src/list.o \
./freertos/src/port.o \
./freertos/src/queue.o \
./freertos/src/stream_buffer.o \
./freertos/src/tasks.o \
./freertos/src/timers.o 


# Each subdirectory must supply rules for building sources it contributes
freertos/src/%.o: ../freertos/src/%.c freertos/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M3 -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_chip_175x_6x\inc" -I"C:\Users\Juan\OneDrive - UTN.BA\5to_-_TDIII\FRA\workspace\lpc_board_nxp_lpcxpresso_1769\inc" -I"D:\Juan\Documents\GitHub\Parcial1TD3.FRA\Código\Prueba lectura de pulsos\example\inc" -I"D:\Juan\Documents\GitHub\Parcial1TD3.FRA\Código\Prueba lectura de pulsos\freertos\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m3 -mthumb -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos-2f-src

clean-freertos-2f-src:
	-$(RM) ./freertos/src/FreeRTOSCommonHooks.d ./freertos/src/FreeRTOSCommonHooks.o ./freertos/src/croutine.d ./freertos/src/croutine.o ./freertos/src/event_groups.d ./freertos/src/event_groups.o ./freertos/src/heap_3.d ./freertos/src/heap_3.o ./freertos/src/list.d ./freertos/src/list.o ./freertos/src/port.d ./freertos/src/port.o ./freertos/src/queue.d ./freertos/src/queue.o ./freertos/src/stream_buffer.d ./freertos/src/stream_buffer.o ./freertos/src/tasks.d ./freertos/src/tasks.o ./freertos/src/timers.d ./freertos/src/timers.o

.PHONY: clean-freertos-2f-src


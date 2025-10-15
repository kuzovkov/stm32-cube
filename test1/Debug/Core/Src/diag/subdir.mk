################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/diag/Trace.c \
../Core/Src/diag/trace_impl.c 

OBJS += \
./Core/Src/diag/Trace.o \
./Core/Src/diag/trace_impl.o 

C_DEPS += \
./Core/Src/diag/Trace.d \
./Core/Src/diag/trace_impl.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/diag/%.o Core/Src/diag/%.su Core/Src/diag/%.cyclo: ../Core/Src/diag/%.c Core/Src/diag/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-diag

clean-Core-2f-Src-2f-diag:
	-$(RM) ./Core/Src/diag/Trace.cyclo ./Core/Src/diag/Trace.d ./Core/Src/diag/Trace.o ./Core/Src/diag/Trace.su ./Core/Src/diag/trace_impl.cyclo ./Core/Src/diag/trace_impl.d ./Core/Src/diag/trace_impl.o ./Core/Src/diag/trace_impl.su

.PHONY: clean-Core-2f-Src-2f-diag


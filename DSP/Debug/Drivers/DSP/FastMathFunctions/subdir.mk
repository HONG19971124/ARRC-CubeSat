################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DSP/FastMathFunctions/arm_cos_f32.c \
../Drivers/DSP/FastMathFunctions/arm_cos_q15.c \
../Drivers/DSP/FastMathFunctions/arm_cos_q31.c \
../Drivers/DSP/FastMathFunctions/arm_sin_f32.c \
../Drivers/DSP/FastMathFunctions/arm_sin_q15.c \
../Drivers/DSP/FastMathFunctions/arm_sin_q31.c \
../Drivers/DSP/FastMathFunctions/arm_sqrt_q15.c \
../Drivers/DSP/FastMathFunctions/arm_sqrt_q31.c 

OBJS += \
./Drivers/DSP/FastMathFunctions/arm_cos_f32.o \
./Drivers/DSP/FastMathFunctions/arm_cos_q15.o \
./Drivers/DSP/FastMathFunctions/arm_cos_q31.o \
./Drivers/DSP/FastMathFunctions/arm_sin_f32.o \
./Drivers/DSP/FastMathFunctions/arm_sin_q15.o \
./Drivers/DSP/FastMathFunctions/arm_sin_q31.o \
./Drivers/DSP/FastMathFunctions/arm_sqrt_q15.o \
./Drivers/DSP/FastMathFunctions/arm_sqrt_q31.o 

C_DEPS += \
./Drivers/DSP/FastMathFunctions/arm_cos_f32.d \
./Drivers/DSP/FastMathFunctions/arm_cos_q15.d \
./Drivers/DSP/FastMathFunctions/arm_cos_q31.d \
./Drivers/DSP/FastMathFunctions/arm_sin_f32.d \
./Drivers/DSP/FastMathFunctions/arm_sin_q15.d \
./Drivers/DSP/FastMathFunctions/arm_sin_q31.d \
./Drivers/DSP/FastMathFunctions/arm_sqrt_q15.d \
./Drivers/DSP/FastMathFunctions/arm_sqrt_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DSP/FastMathFunctions/arm_cos_f32.o: ../Drivers/DSP/FastMathFunctions/arm_cos_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_cos_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_cos_q15.o: ../Drivers/DSP/FastMathFunctions/arm_cos_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_cos_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_cos_q31.o: ../Drivers/DSP/FastMathFunctions/arm_cos_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_cos_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_sin_f32.o: ../Drivers/DSP/FastMathFunctions/arm_sin_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_sin_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_sin_q15.o: ../Drivers/DSP/FastMathFunctions/arm_sin_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_sin_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_sin_q31.o: ../Drivers/DSP/FastMathFunctions/arm_sin_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_sin_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_sqrt_q15.o: ../Drivers/DSP/FastMathFunctions/arm_sqrt_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_sqrt_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/DSP/FastMathFunctions/arm_sqrt_q31.o: ../Drivers/DSP/FastMathFunctions/arm_sqrt_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -DARM_MATH_CM4 '-D__FPU_PRESENT=1U' -c -I../Drivers/DSP/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/DSP/FastMathFunctions/arm_sqrt_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


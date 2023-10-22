#######################################
# Generic Makefile (based on file automatically-generated by tool: [projectgenerator] version: [3.17.1])
#
# @author: JIANG Yicheng (RM2024)
#
# This makefile is only for HKUST Enterprize RM2024 internal competition.
#######################################

######################################
# target
######################################
TARGET = RM2024-Internal-Root


######################################
# building variables
######################################

# debug option
#-g0	no debug information
#-g1	minimal debug information
#-g		default debug information
#-g3	maximal debug information
DEBUG = -g3

# optimization
# -O0 		no optimization
# -O1 		
# -O2
# -O3
# -Os 		optimize for size
# -Ofast 	optimize for speed
# -Og 		optimize for debugging
# do not use -flto, it will make the code undebuggable
OPT = -O3

# silent mode
VERBOSE = 0

# C standard
CSTD = -std=gnu11

# C++ standard
CPPSTD = -std=gnu++14

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/can.c \
Core/Src/usart.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Core/Src/system_stm32f1xx.c \
Core/Src/tim.c \
Core/Src/dma.c \
Core/Src/stm32f1xx_hal_timebase_tim.c

# CPP sources
CPP_SOURCES =  \
Diagnostic/Hook.cpp \
$(wildcard Core/Src/*.cpp)

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s

 
#######################################
# CFLAGS
#######################################

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xB

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/DSP/Include


include Core.mk

# *** EOF ***
#Author: Ahmed Ashraf
#Purpose: This makefile is used for ARM development purposes
#Version: v1.0
#Date: 19-06-2022

##################################### Development parameters ######################################

#CPP include directory
CPP_INC_DIR := -I../LIB

#CPP source files
CPP_SRC_FILES := ../main.cpp
CPP_SRC_FILES += ../stm32f10x_startupCode.cpp

#CPP object files
CPP_OBJ_FILES := main.o
CPP_OBJ_FILES += stm32f10x_startupCode.o

#Linker script file name
LD_FILE := ../stm32f10x_linkerScript.ld

#ELF file name
ELF_FILE := main.elf

#HEX file name
HEX_FILE := main.hex

#MAP file name
MAP_FILE := main.map

#Defining the CPU parameters
CPU := cortex-m3

#Defining compiler parameters
COMPILER := g++
COMPILER_START := arm-none-eabi-
COMPILER_OPTIONS := -Wall -Os -mthumb -fno-exceptions -fno-builtin -mcpu=$(CPU) $(CPP_INC_DIR)

# -fno-exceptions: Suppress all the exceptions
# -fno-builtin: Supress all the builtin functions like: memset and memcpy

############################################## Rules ##############################################

#Rebuilding binaries command - DEP[clean, elf file]
rebuild: clean main.hex

#Building binaries command - DEP[elf file]
build: main.hex

#Generating the elf file
main.elf:
	@echo "Building target..."
	@echo "Invoking ARM Compiler..."
	@$(COMPILER_START)$(COMPILER) $(COMPILER_OPTIONS) -c $(CPP_SRC_FILES)
	@echo "Invoking ARM Linker..."
	@$(COMPILER_START)$(COMPILER) -nostdlib -T $(LD_FILE) -Wl,-Map,$(MAP_FILE) -o $(ELF_FILE) $(CPP_OBJ_FILES)
	@echo "ELF file generated"

#Generating the hex file - DEP[elf file]
main.hex: main.elf
	@echo "Creating flash image..."
	@$(COMPILER_START)objcopy -O ihex $(ELF_FILE) $(HEX_FILE)
	@echo "Flash image generated"
	@echo "Printing program size"
	@$(COMPILER_START)size --format=gnu $(ELF_FILE)
	@echo "Build finished"

#Binaries cleaning rule - No DEP
clean:
	@rm -rf *.o *.hex *.elf *.map
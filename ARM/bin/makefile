#Author: Ahmed Ashraf
#Purpose: This makefile is used for ARM development purposes
#Version: v1.0
#Date: 19-06-2022

##################################### Development parameters ######################################

#CPP include directory
CPP_INC_DIR := -I../LIB

#CPP source files
CPP_SRC_FILES := ../main.cpp

#CPP object files
CPP_OBJ_FILES := main.o

#ELF file name
ELF_FILE := main.elf

#HEX file name
HEX_FILE := main.hex

#MAP file name
MAP_FILE := main.map

#Defining the CPU parameters
CPU := cortex-m3

#Defining compiler parameters
COMPILER := arm-none-eabi-g++
COMPILER_OPTIONS := -Wall -Os -mthumb -mcpu=$(CPU) $(CPP_INC_DIR)

############################################## Rules ##############################################

#Building binaries command - DEP[clean, elf file]
all: clean main.hex

#Generating the elf file
main.elf:
	@echo "Building target..."
	@echo "Invoking AVR Compiler..."
	@$(COMPILER) $(COMPILER_OPTIONS) -c $(CPP_SRC_FILES)
	@echo "Invoking AVR Linker..."
	@$(COMPILER) -Wl,-Map,$(MAP_FILE) -mmcu=$(MCU) -o $(ELF_FILE) $(CPP_OBJ_FILES)
	@echo "ELF file generated"

main.hex: main.elf
	@echo "Creating flash image..."
	@avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex $(ELF_FILE) $(HEX_FILE)
	@echo "Flash image generated"
	@echo "Printing program size"
	@avr-size --format=avr --mcu=atmega8 $(ELF_FILE)
	@echo "Build finished"

#Binaries cleaning rule - No DEP
clean:
	@rm -rf *.o *.hex *.elf *.map
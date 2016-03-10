CC = gcc
CC-AVR = avr-gcc
DEBUG = 0
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

# path to the LULU P colony simulator
LULU_PCOL_SIM_PATH = /home/andrei/script_Python/lulu_pcol_sim
# path to the LULU P colony simulator script
LULU_PCOL_SIM = $(LULU_PCOL_SIM_PATH)/sim.py
# path to the lulu to C converter script
LULU_C = /home/andrei/script_Python/lulu_c/lulu_c.py
# path to one example instance file (can be set as an Environment variable to any Lulu formatted input file)
LULU_INSTANCE_FILE = $(LULU_PCOL_SIM_PATH)/input_files/input_ag_decrement.txt
# path to the LULU headers
LULU_HEADERS = ../lulu/src
# path to the LULU C library
LULU_LIB = ../lulu/build/lulu.a
# path to the Kilombo headers
KILOMBO_HEADERS = /usr/include
# path to the Kilolib headers -- only needed for AVR
KILOLIB_HEADERS = ../kilolib
# path to the Kilolib library -- only needed for AVR
KILOLIB_LIB = $(KILOLIB_HEADERS)/build/kilolib.a
# path to the LULU C library for AVR
LULU_LIB_AVR = ../lulu/build_hex/lulu.a

ifeq ($(DEBUG),1)
  #debug & testing flags
  CFLAGS = -Wall -g -O0 -fbuiltin -c -DPCOL_SIM -DDEBUG_PRINT=0
  BFLAGS = -Wall -g -O0 -fbuiltin -DPCOL_SIM -DDEBUG_PRINT=0

  #compilation flags for simulated version
  SIM_CFLAGS = -c -g -O0 -Wall -std=c99
  #linking flags for simulated version
  SIM_LFLAGS = -lsim -lSDL -lm -ljansson
else
  #release flags
  CFLAGS = -Wall -g -O2 -fbuiltin -c -DPCOL_SIM -DDEBUG_PRINT=1
  BFLAGS = -Wall -g -O2 -fbuiltin -DPCOL_SIM -DDEBUG_PRINT=1

  #compilation flags for simulated version
  SIM_CFLAGS = -c -g -O2 -Wall -std=c99
  #linking flags for simulated version
  SIM_LFLAGS = -lsim -lSDL -lm -ljansson
endif

CFLAGS_AVR = -c -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -I$(KILOLIB_HEADERS) -DKILOBOT
BFLAGS_AVR = -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -I$(KILOLIB_HEADERS) -DKILOBOT
FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0


all: build/lulu_kilobot

clean:
	rm -vf build/*

clean_autogenerated_lulu:
	rm -vf src/instance.*

clean_hex:
	rm -vf build_hex/*

build/lulu_kilobot: build/lulu_kilobot.o build/instance.o
	$(CC) $(SIM_LFLAGS) $^ $(LULU_LIB) -o $@

build/lulu_kilobot.o: src/lulu_kilobot.c src/instance.h $(LULU_HEADERS)/rules.h
	$(CC) $(SIM_CFLAGS) -I$(LULU_HEADERS)/ src/lulu_kilobot.c -o $@

build/instance.o: src/instance.h src/instance.c $(LULU_HEADERS)/rules.h
	$(CC) $(CFLAGS) src/instance.c -I$(LULU_HEADERS)/ -o $@

src/instance.h src/instance.c:
	python $(LULU_C) $(LULU_INSTANCE_FILE) src/instance

# --------------------------------------------------------------------------------------------------------------------
# The following rules build for the kilobot

hex: build_hex/lulu_kilobot.hex

build_hex/lulu_kilobot.elf: build_hex/lulu_kilobot.o build_hex/instance.o
	$(CC-AVR) $(BFLAGS_AVR) $^ $(LULU_LIB_AVR) $(KILOLIB_LIB) -o $@

build_hex/lulu_kilobot.o: src/lulu_kilobot.c src/instance.h $(LULU_HEADERS)/rules.h
	$(CC-AVR) $(CFLAGS_AVR) -I$(LULU_HEADERS) -I$(KILOMBO_HEADERS)/ src/lulu_kilobot.c -o $@

build_hex/instance.o: src/instance.h src/instance.c $(LULU_HEADERS)/rules.h
	$(CC-AVR) $(CFLAGS_AVR) src/instance.c -I$(LULU_HEADERS)/ -o $@

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

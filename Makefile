CC = gcc
CC-AVR = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude
AVRSIZE = avr-size

DEBUG = 0
SPECIAL = 0
#IN_EXTEROCEPTIVE_CHECK_CRC = 1
IN_EXTEROCEPTIVE_CHECK_CRC = 0
OUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT = 1

# optimization flags for AVR
#AVR_OPTIM = -O3
AVR_OPTIM = -Os -mcall-prologues

# path to the LULU P colony simulator
LULU_PCOL_SIM_PATH = /home/andrei/script_Python/lulu_pcol_sim
# path to the LULU P colony simulator script
LULU_PCOL_SIM = $(LULU_PCOL_SIM_PATH)/sim.py
# path to the lulu to C converter script
LULU_C = /home/andrei/script_Python/lulu_c/lulu_c.py
# path to one example instance file (can be set as an Environment variable to any Lulu formatted input file)
#LULU_INSTANCE_FILE = test_secure_disperse.lulu pi_disperse 3 60
#LULU_INSTANCE_FILE = test_secure_disperse.lulu pi_disperse 10 0
#LULU_INSTANCE_FILE = test_disperse.lulu pi_disperse 3 60
#LULU_INSTANCE_FILE = input_files/test_disperse.lulu pi_disperse 2 0
LULU_INSTANCE_FILE = input_files/segregation/segregation.lulu pi_segregate 2 0
# path to the LULU headers
LULU_HEADERS = ../lulu/src
# path to the LULU C library
LULU_LIB = ../lulu/build/lulu.a

# --------------------------------------------------------------------------------------------------------------------
# The following path are for the kilobot

# path to the Kilombo headers
KILOMBO_HEADERS_AVR ="../kilombo/src"
# path to the Kilolib headers -- only needed for AVR
KILOLIB_HEADERS = ../kilolib
# path to the Kilolib library -- only needed for AVR
KILOLIB_LIB = $(KILOLIB_HEADERS)/build/kilolib.a

ifeq ($(DEBUG),1)
  #debug & testing flags
  CFLAGS = -Wall -g -O0 -fbuiltin -c -DPCOL_SIM -DDEBUG_PRINT=0
  BFLAGS = -Wall -g -O0 -fbuiltin -DPCOL_SIM -DDEBUG_PRINT=0

  #compilation flags for simulated version
  SIM_CFLAGS = -c -g -O0 -Wall -std=c99 -DPCOL_SIM -DDEBUG_PRINT=0
  #linking flags for simulated version
  SIM_LFLAGS = -lsim -lSDL -lm -ljansson -DPCOL_SIM -DDEBUG_PRINT=0

  # compilation flags for kilobot (AVR) with serial message printing
  CFLAGS_AVR = -c -mmcu=atmega328p -Wall -gdwarf-2 $(AVR_OPTIM) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -DDEBUG_PRINT=0 -I$(KILOLIB_HEADERS) -DKILOBOT -Wl,-u,vfprintf -lprintf_min
  # linking flags for kilobot (AVR) with serial message printing
  BFLAGS_AVR = -mmcu=atmega328p -Wall -gdwarf-2 $(AVR_OPTIM) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -DDEBUG_PRINT=0 -I$(KILOLIB_HEADERS) -DKILOBOT -Wl,-u,vfprintf -lprintf_min

  # path to the LULU C library for AVR (with DEBUG functions included)
  LULU_LIB_AVR = ../lulu/build_hex/lulu_debug.a
else
  #release flags
  CFLAGS = -Wall -g -O2 -fbuiltin -c -DPCOL_SIM
  BFLAGS = -Wall -g -O2 -fbuiltin -DPCOL_SIM

  #compilation flags for simulated version
  SIM_CFLAGS = -c -g -O2 -Wall -std=c99 -DPCOL_SIM
  #linking flags for simulated version
  SIM_LFLAGS = -lsim -lSDL -lm -ljansson -DPCOL_SIM

  # compilation flags for kilobot (AVR) WITHOUT serial message printing
  CFLAGS_AVR = -c -mmcu=atmega328p -Wall -gdwarf-2 $(AVR_OPTIM) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -I$(KILOLIB_HEADERS) -DKILOBOT
  # linking flags for kilobot (AVR) WITHOUT serial message printing
  BFLAGS_AVR = -mmcu=atmega328p -Wall -gdwarf-2 $(AVR_OPTIM) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=8000000 -I$(KILOLIB_HEADERS) -DKILOBOT

  # path to the LULU C library for AVR
  LULU_LIB_AVR = ../lulu/build_hex/lulu.a
endif

ifeq ($(ID_SECURITY),1)
  USING_ID_SECURITY=-DUSING_ID_SECURITY
endif

ifeq ($(SPECIAL),1)
  CFLAGS += -DREQUIRES_SPECIAL_BEHAVIOUR
  BFLAGS += -DREQUIRES_SPECIAL_BEHAVIOUR

  #compilation flags for simulated version
  SIM_CFLAGS += -DREQUIRES_SPECIAL_BEHAVIOUR
  #linking flags for simulated version
  SIM_LFLAGS += -DREQUIRES_SPECIAL_BEHAVIOUR

  CFLAGS_AVR += -DREQUIRES_SPECIAL_BEHAVIOUR
  BFLAGS_AVR += -DREQUIRES_SPECIAL_BEHAVIOUR
endif

ifeq ($(IN_EXTEROCEPTIVE_CHECK_CRC),1)
  CFLAGS += -DIN_EXTEROCEPTIVE_CHECK_CRC
  BFLAGS += -DIN_EXTEROCEPTIVE_CHECK_CRC

  #compilation flags for simulated version
  SIM_CFLAGS += -DIN_EXTEROCEPTIVE_CHECK_CRC
  #linking flags for simulated version
  SIM_LFLAGS += -DIN_EXTEROCEPTIVE_CHECK_CRC

  CFLAGS_AVR += -DIN_EXTEROCEPTIVE_CHECK_CRC
  BFLAGS_AVR += -DIN_EXTEROCEPTIVE_CHECK_CRC
endif

ifeq ($(OUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT),1)
  CFLAGS += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT
  BFLAGS += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT

  #compilation flags for simulated version
  SIM_CFLAGS += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT
  #linking flags for simulated version
  SIM_LFLAGS += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT

  CFLAGS_AVR += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT
  BFLAGS_AVR += -DOUT_EXTEROCEPTIVE_AUTO_RESET_OUTPUT
endif

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0


all: build/lulu_kilobot hex

clean: clean_sim clean_hex clean_autogenerated_lulu

clean_autogenerated_lulu:
	rm -vf src/instance.*

clean_hex:
	rm -vf build_hex/*

clean_sim:
	rm -vf build/*

ifeq ($(SPECIAL),1)
build/lulu_kilobot: build/lulu_kilobot.o build/instance.o build/special_behaviour.o
	$(CC) $(SIM_LFLAGS) $^ $(LULU_LIB) -o $@
else
build/lulu_kilobot: build/lulu_kilobot.o build/instance.o
	$(CC) $(SIM_LFLAGS) $^ $(LULU_LIB) -o $@
endif

build/lulu_kilobot.o: src/lulu_kilobot.c src/instance.h $(LULU_HEADERS)/rules.h
	$(CC) $(SIM_CFLAGS) $(USING_ID_SECURITY) -I$(LULU_HEADERS)/ src/lulu_kilobot.c -o $@

build/instance.o: src/instance.h src/instance.c $(LULU_HEADERS)/rules.h
	$(CC) $(CFLAGS) src/instance.c -I$(LULU_HEADERS)/ -o $@

src/instance.h src/instance.c:
	python $(LULU_C) $(LULU_INSTANCE_FILE) src/instance

build/special_behaviour.o: src/special_behaviour.h src/special_behaviour.c src/lulu_kilobot.h
	$(CC) $(CFLAGS) src/special_behaviour.c -I$(LULU_HEADERS)/ -o $@
# --------------------------------------------------------------------------------------------------------------------
# The following rules build for the kilobot

hex: build_hex/lulu_kilobot.hex

build_hex/lulu_kilobot.elf: build_hex/lulu_kilobot.o build_hex/instance.o build_hex/special_behaviour.o
	$(CC-AVR) $(BFLAGS_AVR) $^ $(LULU_LIB_AVR) $(KILOLIB_LIB) -o $@

build_hex/lulu_kilobot.o: src/lulu_kilobot.c src/instance.h $(LULU_HEADERS)/rules.h
	$(CC-AVR) $(CFLAGS_AVR) $(USING_ID_SECURITY) -I$(LULU_HEADERS) -I$(KILOMBO_HEADERS_AVR)/ src/lulu_kilobot.c -o $@

build_hex/instance.o: src/instance.h src/instance.c $(LULU_HEADERS)/rules.h
	$(CC-AVR) $(CFLAGS_AVR) src/instance.c -I$(LULU_HEADERS)/ -o $@

build_hex/special_behaviour.o: src/special_behaviour.h src/special_behaviour.c src/lulu_kilobot.h
	$(CC-AVR) $(CFLAGS_AVR) src/special_behaviour.c -I$(LULU_HEADERS)/ -I$(KILOMBO_HEADERS_AVR)/ -o $@

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

mem_usage: build_hex/lulu_kilobot.elf
	$(AVRSIZE) -C --mcu=atmega328p $^

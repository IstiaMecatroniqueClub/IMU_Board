# Parameters for compiling

TARGET = output

FOLDER_NAME = include

F_CPU = 16000000UL

SRC = $(wildcard $(FOLDER_NAME)/*.cpp *.cpp)
INC = -I $(FOLDER_NAME)/
OBJ = $(SRC:.cpp=.o)

# Default compiler and programmer
CC = avr-g++
AVRDUDE = avrdude
OBJCOPY=avr-objcopy
# Device name (compiler and programmer)
MCU = atmega32m1
AVRDUDE_MCU = m32m1
AVRDUDE_PROG = avrispmkii

# for the documentation
DOCDIR = ../documentation
DOCFILE = doxygen_configuration.txt

AVRDUDEFLAGS = -P usb
# Default compiler and linker flags
CFLAGS = -c -W -Wall -Werror -mmcu=$(MCU) -Os $(INC) -DF_CPU=$(F_CPU) -std=c++11
LDFLAGS =

all: hex upload clean

documentation: $(DOCDIR)/$(DOCFILE)
	rm -rf $(DOCDIR)/html
	doxygen $(DOCDIR)/doxygen_configuration.txt

hex: $(TARGET).hex

# Create object files
%.o : %.cpp
	$(CC) $(CFLAGS) $^ -o $@

$(TARGET).hex: $(OBJ)
	$(CC) $(LDFLAGS) -mmcu=$(MCU) $^ -o $@

clean:
	rm $(OBJ)
	rm $(TARGET).hex

# Upload hex file in the target
upload:
	$(AVRDUDE) -c $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(AVRDUDEFLAGS) -U flash:w:$(TARGET).hex

flash:
	$(AVRDUDE) -c $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(AVRDUDEFLAGS) -U lfuse:w:0xEE:m

# .PHONY => force the update
.PHONY: clean all upload documentation

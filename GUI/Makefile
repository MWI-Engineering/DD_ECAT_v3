# Makefile for FFB Wheel System

# Compiler
CC = gcc

# Compiler flags

CFLAGS = -Wall -Wextra -pthread 

# Linker flags
LDFLAGS = -lpthread -ljson-c -lm

# Source files
#   -I/home/mwi/DD_ECAT_v3: Add this line for HID interface
SRCS = main_integration.c udp_communication.c hid_interface.c -I/home/mwi/DD_ECAT_v3

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
TARGET = ffb_GUI

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

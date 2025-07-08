# Makefile for Raspberry Pi FFB Steering Wheel Project

# Compiler
CC = gcc

# Compiler flags
# -Wall: Enable all standard warnings
# -O2: Optimization level 2
# -std=c11: Use C11 standard
# -pthread: Link with POSIX threads library
# -I.: Add current directory to include paths
# -I/home/mwi/SOEM/install/include: Add SOEM include path
# -D_POSIX_C_SOURCE=200809L: Required for clock_gettime on some systems
CFLAGS = -Wall -O2 -std=c11 -pthread -I. -I/home/mwi/SOEM/install/include

# Linker flags
# -L/home/mwi/SOEM/install/lib: Add SOEM library path
# -lsoem: Link with SOEM library
# -lrt: Link with real-time library (for clock_gettime)
# -lpcap: SOEM often depends on libpcap for raw socket access
# -lusb-1.0: Example for libusb (if used for HID)
LDFLAGS = -L/home/mwi/SOEM/install/lib -lrt -lsoem -lpcap

# Source files
SRCS = main.c ffb_calculator.c hid_interface.c soem_interface.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
TARGET = FF_DD

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

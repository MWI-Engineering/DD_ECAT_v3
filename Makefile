# Makefile for Force Feedback Project on Raspberry Pi

# --- Compiler and Flags ---
CC = gcc
# CFLAGS:
#   -Wall: Enable all common warnings [cite: 1]
#   -Wextra: Enable extra warnings [cite: 1]
#   -g: Include debugging information [cite: 1]
#   -std=c11: Use C11 standard [cite: 1]
#   -O2: Optimization level 2 (you can adjust this, e.g., -O0 for no optimization during debugging) [cite: 1]
#   -I/home/mwi/SOEM/install/include/soem: Add this line for SOEM headers
CFLAGS = -Wall -Wextra -g -std=c11 -O2 -I/home/mwi/SOEM/install/include

# LDFLAGS: Linker flags - specify libraries [cite: 2]
#   -lrt: Real-time extensions library (for clock_gettime) [cite: 2]
#   -lpthread: POSIX threads library [cite: 2]
#   -lm: Math library [cite: 2]
#   -lsoem: SOEM EtherCAT library [cite: 2]
#   -L/home/mwi/SOEM/install/lib: Add this line for SOEM library path
LDFLAGS = -lrt -lpthread -lm -lsoem -L/home/mwi/SOEM/install/lib

# --- Project Files ---
TARGET = ffb_app
SRCS = main.c ffb_calculator.c hid_interface.c soem_interface.c
OBJS = $(SRCS:.c=.o) # Automatically generate .o file names from .c file names

# --- Rules ---

# Default rule: builds the target executable
all: $(TARGET)

# Rule to link the object files into the executable
$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)..."
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET) created." 

# Generic rule to compile .c files into .o files
# $<: the first prerequisite (e.g., main.c)
# $@: the target of the rule (e.g., main.o)
%.o: %.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule: removes all generated object files and the executable
clean:
	@echo "Cleaning up..."
	rm -f $(OBJS) $(TARGET)
	@echo "Clean complete." 
# Phony targets: tell make that these are not actual files 
.PHONY: all clean
# Makefile for Force Feedback Project on Raspberry Pi

# --- Compiler and Flags ---
CC = gcc
# CFLAGS:
#   -Wall: Enable all common warnings
#   -Wextra: Enable extra warnings
#   -g: Include debugging information
#   -std=c11: Use C11 standard
#   -O2: Optimization level 2 (you can adjust this, e.g., -O0 for no optimization during debugging)
#   -D_GNU_SOURCE: Define _GNU_SOURCE for specific GNU extensions (like `gettimeofday` on some systems)
#   -D_USE_MATH_DEFINES: Define _USE_MATH_DEFINES for M_PI on Windows compatibility (though Raspberry Pi is Linux)
CFLAGS = -Wall -Wextra -g -std=c11 -O2 -D_GNU_SOURCE -D_USE_MATH_DEFINES

# LDFLAGS: Linker flags - specify libraries
#   -lrt: Real-time extensions library (for clock_gettime)
#   -lpthread: POSIX threads library
#   -lm: Math library
#   -lethercat: SOEM EtherCAT library
#   -L/usr/local/lib: (Optional) If your SOEM library is in a non-standard path, add -L path/to/soem/lib
LDFLAGS = -lrt -lpthread -lm -lethercat

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

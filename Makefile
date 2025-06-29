# Project names
DRONE_TARGET = drone
CENTER_TARGET = center_control

# Compiler and flags
CC = gcc
CFLAGS = -Wall -O2 -g -std=c99
LDFLAGS = -lpthread
LDFLAGS = -lpthread -lm 

# Source and object directories
SRCDIR = .
OBJDIR = obj
BINDIR = .
INCDIR = .

# Source files for each program
DRONE_SRCS = $(filter-out $(SRCDIR)/center_control.c, $(wildcard $(SRCDIR)/*.c))
CENTER_SRCS = $(filter-out $(SRCDIR)/drone.c, $(wildcard $(SRCDIR)/*.c))

# Object files
DRONE_OBJS = $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(DRONE_SRCS))
CENTER_OBJS = $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(CENTER_SRCS))

# Include directories
INCLUDES = -I$(INCDIR)

# Build rules
all: $(BINDIR)/$(DRONE_TARGET) $(BINDIR)/$(CENTER_TARGET)

$(BINDIR)/$(DRONE_TARGET): $(DRONE_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BINDIR)/$(CENTER_TARGET): $(CENTER_OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	@echo "Compiled: $<"

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR) $(BINDIR)/$(DRONE_TARGET) $(BINDIR)/$(CENTER_TARGET)

.PHONY: all clean
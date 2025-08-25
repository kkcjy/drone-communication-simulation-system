CC = gcc

SR_CFLAGS  = -Wall -g -DSIMULATION_COMPILE -Wno-format -Wno-unused-variable
DSR_CFLAGS = -Wall -g -DSIMULATION_COMPILE -Wno-format -Wno-unused-variable

FRAME_SRC = frame.h
CENTER_SRC = center.c
DRONE_SRC = drone.c
SUPPORT_SRC = support.h

SR_SRC = adhocuwb_swarm_ranging.c support.c
DSR_SRC = adhocuwb_dynamic_swarm_ranging.c support.c

CENTER_OUT = center
DRONE_OUT = drone

all: $(CENTER_OUT) $(DRONE_OUT)

SWARM_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_SRC) | grep -v '^[[:space:]]*$$' | grep -q '^[[:space:]]*#define[[:space:]]*SWARM_RANGING_MODE[[:space:]]*$$' && echo 1)
DYNAMIC_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_SRC) | grep -v '^[[:space:]]*$$' | grep -q '^[[:space:]]*#define[[:space:]]*DYNAMIC_SWARM_RANGING_MODE[[:space:]]*$$' && echo 1)

ifeq ($(SWARM_MODE_DEFINED)$(DYNAMIC_MODE_DEFINED),11)
$(error "Error: Both modes are defined! Please define only one mode")
else ifneq ($(SWARM_MODE_DEFINED),)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC) -lm

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(SR_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC) -lm
else ifneq ($(DYNAMIC_MODE_DEFINED),)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(CENTER_SRC) -lm

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(DSR_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(DRONE_SRC) $(DSR_SRC) -lm
else
$(error "Error: No valid mode defined in $(FRAME_SRC)! Please define either DYNAMIC_SWARM_RANGING_MODE or SWARM_RANGING_MODE (not commented)")
endif

mode:
ifeq ($(SWARM_MODE_DEFINED),1)
	@echo "Current mode: SWARM_RANGING_MODE"
else ifeq ($(DYNAMIC_MODE_DEFINED),1)
	@echo "Current mode: DYNAMIC_SWARM_RANGING_MODE"
endif

clean:
	rm -f $(CENTER_OUT) $(DRONE_OUT)
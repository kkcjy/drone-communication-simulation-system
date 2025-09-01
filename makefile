CC = gcc

SR_CFLAGS  = -Wall -IAdHocUWB/Inc -g -DSIMULATION_COMPILE -Wno-format -Wno-unused-variable
DSR_CFLAGS = -Wall -IAdHocUWB/Inc -g -DSIMULATION_COMPILE -Wno-format -Wno-unused-variable

FRAME_INC = frame.h
CENTER_SRC = center.c
DRONE_SRC = drone.c
SUPPORT_INC = support.h
SUPPORT_SRC = support.c

SR_SRC = AdHocUWB/Src/adhocuwb_swarm_ranging.c
DSR_SRC = AdHocUWB/Src/adhocuwb_dynamic_swarm_ranging.c

CENTER_OUT = center
DRONE_OUT = drone

all: $(CENTER_OUT) $(DRONE_OUT)

IEEE_MODE_DEFINED   = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_INC) | grep -q '^[[:space:]]*#define[[:space:]]*IEEE_802_15_4Z[[:space:]]*$$' && echo 1 || echo 0)
SWARM_V1_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_INC) | grep -q '^[[:space:]]*#define[[:space:]]*SWARM_RANGING_V1[[:space:]]*$$' && echo 1 || echo 0)
SWARM_V2_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_INC) | grep -q '^[[:space:]]*#define[[:space:]]*SWARM_RANGING_V2[[:space:]]*$$' && echo 1 || echo 0)
DYNAMIC_MODE_DEFINED  = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_INC) | grep -q '^[[:space:]]*#define[[:space:]]*DYNAMIC_RANGING_MODE[[:space:]]*$$' && echo 1 || echo 0)
COMPENSATE_DYNAMIC_MODE_DEFINED  = $(shell grep -v '^[[:space:]]*//' $(SUPPORT_INC) | grep -q '^[[:space:]]*#define[[:space:]]*COMPENSATE_DYNAMIC_RANGING_MODE[[:space:]]*$$' && echo 1 || echo 0)

MODE_COUNT = $(shell expr $(IEEE_MODE_DEFINED) + $(SWARM_V1_MODE_DEFINED) + $(SWARM_V2_MODE_DEFINED) + $(DYNAMIC_MODE_DEFINED) + $(COMPENSATE_DYNAMIC_MODE_DEFINED))

ifeq ($(MODE_COUNT),0)
$(error "Error: No mode defined in $(SUPPORT_INC)! Please define one mode")
else ifneq ($(MODE_COUNT),1)
$(error "Error: Multiple modes defined! Please define only one mode")
endif

# IEEE
ifeq ($(IEEE_MODE_DEFINED),1)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_INC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC) $(SUPPORT_SRC) -lm
$(DRONE_OUT): $(DRONE_SRC) $(FRAME_INC) $(SR_SRC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC) $(SUPPORT_SRC) -lm
endif

# SWARM_V1
ifeq ($(SWARM_V1_MODE_DEFINED),1)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_INC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC) $(SUPPORT_SRC) -lm
$(DRONE_OUT): $(DRONE_SRC) $(FRAME_INC) $(SR_SRC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC) $(SUPPORT_SRC) -lm
endif

# SWARM_V2
ifeq ($(SWARM_V2_MODE_DEFINED),1)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_INC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC) $(SUPPORT_SRC) -lm
$(DRONE_OUT): $(DRONE_SRC) $(FRAME_INC) $(SR_SRC) $(SUPPORT_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC) $(SUPPORT_SRC) -lm
endif

# DYNAMIC
ifeq ($(DYNAMIC_MODE_DEFINED),1)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_INC) $(SUPPORT_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(CENTER_SRC) $(SUPPORT_SRC) -lm
$(DRONE_OUT): $(DRONE_SRC) $(FRAME_INC) $(DSR_SRC) $(SUPPORT_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(DRONE_SRC) $(DSR_SRC) $(SUPPORT_SRC) -lm
endif

# COMPENSATE_DYNAMIC
ifeq ($(COMPENSATE_DYNAMIC_MODE_DEFINED),1)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_INC) $(SUPPORT_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(CENTER_SRC) $(SUPPORT_SRC) -lm
$(DRONE_OUT): $(DRONE_SRC) $(FRAME_INC) $(DSR_SRC) $(SUPPORT_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(DRONE_SRC) $(DSR_SRC) $(SUPPORT_SRC) -lm
endif

mode:
ifeq ($(IEEE_MODE_DEFINED),1)
	@echo "Current mode: IEEE_802_15_4Z"
else ifeq ($(SWARM_V1_MODE_DEFINED),1)
	@echo "Current mode: SWARM_RANGING_V1"
else ifeq ($(SWARM_V2_MODE_DEFINED),1)
	@echo "Current mode: SWARM_RANGING_V2"
else ifeq ($(DYNAMIC_MODE_DEFINED),1)
	@echo "Current mode: DYNAMIC_RANGING_MODE"
else ifeq ($(COMPENSATE_DYNAMIC_MODE_DEFINED),1)
	@echo "Current mode: COMPENSATE_DYNAMIC_RANGING_MODE"
endif

clean:
	rm -f $(CENTER_OUT) $(DRONE_OUT)
CL65 ?= cl65
TARGET ?= atari
BUILD_DIR ?= build
CC65_HOME ?= /usr/share/cc65
CFLAGS ?= -O -I $(CC65_HOME)/include --asm-include-dir $(CC65_HOME)/asminc
LDFLAGS ?= -C $(CC65_HOME)/cfg/$(TARGET).cfg -L $(CC65_HOME)/lib

APP = fnaudio-test
SOURCES = main.c sio_call.s

.PHONY: all clean

all: $(BUILD_DIR)/$(APP).xex

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/$(APP).xex: $(SOURCES) | $(BUILD_DIR)
	$(CL65) -t $(TARGET) $(CFLAGS) $(LDFLAGS) -o $@ $(SOURCES)

clean:
	rm -rf $(BUILD_DIR)

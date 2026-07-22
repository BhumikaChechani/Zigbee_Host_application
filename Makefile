# ------------------------------------------------------------------
#  ZNP MT Host Controller (C) - Siren & Button Integration
#
#  Layout:
#    include/   public headers (.h)
#    src/       translation units (.c)
#    build/     object + dependency files (.o/.d)   [generated]
#    bin/       final executable                     [generated]
#
#  Run from this folder so devices.txt (registry persistence) resolves.
# ------------------------------------------------------------------
CC        = gcc
CFLAGS    = -Wall -Wextra -pthread -O2 -std=c11 -D_DEFAULT_SOURCE
CPPFLAGS  = -Iinclude
LDFLAGS   = -pthread

SRC_DIR   = src
BUILD_DIR = build
BIN_DIR   = bin

# Extract the firmware version from config.h
APP_VERSION := $(shell grep -oP '^\#define\s+APP_VERSION\s+"\K[^"]+' include/config.h)
TARGET    = $(BIN_DIR)/znp_host_c_v$(APP_VERSION)

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
DEPS = $(OBJS:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR) $(BIN_DIR):
	mkdir -p $@

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

.PHONY: all clean

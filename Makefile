# Build/upload the Shed Fan Controller sketch with arduino-cli.
#
#   make            - compile (default)
#   make verify      - same as compile; arduino-cli has no separate verify
#                      step - this is what the Arduino IDE's Verify button
#                      does under the hood (compile without upload)
#   make port        - print the auto-detected serial port
#   make upload      - compile, then upload to PORT (auto-detected if not
#                      set) and read the binary back to confirm it matches
#   make format       - reformat every .ino/.h file in place with clang-format
#                       (style in .clang-format)
#   make format-check - fail if any file isn't already clang-format-clean,
#                       without changing anything (for CI)
#   make clean       - remove build artifacts
#
# Override on the command line, e.g.:
#   make upload PORT=/dev/cu.usbserial-1420
#   make upload FQBN=arduino:avr:nano:cpu=atmega328   # new-bootloader Nano

SKETCH_DIR := .
BUILD_DIR  := build
SOURCES    := $(wildcard *.ino) $(wildcard *.h)

# Most Nano boards in the wild - genuine ones from before ~2018 and nearly
# all CH340-based clones - need the old-bootloader option below; a Nano with
# a newer bootloader will fail to upload against this FQBN and needs the
# override shown above.
FQBN ?= arduino:avr:nano:cpu=atmega328old

.PHONY: all compile verify upload port format format-check clean

all: compile

compile:
	arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKETCH_DIR)

verify: compile

# Auto-detect the board's serial port: prefer arduino-cli's own board
# recognition (matching_boards is only set for ports it can identify as a
# known board), else fall back to the usual USB-serial device naming
# patterns and take the first match. Override with `make PORT=... upload`
# if both miss (e.g. an unrecognized USB-serial chip).
PORT ?= $(shell arduino-cli board list --format json 2>/dev/null | python3 -c \
	"import json, sys; data = json.load(sys.stdin); \
	ports = [e['port']['address'] for e in data.get('detected_ports', []) if e.get('matching_boards')]; \
	print(ports[0] if ports else '')" 2>/dev/null)
ifeq ($(PORT),)
PORT := $(shell ls /dev/cu.usbserial-* /dev/cu.wchusbserial-* /dev/cu.usbmodem* /dev/cu.SLAB_USBtoUART* 2>/dev/null | head -1)
endif

port:
	@if [ -z "$(PORT)" ]; then \
		echo "No serial port found - is the Nano plugged in?"; \
		echo "Connected serial ports:"; \
		arduino-cli board list; \
		exit 1; \
	fi
	@echo $(PORT)

upload: compile port
	arduino-cli upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) --verify $(SKETCH_DIR)

# style lives in .clang-format
format:
	clang-format -i $(SOURCES)

format-check:
	clang-format --dry-run --Werror $(SOURCES)

clean:
	rm -rf $(BUILD_DIR)

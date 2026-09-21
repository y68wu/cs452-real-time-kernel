CC ?= gcc
CFLAGS ?= -Wall -Wextra -std=c11 -I src

SRC = src/main.c \
      src/clock.c \
      src/command.c \
      src/display.c \
      src/train.c \
      src/sensor.c

OBJ = $(SRC:.c=.o)

TARGET = build/a0.elf

.PHONY: all clean run

all: $(TARGET)

$(TARGET): build $(OBJ) ; $(CC) $(CFLAGS) -o $(TARGET) $(OBJ)

build: ; mkdir -p build

run: $(TARGET) ; ./$(TARGET)

clean: ; rm -f $(OBJ) $(TARGET); rmdir build 2>/dev/null || true

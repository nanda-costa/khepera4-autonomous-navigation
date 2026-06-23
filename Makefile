export LD_LIBRARY_PATH := /usr/local/webots/lib/controller:$(LD_LIBRARY_PATH)
CC = gcc
CFLAGS = -Wall -Iinclude

WEBOTS_HOME = /usr/local/webots

CFLAGS += -I"$(WEBOTS_HOME)/include/controller/c"
LFLAGS = -L"$(WEBOTS_HOME)/lib/controller" -lController -lm

TARGET = navegacao_khepera
SRCS = src/main.c src/actuation.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LFLAGS)

clean:
	rm -f src/*.o $(TARGET)
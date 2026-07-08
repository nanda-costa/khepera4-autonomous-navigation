CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Iinclude -I./include_khepera -DROBOT_FISICO -fno-stack-protector -U_FORTIFY_SOURCE
LDFLAGS = -L. -Wl,--allow-shlib-undefined -Wl,--wrap=memcpy -lm ./libkhepera.so

TARGET = navegacao_fisi_khepera
SRCS = src/main.c src/actuation.c src/perception.c src/decision.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

clean:
	rm -f src/*.o $(TARGET)
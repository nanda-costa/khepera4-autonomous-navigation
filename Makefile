CC = gcc
CFLAGS = -Wall -Iinclude
TARGET = navegacao_khepera

SRCS = src/main.c src/percepcao.c src/atuacao.c src/decisao.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) -lm

clean:
	rm -f src/*.o $(TARGET)
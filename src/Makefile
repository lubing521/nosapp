SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)
#CC = gcc
#tw
#CC=mips-linux-gcc
#nc mips
#CC=mipsel-unknown-linux-uclibc-gcc
#nc arm
#CC=arm-marvell-linux-uclibcgnueabi-gcc
INCLUDES = -I ./lib
LIBS = -L ./lib
#CCFLAGS = -g -Wall -O0
CCFLAGS = -I../../../include -I./ -Wall -O0
LDFLAGS = -L../../../lib -lnorouter -lpthread
#CFLAGS=-I../../include -I./
#LDFLAGS=-L../../lib -lnorouter cgicomm.o

ismartus : $(OBJS)
	$(CC) $^ -o $@ $(INCLUDES) $(LIBS) $(LDFLAGS)

%.o : %.c
	$(CC) -c $< $(CCFLAGS)
clean:
	rm -rf *.o
	rm -rf ismartus
.PHONY:clean



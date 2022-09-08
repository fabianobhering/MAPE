SRCS=$(wildcard *.c)
OBJS=$(SRCS:.c=.o)

CFLAGS=-O0 -g -Wall -DUSE_INT_WEIGHT -lm# -pg
#CFLAGS=-O2 -Wall -DUSE_INT_WEIGHT# -pg


FITPATH_OBJS=array.o \
		dijkstra.o \
		graph.o \
		heap.o \
		heuristics.o \
		list.o \
		mainHeuristicILS.o \
		parser.o \
		prefixTree.o \
		set.o \
		simulationh2.o \
		stack.o \
		stateh2.o \
		yen.o

MAPE_OBJS=array.o \
		dijkstra.o \
		graph.o \
		heap.o \
		heuristics.o \
		list.o \
		mainMAPE.o \
		parser.o \
		prefixTree.o \
		set.o \
		simulationh2.o \
		stack.o \
		stateh2.o \
		yen.o


all: pathGenerator MAPE heuristicILS_MAPE

pathGenerator: ${PATHGENERATOR_OBJS}
	${CC} ${PATHGENERATOR_OBJS} -o pathGenerator ${CFLAGS}

FITPATH: ${FITPATH_OBJS}
	${CC} ${FITPATH_OBJS} -o heuristicILS_MAPE ${CFLAGS}

MAPE: ${MAPE_OBJS}
	${CC} ${MAPE_OBJS} -o MAPE ${CFLAGS}

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f pathGenerator FITPATH MAPE
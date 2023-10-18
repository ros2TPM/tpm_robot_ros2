CC = gcc
CFLAGS = -fPIC -Wall -O3

INC = -I./ -I../../RobC/include
SRC = 	$(wildcard *.c)
OBJ = 	$(filter %.o,$(SRC:.cxx=.o) $(SRC:.c=.o))

libRPiMNet.so: $(OBJ)
		$(CC) -shared -lstdc++ -o $@ $(OBJ)

%.o: %.c
		$(CC) $< ${INC} -lstdc++ -c $(CFLAGS) -o $@

.PHONY: clean
clean:
	rm -f $(OBJ) *.so

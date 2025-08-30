CC = gcc

CFLAGS = -I/usr/include/libusb-1.0 -I../../../../../vendor/libdw3000/include -I../Inc -DSNIFFER_COMPILE


LDFLAGS = -lusb-1.0

SRC = sniffer.c
TARGET = sniffer

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)
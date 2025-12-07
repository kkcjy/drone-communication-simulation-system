CC = gcc

CFLAGS = -I/usr/include/libusb-1.0 -I../../../../../vendor/libdw3000/include -I../Inc -DSNIFFER_STORAGE_COMPILE


LDFLAGS = -lusb-1.0

SRC = sniffer_storage.c
TARGET = sniffer_storage

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)
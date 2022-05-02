CC = g++
CFLAGS = -I.
TARGET = testing
LIBS = -lEposCmd

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) -o $(TARGET) $(TARGET).cpp $(CFLAGS) $(LIBS)

clean:
	$(RM) $(TARGET)

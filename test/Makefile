# SIL verification build for the generated CruiseControl SWC.
#   make        -> build ./sil_test
#   make run    -> build and run
#   make clean

CC      := gcc
CFLAGS  := -Wall -Wextra -std=c99 -O2
INCS    := -I../CruiseControl_autosar_rtw \
           -I../CruiseControl_autosar_rtw/stub \
           -I../AUTOSAR_Include
SRCS    := sil_test.c ../CruiseControl_autosar_rtw/CruiseControl.c
TARGET  := sil_test

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCS) -o $@ $(SRCS) -lm

.PHONY: run clean
run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.o

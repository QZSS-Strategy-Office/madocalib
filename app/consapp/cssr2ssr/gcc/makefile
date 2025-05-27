# makefile for MADOCALIB cssr2ssr
# Rev.2025/04/16

SRC     = ../../../../src

CC      = gcc
OPTS    = -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=5 -DNEXOBS=5
CFLAGS  = -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -Wno-stringop-truncation -I$(SRC) $(OPTS) -g
LDLIBS  = -lm -lrt
OBJS    = cssr2ssr.o cssr.o rtkcmn.o rtcm.o rtcm2.o rtcm3.o rtcm3e.o

cssr2ssr.exe : $(OBJS)
	$(CC) -o cssr2ssr $(OBJS) $(LDLIBS) 
cssr2ssr.o : ../cssr2ssr.c
	$(CC) -c $(CFLAGS) ../cssr2ssr.c
cssr.o     : ../cssr.c
	$(CC) -c $(CFLAGS) ../cssr.c
rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
rtcm.o     : $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
rtcm3e.o   : $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c
cssr2ssr.o : $(SRC)/rtklib.h
cssr.o     : $(SRC)/rtklib.h
rtkcmn.o   : $(SRC)/rtklib.h
rtcm.o     : $(SRC)/rtklib.h
rtcm2.o    : $(SRC)/rtklib.h
rtcm3.o    : $(SRC)/rtklib.h
rtcm3e.o   : $(SRC)/rtklib.h


clean :
	rm -rf cssr2ssr *.o




# makefile for MADOCALIB rnx2rtkp.exe
# Rev.2025/04/16

BINDIR  = ..\..\..\..\bin
SRC     = ..\..\..\..\src

CC      = gcc
OPTS    = -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=5 -DNEXOBS=5 -DWIN32
CFLAGS  = -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -Wno-stringop-truncation -I$(SRC) $(OPTS) -g
LDLIBS  = -lwinmm
OBJS    = rnx2rtkp.o rtkcmn.o rinex.o rtkpos.o postpos.o solution.o lambda.o geoid.o sbas.o preceph.o pntpos.o ephemeris.o options.o ppp.o ppp_ar.o ppp_iono.o rtcm.o rtcm2.o rtcm3.o rtcm3e.o ionex.o tides.o mdccssr.o mdciono.o

rnx2rtkp.exe : $(OBJS)
	$(CC) -o rnx2rtkp.exe $(OBJS) $(LDLIBS) 
rnx2rtkp.o : ../rnx2rtkp.c
	$(CC) -c $(CFLAGS) ../rnx2rtkp.c
rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
rinex.o    : $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
rtkpos.o   : $(SRC)/rtkpos.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkpos.c
postpos.o  : $(SRC)/postpos.c
	$(CC) -c $(CFLAGS) $(SRC)/postpos.c
solution.o : $(SRC)/solution.c
	$(CC) -c $(CFLAGS) $(SRC)/solution.c
lambda.o   : $(SRC)/lambda.c
	$(CC) -c $(CFLAGS) $(SRC)/lambda.c
geoid.o    : $(SRC)/geoid.c
	$(CC) -c $(CFLAGS) $(SRC)/geoid.c
sbas.o     : $(SRC)/sbas.c
	$(CC) -c $(CFLAGS) $(SRC)/sbas.c
preceph.o  : $(SRC)/preceph.c
	$(CC) -c $(CFLAGS) $(SRC)/preceph.c
pntpos.o   : $(SRC)/pntpos.c
	$(CC) -c $(CFLAGS) $(SRC)/pntpos.c
ephemeris.o: $(SRC)/ephemeris.c
	$(CC) -c $(CFLAGS) $(SRC)/ephemeris.c
options.o  : $(SRC)/options.c
	$(CC) -c $(CFLAGS) $(SRC)/options.c
ppp.o      : $(SRC)/ppp.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp.c
ppp_ar.o   : $(SRC)/ppp_ar.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_ar.c
ppp_iono.o : $(SRC)/ppp_iono.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_iono.c
rtcm.o     : $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
rtcm3e.o   : $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c
ionex.o    : $(SRC)/ionex.c
	$(CC) -c $(CFLAGS) $(SRC)/ionex.c
tides.o    : $(SRC)/tides.c
	$(CC) -c $(CFLAGS) $(SRC)/tides.c
mdccssr.o  : $(SRC)/mdccssr.c
	$(CC) -c $(CFLAGS) $(SRC)/mdccssr.c
mdciono.o  : $(SRC)/mdciono.c
	$(CC) -c $(CFLAGS) $(SRC)/mdciono.c
rnx2rtkp.o : $(SRC)/rtklib.h
rtkcmn.o   : $(SRC)/rtklib.h
rinex.o    : $(SRC)/rtklib.h
rtkpos.o   : $(SRC)/rtklib.h
postpos.o  : $(SRC)/rtklib.h
solution.o : $(SRC)/rtklib.h
lambda.o   : $(SRC)/rtklib.h
geoid.o    : $(SRC)/rtklib.h
sbas.o     : $(SRC)/rtklib.h
preceph.o  : $(SRC)/rtklib.h
pntpos.o   : $(SRC)/rtklib.h
ephemeris.o: $(SRC)/rtklib.h
options.o  : $(SRC)/rtklib.h
ppp.o      : $(SRC)/rtklib.h
ppp_ar.o   : $(SRC)/rtklib.h
ppp_iono.o : $(SRC)/rtklib.h
rtcm.o     : $(SRC)/rtklib.h
rtcm2.o    : $(SRC)/rtklib.h
rtcm3.o    : $(SRC)/rtklib.h
rtcm3e.o   : $(SRC)/rtklib.h
ionex.o    : $(SRC)/rtklib.h
tides.o    : $(SRC)/rtklib.h
mdccssr.o  : $(SRC)/rtklib.h
mdciono.o  : $(SRC)/rtklib.h

install :
	copy rnx2rtkp.exe $(BINDIR)

clean :
	del *.o
	del rnx2rtkp.exe




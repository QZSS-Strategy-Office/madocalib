@echo off
rem : 
rem : This is a sample BAT file that executes ppp-ar from 00min00sec to 59min30sec every hour
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\rnx2rtkp.exe
set CONF=..\app\consapp\rnx2rtkp\gcc_mingw\sample_pppar.conf
set OBS=.\data\rinex\obs\%%Y\%%n\MIZU00JPN_R_%%Y%%n0000_01D_30S_MO_BDSTEST.rnx
set NAV=.\data\rinex\BRDC\%%Y\%%n\BRDC00IGS_R_%%Y%%n0000_01D_MN_BDSTEST.rnx
set L6E1=.\data\l6\%%Y\%%n\%%Y%%n%%HU.207.l6
set L6D1=""
set L6D2=""
set ANT=.\data\igs20_2290.atx
set OUT=.\result_exec_rnx2rtkp

rem Start date and time
set TS_DATE=2026/03/28
set TS_TIME=00:00:00

rem End date and time
set TE_DATE=2026/03/28
set TE_TIME=23:59:30

mkdir %OUT%

%BIN% -ts %TS_DATE% %TS_TIME% -te %TE_DATE% %TE_TIME% -ti 30 -k %CONF% -o %OUT%\%%Y%%m%%d%%h%%M%%S.pos -ant %ANT% %OBS% %NAV% %L6E1%

echo Output files (*.pos) were stored in the following folder
echo %OUT%
echo.
echo The output files can be visualized by rtkplot.exe in the same folder.
echo  - Double-click rtkplot.exe to start it.
echo  - Drag and drop selected *.pos files into rtkplot panel.
echo.

pause

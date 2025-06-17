@echo off
rem : 
rem : This is a sample BAT file that executes ppp from 00min00sec to 59min30sec every hour
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\rnx2rtkp.exe
set CONF=..\app\consapp\rnx2rtkp\gcc_mingw\sample.conf
set OBS=.\data\rinex\MIZU00JPN_R_%%Y%%n0000_01D_30S_MO.rnx
set NAV=.\data\rinex\BRDM00DLR_S_%%Y%%n0000_01D_MN.rnx
set L6E1=.\data\l6_is-qzss-mdc-004\%%Y\%%n\%%Y%%n%%HU.204.l6
set L6E2=.\data\l6_is-qzss-mdc-004\%%Y\%%n\%%Y%%n%%HU.206.l6
set L6D1=""
set L6D2=""
set ANT=.\data\igs20.atx
set OUT=.\result_exec_ppp

mkdir %OUT%

echo.
echo This is a scenario of PPP by MADOCA-PPP
echo Input RINEX OBS and NAV are at
echo %OBS%
echo %NAV%
echo.
echo Input L6 messages are in
echo .\data\l6_is-qzss-mdc-004
echo.

for /l %%X in (0,1,23) do (
    
    %BIN% -ts 2025/04/01 %%X:00:00 -te 2025/04/01 %%X:59:30 -ti 30 -k %CONF% -o %OUT%\%%Y%%m%%d%%h%%M%%S.pos -ant %ANT% %OBS% %NAV% %L6E1% %L6E2%
    
)
echo Output files (*.pos) were stored in the following folder
echo %OUT%
echo.
echo The output files can be visualized by rtkplot.exe in the same folder.
echo  - Double-click rtkplot.exe to start it.
echo  - Drag and drop selected *.pos files into rtkplot panel.
echo.
pause

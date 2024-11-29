@echo off
rem : 
rem : This is a sample BAT file that executes ppp from 00min00sec to 59min30sec every hour
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\rnx2rtkp.exe
set CONF=..\app\consapp\rnx2rtkp\gcc_mingw\sample.conf
set OBS=.\TSK200JPN_S_20241620000_01D_30S_MO.rnx
set NAV=.\TSK200JPN_S_20241620000_01D_MN.rnx
set L6E=.\2024162all.204.l6
set ANT=.\igs20.atx
set OUT=.\result

mkdir %OUT%

for /l %%H in (0,1,23) do (
    if %%H lss 10 (
        set "HH=0%%H"
    ) else (
        set "HH=%%H"
    )
    
    %BIN% -k %CONF% -ts 2024/06/10 !HH!:00:00 -te 2024/06/10 !HH!:59:30 %OBS% %NAV% %L6E% -o %OUT%\ppp_20240610!HH!.pos -ant %ANT% -x 2
    
)
pause

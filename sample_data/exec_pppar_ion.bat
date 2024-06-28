@echo off
rem : 
rem : This is a sample BAT file that executes ppp-ar with ionospheric correction 
rem : from 00min00sec to 59min30sec every hour
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\rnx2rtkp.exe
set CONF=..\app\consapp\rnx2rtkp\gcc_mingw\sample_pppar_iono.conf
set OBS=.\TSK200JPN_S_20241620000_01D_30S_MO.rnx
set NAV=.\TSK200JPN_S_20241620000_01D_MN.rnx
set L6E=.\2024162all.204.l6
set L6D=.\2024162all.201.l6 rem for Japan and Eastern Australia Region
rem set L6D=.\2024162all.200.l6 rem for Southeast Asia and Western Australia Region
set OUT=.\result

mkdir %OUT%

for /l %%H in (0,1,23) do (
    if %%H lss 10 (
        set "HH=0%%H"
    ) else (
        set "HH=%%H"
    )
    
    %BIN% -k %CONF% -ts 2024/06/10 !HH!:00:00 -te 2024/06/10 !HH!:59:30 %OBS% %NAV% %L6E% -mdciono %L6D% -o %OUT%\pppar_ion_20240610!HH!.pos -x 2
    
)
pause

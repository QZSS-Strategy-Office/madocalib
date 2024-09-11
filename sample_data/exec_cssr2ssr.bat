@echo off
rem : 
rem : This is a sample BAT file that executes cssr2ssr
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\cssr2ssr.exe
set L6E=.\2024162all.204.l6


set OUT=.\result_rtcm3

set RTCM3=2024162all.204.rtcm3

mkdir %OUT%


%BIN% %L6E% -o %OUT%\%RTCM3% -td 2024/06/10


pause

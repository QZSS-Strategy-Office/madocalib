@echo off
rem : 
rem : This is a sample BAT file that executes cssr2ssr
rem : 

setlocal enabledelayedexpansion

set BIN=..\bin\cssr2ssr.exe

set L6E=.\data\l6_is-qzss-mdc-005\2026\087\2026087A.207.l6
set OUT=.\result_exec_cssr2ssr
set RTCM3=2026087A.207.rtcm3
set TXT=2026087A.207.l6.txt

mkdir %OUT%

%BIN% %L6E% -o %OUT%\%RTCM3% -td 2026/03/28 -bds2prn C09,C10 -d > %OUT%\%TXT%

echo Output files were stored in the following folder
echo %OUT%

pause

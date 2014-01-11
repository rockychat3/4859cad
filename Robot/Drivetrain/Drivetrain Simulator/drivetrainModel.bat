@echo off

echo running...

set filename=10.71_6in.CSV
set xstop=10
set tstop=100

set Kro=10
set Krv=0.0
set Kf=0.7

set r=6
set G=12.75

set M=120.0
set us=.7
set uk=.6

set Rcom=0.013
set Rone=0.002
set Vbat=12.7

set Vspec=12
set Tspec=343.4
set Wspec=5310
set Ispec=133
set n=4

drivetrainModel.exe > %filename%

echo .
echo .
echo output file %filename% created.
echo .
echo .

pause
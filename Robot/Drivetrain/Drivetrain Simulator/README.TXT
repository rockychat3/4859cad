***IF YOU DOWNLOADED THIS FROM CHIEF DELPHI, PLEASE CHECK THE BOTTOM OF THE LIST OF ATTACHMENTS FOR AN UPDATED VERSION***    12/18/2013 (c) Ether


TO INSTALL:

This is a "portable" app.  

No installation required.  

Nothing is added to or modified in the Registry.

Just unzip the files to a folder.


TO RUN:

1) Open the folder containing "drivetrainModel.bat" and "drivetrainModel.exe"

2) Open the file "drivetrainModel.bat" with any text editor,
   modify the parameters as desired (see below), and save the file.
   You may leave the text editor open if you plan to try other changes.

2) Double-click on "drivetrainModel.bat" file.
   It sets the parameters in the environment and then runs the executable,
   which creates an output file in CSV format.

3) You can open the CSV file in Excel by simply double-clicking on it,
   if you have set the file associations to tell Windows to use Excel for CSV files.

4) Once the CSV file is open in Excel, you can easily plot a graph.
   Save the file as XLS if you want to save the graph you created.




EXAMPLE PARAMETERS (user changes these as desired):

set filename=2013-12-18_922p.CSV  // type the name you want for the output file
set xstop=15  // terminate program if distance in feet exceeds this value
set tstop=100  // terminate the program if elapsed time in seconds exceeds this value

set Kro=10  // non-speed-dependent non-torque-dependent constant friction force in pounds
set Krv=0.0  // speed-dependent losses, pounds per ft/sec
set Kf=0.7  // drivetrain torque efficiency fraction, unitless, range 0 to 1

set r=3   // wheel radius, inches
set G=12.75  // total gear ratio (speed reduction) from motor to wheel (including sprockets or pulleys)

set M=150.0  // total mass of robot, lbm
set us=1.0   // static coefficient of friction of wheels
set uk=0.8   // dynamic coefficient of friction of wheels

set Rcom=0.013  // circuit resistance from battery to PDB, including battery internal resistance, 120amp breaker,
                // and all wires (both directions) and connections

set Rone=0.002  // circuit resistance from PDB to motor, including 40amp breaker and all wires and connections
                // in both directions (but *not* including motor resistance)

set Vbat=12.7   // fully charged battery voltage with surface charge removed (~12.7 volts for lead-acid)

set Vspec=12    // motor specification voltage
set Tspec=343.4  // motor stall torque, in_oz
set Wspec=5310  // motor from RPM
set Ispec=133   // motor stall amps
set n=4         // number of motors


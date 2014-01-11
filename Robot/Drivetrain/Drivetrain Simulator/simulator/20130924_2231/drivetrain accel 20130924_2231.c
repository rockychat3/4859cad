#define build "20130924_2231"

#include <math.h>
#include <stdio.h>

// The following constants define the drivetrain being modeled:

// note: k1, k2, & k3 have been renamed Kro, Krv, & Kf
double Kro = 10;  // rolling resistance tuning parameter, lbf
double Krv = 0; // rolling resistance tuning parameter, lbf/(ft/sec)
double Kf = 0.9;  // drivetrain efficiency fraction

double Vspec = 12;   // motor spec volts
double Tspec = 343.4;   // motor stall torque, in_oz
double Wspec = 5310; // motor free speed, RPM
double Ispec = 133;  // motor stall amps
int n=4;             // number of motors

double G = 12.75;  // gear ratio
double r = 3;  // wheel radius, inches

double M  = 150;  // vehicle mass, lbm
double uk = 0.7;  // coefficient of kinetic friction
double us = 1.0;  // coefficient of static friction

double Rcom = 0.013; // ohms, battery internal resistance plus
                     // wire and connection resistance
					 // from battery to PDB (including main breaker)
					 
double Vbat = 12.7;  // fully-charged open-circuit battery volts

double Rone = 0.002;  // ohms, circuit wiring and connector resistance
                      // from PDB to motor (including 40A breaker)
				  
#define dt 0.1  // integration step size, seconds
#define tstop 1.0 // integration duration, seconds

// -------------- end of user-defined constants -----------------

double  // derived constants:
Toffset, Tslope,  // offset and slope of adjusted Torque vs Speed motor curve
Kt,  // motor torque constant, Nm/amp
Vfree,  // vehicle speed at motor free speed, meters/sec
W,  // vehicle weight, Newtons
F2A; // force to amps

// working variables:

int slipping = 0;  // state variable, init to zero
double Vm;     // Voltage at the motor
double V = 0;  // vehicle speed, meters/sec
double x = 0;  // vehicle distance traveled, meters
double t,  // elapsed time, seconds
       a,  // vehicle acceleration, meters/sec/sec
	   A;  // current per motor, amps

void English2SI(void){
Kro *= 4.448222;  // convert lbf to Newtons
Krv *= 4.448222*3.28083; // convert lbf/(ft/s) to Newtons/(meter/sec)
Tspec *= 0.00706155;  // convert oz_in to Newton_meters
Wspec = Wspec/60*2*3.1415926536; // convert RPM to rad/sec
r = r*2.54/100;  // convert inches to meters
M *= 0.4535924;  // convert lbm to kg
}


double accel(double V){  // compute acceleration w/ slip
double 
Wm, // motor speed associated with vehicle speed
L,  // rolling resistance losses, Newtons
Tm, // motor torque, Newtons
Tw, // wheel torque, Newtons
Ft, // available vehicle force due to wheel torque, Newtons
F,  // slip-adjusted vehicle force due to wheel torque, Newtons
Fa; // vehicle accel force, Newtons
Wm = V/r*G;
Tm = Toffset-Tslope*Wm; // available torque at motor @ V
Tw = Kf*Tm*G; // available torque at one wheel @ V
Ft = Tw/r*n;  // available force at wheels @ V
if (Ft>W*us) slipping=1;
else if (Ft<W*uk) slipping=0;
F = slipping? W*uk : Ft;
A = F*F2A;    // computed here for output
Vm = Vbat-n*A*Rcom-A*Rone;  // computed here for output
L = Kro+Krv*V; // rolling resistance force
Fa = F-L; // net force available for acceleration
if (Fa<0) Fa=0;
return Fa/M;
}

void print(void){
printf("%f,%f,%f,%d,%f,%f,%f\n",t,x*3.28083,V*3.28083,slipping,a*3.28083,n*A/10,Vm);
}

void Heun(void){ // numerical integration using Heun's Method
double Vtmp, atmp; // local scratch variables
for (t=dt; t<=tstop; t+=dt) {
Vtmp = V+a*dt;  // kickstart with Euler step
atmp = accel(Vtmp);
Vtmp = V+(a+atmp)/2*dt; // recalc Vtmp trapezoidally
a = accel(Vtmp);  // update a
x+=(V+Vtmp)/2*dt;  // update x trapezoidally
V=Vtmp;            // update V
print();}
}

// for reference only; not used:
void Euler(void){ // numerical integration using Euler's Method
for (t=dt; t<=tstop; t+=dt) {
V+=a*dt;
x+=V*dt;
a = accel(V); 
print();}
}


void main(void){

printf("t,feet,ft/s,slip,ft/s/s,amps/10,Vm,%s Kro=%4.1f Krv=%4.1f Kf=%3.1f Vspec=%3.1f Tspec=%4.1f Wspec=%5.0f Ispec=%4.1f Rcom=%4.3f Vbat=%4.2f Rone=%4.3f n=%d G=%5.2f r=%3.1f M=%3.0f uk=%3.2f us=%3.2f\n",
build,Kro,Krv,Kf,Vspec,Tspec,Wspec,Ispec,Rcom,Vbat,Rone,n,G,r,M,uk,us); // print CSV header

English2SI();

// calculate Derived Constants once:
Toffset = (Tspec*Vbat*Wspec)/(Vspec*Wspec+Ispec*Rone*Wspec+Ispec*n*Rcom*Wspec);
Tslope = (Tspec*Vspec)/(Vspec*Wspec+Ispec*Rone*Wspec+Ispec*n*Rcom*Wspec);
Kt = Tspec/Ispec;
W = M*9.80665;
F2A = r/(n*Kf*G*Kt); // vehicle total force to per-motor amps conversion

a=accel(V); // compute accel at t=0
print();    // output values at t=0

Heun();  // numerically integrate and output using Heun's method

}

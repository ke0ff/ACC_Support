// OpenSCAD 2019.05
// FF-CAD GEN-II Support HW, REV-001
// Joe Haas, KE0FF, 10/06/2026
// Standoffs for the FF-CAD ACC installation.
//
// Rev-001, 10/06/2026
//	initial code

//----------------------------------------------------------------------------------------------------------------------
// User defined parameters.  Modify these to suit a particular application
// NOTE: All data in this file is in mm
//----------------------------------------------------------------------------------------------------------------------
// parametric variables:

i2m = 25.4;
m2m = i2m/1000;


// NOTES ///////////////////
// stoff() and off_stoff() are the RC-96 standoffs.  Set stoff85 = 0;
//
// off_stoff85(), and cap85() are the RC-85 standoffs using two machine-pin standoff sockets at U4
// stoff() with stoff85 = .44*i2m is the RC-85 standoff
//
///////////////////////////////////////////////////////////

// RC-96:
//stoff85 = 0;	    // uncomment for all RC96
//stoff();
//off_stoff();

// RC-85 standoff delta (2 machine-pin sockets):
stoff85 = 0.66*i2m; // uncomment for all RC85
//stoff();
//off_stoff85();
cap85();

///////////////////////////////////////////////////////////
sdia = 0.375*i2m;
sdia2 = 0.4*i2m;

tdia4 = (0.07*i2m)+.4;
cham = .6;
halfht = (0.5*i2m)-(cham*2) + stoff85;

dia4 = (0.125*i2m)+.4;
oox = 0.29*i2m;
ooy = 0.29*i2m;
ooz = 0.13*i2m;
oht = (0.5*i2m);
pthk = 0.063*i2m;
chame = 1.5;

// 0.5" standoff
module stoff(){

	difference(){
		union(){
			if(stoff85 == 0){
				translate([0,0,cham]) cylinder(r=sdia/2, h=halfht, $fn=32);
			}else{
				translate([0,0,cham]) cylinder(r=sdia2/2, h=halfht, $fn=32);
			}
			if(stoff85 == 0){
				cylinder(r2=sdia/2, r1=((sdia)/2)-cham, h=cham, $fn=32);
				translate([0,0,halfht+cham]) cylinder(r1=sdia/2, r2=((sdia)/2)-cham, h=cham, $fn=32);
			}else{
				cylinder(r2=sdia2/2, r1=((sdia2)/2)-cham, h=cham, $fn=32);
				translate([0,0,halfht+cham]) cylinder(r1=sdia2/2, r2=((sdia2)/2)-cham, h=cham, $fn=32);
			}
		}
		cylinder(r=tdia4/2, h=3.5*i2m, $fn=32, center=true);
		translate([0,0,halfht+(2*cham)-(tdia4)+.2]) cylinder(r2=(2*tdia4/2), r1=0, h=tdia4, $fn=16);
		if(stoff85 != 0){
			translate([0,0,-.2]) cylinder(r1=(2*tdia4/2), r2=0, h=tdia4, $fn=16);
		}
	}
}

//cap85();

module cap85(){
dx = 1;
	translate([-oox/2,0,-2*ooz]) difference(){
		// main body
		translate([dx,0,ooz]) cube([2*oox,ooy,2*ooz], center=true);
		// #4 thru
		translate([(oox/2)+0.01,0,0]) cylinder(r=dia4/2, h=1*i2m, $fn=32, center=true);
		// PCB "notch-out"
		translate([(-oox/2)-(dia4/2),0,(3*ooz)-pthk+(0.001*i2m)]) cube([2*oox,1.1*ooy,2*ooz], center=true);
		// edge chams
		translate([oox+dx,ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([-(oox+dx),ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([(oox+dx),-ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([-(oox+dx),-ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([(oox+dx),0,0]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([-(oox-dx),0,0]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([dx,ooy/2,0]) rotate([0,90,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([dx,-ooy/2,0]) rotate([0,90,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
	}
}
//off_stoff();
//cylinder(r=.5,h=.5*i2m);

module off_stoff(){

	difference(){
		union(){
			// mtg tab
			translate([0,0,ooz/2]) cube([oox,ooy,ooz], center=true);
			// leg
			translate([(.75*oox)-.01,0,(oht-pthk)/2]) cube([.5*oox,ooy,oht+pthk], center=true);
		}
		cylinder(r=dia4/2, h=1.5*i2m, $fn=32, center=true);
//		translate([(oox/4)+(oox/2)-.1,0,((oht-ooz)/2)+ooz]) cube([(oox+.02)/2,ooy+.01,oht-ooz+.01], center=true);
		// edge chams
		translate([oox,ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([oox,-ooy/2,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([oox,-ooy/2,-pthk]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
		translate([oox,-ooy/2,oht]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht, $fn=4, center=true);
	}
}

//off_stoff85();
//cylinder(r=.5,h=.5*i2m);

module off_stoff85(){
oht85 = oht + stoff85;

	difference(){
		union(){
			translate([0,0,ooz/2]) cube([oox,ooy,ooz], center=true);
			translate([oox-.01,0,(oht85)/2]) cube([oox,ooy,oht85], center=true);
		}
		cylinder(r=dia4/2, h=1.5*i2m, $fn=32, center=true);
		translate([(oox/4)+(oox/2)-.1,0,((oht85-ooz)/2)+ooz]) cube([(oox+.02)/2,ooy+.01,oht85-ooz+.01], center=true);
		// edge chams
		translate([(oox/2)+oox,ooy/2,0]) cylinder(r=chame/2, h=3*oht85, $fn=4, center=true);
		translate([(oox/2)+oox,-ooy/2,0]) cylinder(r=chame/2, h=3*oht85, $fn=4, center=true);
		translate([(oox/2)+oox,-ooy/2,-pthk]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht85, $fn=4, center=true);
		translate([(oox/2)+oox,-ooy/2,oht85]) rotate([90,0,0]) cylinder(r=chame/2, h=3*oht85, $fn=4, center=true);
	}
}

///////////////////////////////////////////////////////////
// 16-DIP WW-socket depth-cut jig

//dip_cut();

module dip_cut(){
cham = 1;
chamc = 2.5;
oax = .6*i2m; //.8*i2m;
oay = 1.1*i2m;
oaz = 0.22 * i2m; //(.12*i2m)+(62*m2m);

	difference(){
		cube([oax,oay,oaz], center=true);
		for(y=[0:.1*i2m:.7*i2m]){
			translate([-150*m2m,y-(350*m2m),0]) dip_hole(dz=oaz);
			translate([150*m2m,y-(350*m2m),0]) dip_hole(dz=oaz);
		}
		// chams
		translate([-oax/2,-oay/2,0]) cylinder(r=chamc/2, h=10, $fn=4, center=true);
		translate([-oax/2,oay/2,0]) cylinder(r=chamc/2, h=10, $fn=4, center=true);
		translate([oax/2,-oay/2,0]) cylinder(r=chamc/2, h=10, $fn=4, center=true);
		translate([oax/2,oay/2,0]) cylinder(r=chamc/2, h=10, $fn=4, center=true);

		translate([0,oay/2,oaz/2]) rotate([0,90,0]) cylinder(r=chamc/2, h=30, $fn=4, center=true);
		translate([0,-oay/2,oaz/2]) rotate([0,90,0]) cylinder(r=chamc/2, h=30, $fn=4, center=true);
		translate([oax/2,0,oaz/2]) rotate([90,0,0]) cylinder(r=chamc/2, h=30, $fn=4, center=true);
		translate([-oax/2,0,oaz/2]) rotate([90,0,0]) cylinder(r=chamc/2, h=30, $fn=4, center=true);

		translate([0,oay/2,-oaz/2]) rotate([0,90,0]) cylinder(r=cham/2, h=30, $fn=4, center=true);
		translate([0,-oay/2,-oaz/2]) rotate([0,90,0]) cylinder(r=cham/2, h=30, $fn=4, center=true);
		translate([oax/2,0,-oaz/2]) rotate([90,0,0]) cylinder(r=cham/2, h=30, $fn=4, center=true);
		translate([-oax/2,0,-oaz/2]) rotate([90,0,0]) cylinder(r=cham/2, h=30, $fn=4, center=true);
	}
}

module dip_hole(dz=1){
//hdia = .055*i2m;		// 55mils is cut version, 51 mils is header shipping shroud
hdia = .051*i2m;
csd = .08*i2m;

	cylinder(r=hdia/2, h=10, $fn=16, center=true);
	translate([0,0,(dz/2)+.01-(csd/2)]) cylinder(r2=csd/2, r1=0, h=csd/2, $fn=16);
	translate([0,0,-(dz/2)-.01]) cylinder(r1=csd/2, r2=0, h=csd/2, $fn=16);
}


/////////////////////////
// debug artifacts
//
// X-rulers
//#translate([-.69,0,0]) cube([0.01,30,50]);	// ruler
//#translate([175.41,0,0]) cube([0.01,30,50]);	// inside main void ruler 1
//#translate([1.67,0,0]) cube([0.01,30,50]);	// inside main void ruler 2
//#translate([175.34,0,0]) cube([0.01,30,50]);	// outside shroud ruler 2
// Y-rulers
//#translate([0,23.81,0]) cube([180,0.01,50]);	// ruler
//#translate([0,22.36 ,0]) cube([180,0.01,50]);	// ruler
// Z-rulers
//#translate([0,0,0]) cube([10,40,.01]);	// ruler
//#translate([0,0,2.04]) cube([9,40,.01]);	// ruler

// EOF

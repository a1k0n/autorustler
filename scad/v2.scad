include <traxxas.scad>

// All coordinates are relative to the lower-left corner of the RPi2 viewed
// from the top with the ethernet/USB on the right side.
RPi_mounting_holes = [
  [3.5, 3.5, 0],
  [3.5, 3.5 + 49, 0],
  [3.5 + 58, 3.5, 0],
  [3.5 + 58, 3.5 + 49, 0],
];

RPicam_mounting_holes = [
  [9.35, 2, 0],
  [21.85, 2, 0],
  [9.35, 23, 0],
  [21.85, 23, 0]
];

module rpi2() {
  color([0.0, 0.5, 0.2]) translate([0, -2.5, -2.0]) {
    import("rpi-bplus.stl");
  }
}

module rpicam() {
  difference() {
    color("green") cube([23.9, 25, 0.95]);
    for (h = RPicam_mounting_holes) {
      translate(h + [0, 0, -0.1]) cylinder(d = 2, h = 1.2, $fn = 20);
    }
  }
  color("black") translate([5.1, (25 - 8) / 2, 0.9]) cube([8, 8, 5.25]);
  color("beige") translate([0, 2.25, -2.8]) cube([5.6, 25 - 4.5, 2.85]);
  color("brown") translate([-20, (25 - 16.2) / 2, -1.27]) cube([20.5, 16.2, 0.1]);
}

module mountplate() {
  marginx = 5;
  marginy = 15;
  boardoffset = 3;
// need IMU, battery charger circuit
// battery itself can stick to the car body
  difference() {
    union() {
      translate([-marginx, -marginy, 0])
        cube([85 + 2*marginx, 56 + 2*marginy, 1]);
      for (h = RPi_mounting_holes) {
        translate(h + [0, 0, 0.9]) {
          cylinder(d = 6, h= 0.1 + boardoffset, $fn = 20);
        }
      }
    }
    for (h = RPi_mounting_holes) {
      translate(h + [0, 0, -0.1]) {
        cylinder(d = 2.26, h= 1.2 + boardoffset, $fn = 20);
      }
    }
  }
  #translate([0, 0, boardoffset + 1]) rpi2();
}

module cameramount() {
  rpicam();
}

module chargeboard() {
  cube([22, 37, 7]);
}

module backuplipoly() {
  cube([60, 36, 7]);
}

module car() {
  import("rustler.stl");
}

alignplate_thick = 18/2.54;
alignplate_thick2 = 6/2.54;
alignplate_radius = 24/2.54;
Servoplate_width = 124;
module alignplate_shape() {
  r = alignplate_radius;
  linear_extrude(alignplate_thick) {
    minkowski() {
      circle(r=r);
      polygon(points = [
          [-Servoplate_width/2 + 19, 253/2],
          [-Servoplate_width/2 + 50 - r, 114/2 - r],
          [-Servoplate_width/2 + 165 - r, 114/2 - r],
          [Servoplate_width/2 - 20, 134/2],
          [Servoplate_width/2 - 20, -134/2],
          [-Servoplate_width/2 + 165 - r, -114/2 + r],
          [-Servoplate_width/2 + 50 - r, -114/2 + r],
          [-Servoplate_width/2 + 19, -253/2]]);
    }
  }
}

module alignplate_base() {
  overbore = 2;
  r = alignplate_radius;
  difference() {
    alignplate_shape();
    difference() {
      // thin the plate out where not necessary to be thick, but leave a
      // rigidity spar in the middle
      union() {
        translate([-250 - r, -110, alignplate_thick2])
          cube([450 + r, 220, alignplate_thick - alignplate_thick2 + 0.1]);
        translate([200, -45, alignplate_thick2])
          cube([30, 90, alignplate_thick - alignplate_thick2 + 0.1]);
      }
      // spar for rigidity
      translate([-Servoplate_width/2 + 30, -4, alignplate_thick2 - 0.1])
        cube([Servoplate_width - 35, 8, 8]);
    }

    // battery holder clearance
    translate([-Servoplate_width/2 - r - 0.1, -189/2, -0.1])
      cube([15.1 + r, 189, 17.2]);
    translate([-Servoplate_width/2 - r - 0.1, -61/2, -0.1])
      cube([30.1 + r, 61, 17.2]);

    // 4mm (.16) holes .20 from the right edge, 1.34" apart
    translate([Servoplate_width/2 - 20, 134/2, alignplate_thick])
      m4flushscrew(h=20);
    translate([Servoplate_width/2 - 20, -134/2, alignplate_thick])
      m4flushscrew(h=20);
    // 3mm (.12) holes .19 from the left edge, 2.53" apart
    translate([-Servoplate_width/2 + 19, 253/2, alignplate_thick])
      m3flushscrew(h=20);
    translate([-Servoplate_width/2 + 19, -253/2, alignplate_thick])
      m3flushscrew(h=20);

    // weird doohicky on the front
    translate([Servoplate_width/2 - 38 - 17 - 8, -48/2 - 8, -0.1])
      cube([17+ 16, 48+ 16, 27.1]);
    translate([Servoplate_width/2 - 28 - 33 - 8, -12/2 - 8, -0.1])
      cube([28+ 16, 12+ 16, 47.1]);
  }
}

*translate([-20 - 85/2, -56/2, 40]) mountplate();
*cameramount();
*rotate([90, 0, 0]) scale(10.0) color("gray") car();
*translate([10, -65, 10]) backuplipoly();

*translate([-121, 0, 100]) rotate([0, 0, 180]) servoholder();
#servoholder();
alignplate_base();

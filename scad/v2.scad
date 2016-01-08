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

translate([-20 - 85/2, -56/2, 40]) mountplate();
*cameramount();
rotate([90, 0, 0]) scale(10.0) color("gray") car();
translate([10, -65, 10]) backuplipoly();

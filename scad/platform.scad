include <rpi.scad>

DRAW_RPI = 1;

module boss(od, id, height) {
  eps = id * 0.1;
  difference() {
    union() {
      cylinder(h=height, d=od, $fn=20);
      for (i = [0, 90, 180, 270]) {
        rotate([90, 0, i])
          linear_extrude(id/2, center=true)
            polygon(points = [[od/2 - eps, 0], [od, 0], [od/2 - eps, height]]);
      }
    }
    translate([0, 0, 1]) cylinder(h=height, d=id, $fn=20);
  }
}

// add two bosses facing up w/ raspberry pi at center
// for sd card clearance height must be at least .15"

// the entire enclosure must be at least 4.75" wide (x direction) to fit the SD
// card and usb ports
module rpi_bosses(height = 4) {
  // 2.2mm is a decent drill size for tapping a #4-40 hole which seems
  // to fit just fine
  clips = [
    [-RPi_length/2 + 7, RPi_width/2],
    [RPi_length/2 - 12, RPi_width/2],
    [-RPi_length/2 + 7, -RPi_width/2],
    [RPi_length/2 - 12, -RPi_width/2],
    [-RPi_length/2, -RPi_width/2 + 12],
  ];
  pcb_h = 3;
  scale([1,1,1]/.254) {
    for (h = RPi_mounting_holes) {
      translate([h[0], h[1], 0])
        boss(5, 2.2, height);
    }
    difference() {
      for (c = clips) {
        translate([c[0], c[1], (height + pcb_h)/2])
          cube([4, 4, height + pcb_h - 0.01], center=true);
      }
      translate([0, 0, height + pcb_h/2])
        cube([RPi_length, RPi_width, pcb_h], center=true);
    }
    if (DRAW_RPI) {
      // rpi circuit board:
      %translate([0, 0, height]) {
        RPi();
      }
    }
  }
}

module platform1() {
  h = 10;
  translate([0, 0, h/2]) cube([500, 375, h], center=true);
}

module platform2() {
  h = 10;
  translate([0, 0, h/2]) cube([475, 250, h], center=true);
  // raspberry pi mounting screw bosses
  translate([-20, 0, h]) rpi_bosses();
  // camera holder enclosure
}

// render with mm scale coordinates
scale([.254, .254, .254]) {
/*
  rotate([180, 0, 0]) batteryholder();

  translate([0, -20 - 375/2, 75])
    platform1();

  translate([0, 20 + 50 + 300/2, 20])
    platform2();
*/
  platform2();
}

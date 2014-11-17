include <rpi.scad>
include <traxxas.scad>

DRAW_RPI = 1;

module boss(od, id, height, w=8) {
  eps = id * 0.1;
  difference() {
    union() {
      cylinder(h=height, d=od, $fn=20);
      for (i = [0, 90, 180, 270]) {
        rotate([90, 0, i])
          linear_extrude(w, center=true)
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
  slop = 0.3;
  clips = [
    [-RPi_length/2 + 7, RPi_width/2 + 0.5, 4, 3],
    [RPi_length/2 - 12, RPi_width/2 + 0.5, 4, 3],
    [-RPi_length/2 + 7, -RPi_width/2 - 0.5, 4, 3],
    [RPi_length/2 - 12, -RPi_width/2 - 0.5, 20, 3],
    [-RPi_length/2 - 0.5, -RPi_width/2 + 12, 3, 4],
  ];
  pcb_h = 3;
  scale([1,1,1]/.254) {
    for (h = RPi_mounting_holes) {
      // adding 0.02 height just so it's not totally coplanar with the pcb
      // cutout subtracted below
      translate([h[0], h[1], 0])
        boss(5, 2.2, height+0.02, 2);
    }
    difference() {
      for (c = clips) {
        translate([c[0], c[1], (height + pcb_h)/2])
          cube([c[2], c[3], height + pcb_h - 0.01], center=true);
      }
      translate([0, 0, height + pcb_h/2])
        cube([RPi_length + slop, RPi_width + slop, pcb_h], center=true);
    }
    if (DRAW_RPI) {
      // rpi circuit board:
      %translate([0, 0, height + slop]) {
        RPi();
      }
    }
  }
}

module platform1() {
  h = 4;
  // it's only 1mm thick, so add ridges
  translate([0, 0, h/2])
    cube([500, 375, h], center=true);
  translate([0, 0, h+1]) {
    for (i = [-1 : 1]) {
      translate([125*i, 0, 0])
        cube([8, 375, h*2], center=true);
    }
    cube([500, 8, h*2], center=true);
  }
}

module platform2() {
  h = 4;
  translate([0, 0, h/2])
    cube([475, 250, h], center=true);
  translate([0, 0, h+1]) {
    for (i = [-1 : 1]) {
      translate([475*i/4, 0, 0])
        cube([8, 250, h*2], center=true);
    }
    cube([475, 8, h*2], center=true);
  }
  // raspberry pi mounting screw bosses
  translate([-20, 0, h]) rpi_bosses();
  // camera holder enclosure
}

// the alignment plate screws onto the servo holder plate, and provides the
// mounting base for all components and keeps everything pointing the right way
module alignplate() {
  // we'll define a polygon which winds its way around the features of the
  // servo plate and take a minkowski sum with a circle; we need to fit M3
  // screws with a 6mm conical hole on the left, and M4 screws with an 8mm
  // conical hole on the right.
  // we also need to keep clear of the battery / battery wires cutouts
  // the plate needs to be at least 3mm thick, as that's the depth of the
  // countersunk M4 screw cap heads on the right
  r = 19;
  thick = 12;
  thick2 = 6;
  difference() {
    linear_extrude(thick) {
      minkowski() {
        circle(r=r);
        polygon(points = [
            [-487/2 + 19, 253/2],
            [-487/2 + 50 - r, 114/2 - r],
            [-487/2 + 165 - r, 114/2 - r],
            [487/2 - 20, 134/2],
            [487/2 - 20, -134/2],
            [-487/2 + 165 - r, -114/2 + r],
            [-487/2 + 50 - r, -114/2 + r],
            [-487/2 + 19, -253/2]]);
      }
    }
    // thin the plate out where not necessary to be thick
    translate([-250, -110, thick2]) cube([455, 220, thick - thick2 + 0.1]);
    translate([200, -45, thick2]) cube([30, 90, thick - thick2 + 0.1]);

    // battery holder clearance
    translate([-487/2 - 0.1, -189/2, -0.1]) cube([15.1, 189, 17.2]);
    translate([-487/2 - 0.1, -61/2, -0.1]) cube([30.1, 61, 17.2]);
    // 4mm (.16) holes .20 from the right edge, 1.34" apart
    translate([487/2 - 20, 134/2, thick]) m4flushscrew(h=20);
    translate([487/2 - 20, -134/2, thick]) m4flushscrew(h=20);
    // 3mm (.12) holes .19 from the left edge, 2.53" apart
    translate([-487/2 + 19, 253/2, thick]) m3flushscrew(h=20);
    translate([-487/2 + 19, -253/2, thick]) m3flushscrew(h=20);

    // weird doohicky on the front
    translate([487/2 - 38 - 17, -48/2, -0.1]) cube([17, 48, 27.1]);
    translate([487/2 - 28 - 33, -12/2, -0.1]) cube([28, 12, 47.1]);
  }
  // mounting bosses for rpi chassis
  translate([-487/2 + 22, 100, thick2]) boss(15, 8.9, 15);
  translate([-487/2 + 22, -100, thick2]) boss(15, 8.9, 15);
  translate([-487/2 + 200, 0, thick2]) boss(15, 8.9, 15);

  // captive mounting for IMU, 22x17mm, .15" from base, board is .07" thick
  difference() {
    translate([487/2 - 145, -98/2, thick2-0.1]) cube([74, 98, 25.1]);
    translate([487/2 - 146, -88/2, thick2+15]) cube([70, 88, 30]);
    translate([487/2 - 146, -83/2, thick2]) cube([67, 83, 20]);
  }
  translate([487/2 - 145 + 70 - 11, 88/2 - 13, thick2]) boss(15, 8.9, 15);

  // raspberry pi camera mount
  camheight = 40;  // camera is this many mm above platform
  difference() {
    translate([487/2 - 9, -55, thick2 - 0.1]) cube([8, 110, 45 + camheight/.254]);
    translate([487/2 - 9, 0, (camheight - 2.5)/.254]) cube([8, 8, 8]/.24, center=true);
  }
  for (h = RPiCam_mounting_holes) {
    translate([487/2 - 9, 0, camheight/.254])
      rotate([-90, 180, 90]) translate(h / .254) boss(10, 6, 8, 5);
  }
  for (x = [-45, 45]) {
    translate([487/2 - 3, x + 2.5, thick2 - 0.1])
      rotate([90, 0, 0])
        linear_extrude(5) {
          polygon(points = [[-35, 0], [0, camheight/.254 - 40], [0, 0]]);
        }
  }

  // draw camera board itself in previews
  %scale(1/.254) translate([57, 0, camheight]) rotate([90, 0, 90]) RPiCamera();
}

// render with mm scale coordinates
scale([.254, .254, .254]) {
  *%rotate([180, 0, 0]) batteryholder();
  *%translate([0, 50, -60]) battery();

  *translate([0, -20 - 375/2, 75])
    platform1();

  //translate([0, 20 + 50 + 300/2, 20])
  translate([0, -130, 20])
    platform2();

  rotate([0, 0, 90]) {
    alignplate();
    %servoholder();
  }
}



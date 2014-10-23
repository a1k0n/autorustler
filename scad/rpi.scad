// Raspberry Pi, in mm scale
// origin is centered at bottom of board
// most measurements are from my RPi own board with a dial calipers
// others from
// http://www.raspberrypi.org/raspberry-pi-rev2-template-with-mounting-holes/

RPi_length = 85;
RPi_width = 56;

RPi_mounting_holes = [
  [-RPi_length/2 + 5, -RPi_width/2 + 12.5],
  [RPi_length/2 - 25.5, RPi_width/2 - 18]
];

module HeaderPair() {
  color("black", alpha = 0.7)
    translate([0, 0, 2.5/2])
      cube([2.53, 5.08, 2.5], center=true);
  color("silver", alpha = 0.7) {
    translate([0, -1.27, 2.5])
      cube([0.6, 0.6, 11.5], center=true);
    translate([0, 1.27, 2.5])
      cube([0.6, 0.6, 11.5], center=true);
  }
}

module RPi() {
  // sd card protrusion
  sdcard_frombottom = 14;
  sdcard_width = 24;
  sdcard_bottom = RPi_width/2 - sdcard_frombottom;
  sdcard_center = sdcard_bottom - sdcard_width/2;

  pcb_h = 1.52;

  translate([0,0,pcb_h/2]) {
    // render sdcard first so it shows through the bottom
    translate([RPi_length/2 + 2, -sdcard_center, -.76-1.05])
      color([0, 0, 0, 0.5])
        cube([32, 24, 2.1], center = true);

    // circuit board itself, with mounting holes
    color([0, 0.4, 0, 0.5]) {
      difference() {
        cube([RPi_length, RPi_width, pcb_h], center = true);
        for (h = RPi_mounting_holes) {
          translate(h)
            cylinder(d = 2.9, h = 2, center = true, $fn = 20);
        }
      }
    }
  }

  // top & thru-hole components
  translate([0, 0, pcb_h - .01]) {
    // cpu & flat flex connectors
    color("black", alpha=0.7) {
      translate([-6 + 2.5, -6, 0]) cube([12, 12, 1.4]);
      translate([-RPi_length/2 + 25, RPi_width/2 - 22.5, 0]) cube([4, 22.5, 5.6]);
    }

    // P1 header is 1mm from the bottom-right
    for (i = [0 : 12]) {
      translate([RPi_length/2 - 1 - 2.54/2 - 2.54*i, -RPi_width/2 + 1 + 2.54, 0])
        HeaderPair();
    }

    // usb + ethernet + hdmi
    color("silver", alpha=0.7) {
      translate([-RPi_length/2 - 7,-RPi_width/2 + 18.5,0])
        cube([17,14.5,16]);
      translate([RPi_length/2 - 5.5, RPi_width/2 - 11.5, 0])
        cube([6, 8, 2.5]);
      translate([-RPi_length/2 - 1, RPi_width/2 - 18, 0])
        cube([21.5, 16, 14.5]);
      translate([-RPi_length/2 + 33, RPi_width/2 - 12 + .65, 0])
        cube([15, 12, 6]);
    }
  }
}

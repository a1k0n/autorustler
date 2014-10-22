// all coordinates are in hundredths of an inch
module batteryholder() {
  difference() {
    union() {
      linear_extrude(height = 7) {
        difference() {
          union () {
            translate([-111.5, 0, 0]) circle(r = 20.5, $fn = 100);
            translate([111.5, 0, 0]) circle(r = 20.5, $fn = 100);
            square([223, 41], center = true);
          }
          translate([-111.5, 0, 0]) circle(r = 12, $fn=40);
          translate([111.5, 0, 0]) circle(r = 12, $fn=40);
        }
      }
      translate([0,0,10.4]) cube([189, 41, 7.2], center = true);
    }
    translate([0, 0, 10.4]) cube([52, 27, 7.4], center = true);
    translate([-60, 0, 10.4]) cube([52, 27, 7.4], center = true);
    translate([60, 0, 10.4]) cube([52, 27, 7.4], center = true);
  }
}

module platform1() {
  h = 10;
  translate([0, -20 - 375/2, 75+h/2]) cube([500, 375, h], center=true);
}

module platform2() {
  h = 10;
  translate([0, 20 + 50 + 300/2, 20+h/2]) cube([300, 250, h], center=true);
  // raspberry pi mounting screw bosses
  // camera holder enclosure
}

// render with inch scale coordinates
scale([1/100, 1/100, 1/100]) {
  batteryholder();
  platform1();
  platform2();
}

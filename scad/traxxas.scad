// all coordinates are in hundredths of an inch
module batteryholder() {
  color("gray")
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

module battery() {
  color("red") rotate([90, 0, 0]) for (x = [-1, 1]) {
    for (y = [0, 1, 2]) {
      translate([x * 45, 0, y * (165 + 18)]) cylinder(165, d=88.5);
    }
    translate([0, 88.5*sqrt(3)/2, 165 + 18]) cylinder(165, d=88.5);
  }
}

module m4flushscrew(h) {
  overbore = 2;
  translate([0,0,-h-1]) cylinder(h=h+2, d=16 + overbore*2);
  translate([0,0,-12]) cylinder(h=12.1, r1=8 + overbore, r2=16 + overbore);
}

module m3flushscrew(h) {
  overbore = 2;
  translate([0,0,-h-1]) cylinder(h=h+2, d=12 + overbore*2);
  translate([0,0,-8]) cylinder(h=8.1, r1=6 + overbore, r2=12 + overbore);
}

Servoplate_width = 491;

module servoholder() {
  r = 17;
  w1 = 287 - 2*r;
  w2 = 174 - 2*r;
  l = Servoplate_width - 2*r;
  l1 = 175;
  l2 = 100;
  color("gray") {
  translate([0,0,-17]) difference() {
    linear_extrude(17) minkowski() {
      polygon(points = [
          [-l/2, w1/2], [-l/2 + l1, w1/2], [l/2 - l2, w2/2], [l/2, w2/2],
          [l/2, -w2/2], [l/2 - l2, -w2/2], [-l/2 + l1, -w1/2], [-l/2, -w1/2]]);
      circle(r=r);
    }
    translate([-l/2 - r - 0.1, -189/2, -0.1]) cube([15.1, 189, 17.2]);
    translate([-l/2 - r - 0.1, -61/2, -0.1]) cube([30.1, 61, 17.2]);
    // 4mm (.16) holes .20 from the right edge, 1.34" apart
    translate([l/2 + r - 20, 134/2, 17]) m4flushscrew(h=20);
    translate([l/2 + r - 20, -134/2, 17]) m4flushscrew(h=20);
    // 3mm (.12) holes .19 from the left edge, 2.53" apart
    translate([-l/2 - r + 19, 253/2, 17]) m3flushscrew(h=20);
    translate([-l/2 - r + 19, -253/2, 17]) m3flushscrew(h=20);
  }
  // servo horn clearance bumps
  translate([-l/2 - r + 60, w1/2 + r - 75, -0.1]) cube([85,75,17.1]);
  translate([-l/2 - r + 60, -w1/2 - r, -0.1]) cube([85,75,17.1]);
  // weird doohicky on the front
  translate([l/2 + r - 38 - 17, -48/2, -0.1]) cube([17, 48, 27.1]);
  translate([l/2 + r - 28 - 33, -12/2, -0.1]) cube([28, 12, 47.1]);
  }
}

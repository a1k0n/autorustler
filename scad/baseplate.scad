include <fasteners.scad>
include <traxxas.scad>

$overbore = 0.25;

// All screws are #4-40, 2.2606 internal diameter
BasePlateMountingScrews = [
  [-55, 20], [-55, -20],
  [-30, 10], [-30, -10]
];

module IMUMount() {
  // dimensions of 9DOF PCB
  width = 22.1;
  length = 16.85;
  depth = 1.66;  // PCB is 1.66mm thick or so
  height = 3;  // stand off 3mm
}

module BasePlate() {
  thick = 2;
  difference() {
    union() {
      linear_extrude(thick) minkowski() {
        polygon(points = [
            [Servoplate_screws[0][0], Servoplate_screws[0][1]],
            [40, -10],
            [-55, -10],
            [Servoplate_screws[2][0], Servoplate_screws[2][1]],
            [Servoplate_screws[3][0], Servoplate_screws[3][1]],
            [-55, 10],
            [40, 10],
            [Servoplate_screws[1][0], Servoplate_screws[1][1]]]);
        circle(r=5);
      }
      for (h = BasePlateMountingScrews) {
        translate([h[0], h[1], thick-0.1])
          cylinder(d=5, h=5);
      }
    }

    for (c = Servoplate_cutouts) {
      translate([c[0], -c[1]/2, -0.01]) cube([c[2], c[1], 4.4]);
    }

    for (d = Servoplate_doohicky) {
      margin = 1;
      translate([d[0] - d[1] - margin, -d[2]/2 - margin, -0.1])
        cube([d[1] + 2*margin, d[2] + 2*margin, d[3]]);
    }

    for (s = Servoplate_screws) {
      translate([s[0], s[1], thick + 0.001]) {
        if (s[2] == 3) m3flushscrew(h=20);
        else m4flushscrew(h=20);
      }
    }

    for (h = BasePlateMountingScrews) {
      translate([h[0], h[1], -0.1])
        cylinder(d=2.2606 + $overbore, h=thick+20);
    }
  }
}

translate([0,0,-0.01]) %servoholder();
BasePlate();

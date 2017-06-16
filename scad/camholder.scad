wide_cam_width = 25.15;
wide_cam_height = 24.15;
wide_cam_mount = [
  [-wide_cam_width/2 + 2.1, wide_cam_height/2 - 2.1],
  [wide_cam_width/2 - 2.1, wide_cam_height/2 - 2.1],
  [-wide_cam_width/2 + 2.1, wide_cam_height/2 - 14.6 - 1.1],
  [wide_cam_width/2 - 2.1, wide_cam_height/2 - 14.6 - 1.1],
];

module CameraMount() {
  difference() {
    union() {
      translate([-wide_cam_width/2, -wide_cam_height/2, 0])
        cube([wide_cam_width, wide_cam_height, 2]);
      for (h = wide_cam_mount) {
        translate([h[0], h[1], 2 - 0.1]) {
          cylinder(d=4.25, h=5.1, $fn=10);
        }
      }
    }
    for (h = wide_cam_mount) {
      translate([h[0], h[1], -0.1]) {
        cylinder(d=2.5, h=8.2, $fn=10);
      }
    }
    // hole for lens
    translate([0, wide_cam_height/2 - 2.3 - 7, -0.1])
      cylinder(d=15, h=5);
  }
}

module BumperSpar(sparlen) {
  rotate([90, 0, 0]) rotate([0, 90, 0]) linear_extrude(height = 2)
    polygon(points=[
      [-1.72, 3.0],
      [sparlen, 3], [sparlen, -0.1], [0, -0.1], [-1.72, 3.0]]);
}

bb_len = 105;
module BumperBracket() {
  // 30.75 inner to inner
  // 3.5mm diameter holes (we will probably have to drill out)
  // 60 deg angle from vertical
  rotate([-60, 0, 0]) {
    difference() {
      translate([-25, -20, 0])
        cube([50, 20, 2]);
      translate([-30.75/2 - 3.5/2, -6, -0.1])
        cylinder(d=4.5, h=2.2, $fn=10);
      translate([30.75/2 + 3.5/2, -6, -0.1])
        cylinder(d=4.5, h=2.2, $fn=10);
    }
  }
  linear_extrude(height = 2)
    polygon(points=[
      [-25, 0],
      [-wide_cam_width/2, bb_len],
      [wide_cam_width/2, bb_len],
      [25, 0], [-25, 0]]);
  translate([-12, 0, 1.0]) BumperSpar(bb_len - 5);
  translate([10, 0, 1.0]) BumperSpar(bb_len - 5);
}

BumperBracket();
translate([0, bb_len+10, 0]) CameraMount();

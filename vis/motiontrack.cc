#include <cmath>
#include <cstdio>
#include <iostream>

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Modified from Ceres example simple bundle adjuster:
// the camera has three parameters for focal length & radial distortion, plus
// six extra parameters for orientation.
struct ReprojectionError {
  ReprojectionError(double x0, double y0, double x1, double y1)
      : x0_(x0), y0_(y0), x1_(x1), y1_(y1) {}

  template <typename T>
  bool operator()(const T* const cam_intrinsic,
                  const T* const cam_motion,
                  const T* const point,
                  T* residuals) const {
    // cam_motion[0,1,2] are the angle-axis rotation.
    T p1[3];
    ceres::AngleAxisRotatePoint(cam_motion, point, p1);

    // cam_motion[3,4,5] are the translation.
    p1[0] += cam_motion[3];
    p1[1] += cam_motion[4];
    p1[2] += cam_motion[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp0 = - point[0] / point[2];
    T yp0 = - point[1] / point[2];
    T xp1 = - p1[0] / p1[2];
    T yp1 = - p1[1] / p1[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = cam_intrinsic[1];
    const T& l2 = cam_intrinsic[2];
    T r02 = xp0*xp0 + yp0*yp0;
    T r12 = xp1*xp1 + yp1*yp1;
    T distortion0 = T(1.0) + r02 * (l1 + l2 * r02);
    T distortion1 = T(1.0) + r12 * (l1 + l2 * r12);

    // Compute final projected point position.
    const T& focal = cam_intrinsic[0];
    T predicted_x0 = focal * distortion0 * xp0;
    T predicted_y0 = focal * distortion0 * yp0;
    T predicted_x1 = focal * distortion1 * xp1;
    T predicted_y1 = focal * distortion1 * yp1;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x0 - T(x0_);
    residuals[1] = predicted_y0 - T(y0_);
    residuals[2] = predicted_x1 - T(x1_);
    residuals[3] = predicted_y1 - T(y1_);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const double x0, const double y0, const double x1, const double y1) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 4, 3, 6, 3>(
        new ReprojectionError(x0, y0, x1, y1));
  }

  double x0_, y0_;
  double x1_, y1_;
};

#if 0
struct MyIterCB : public ceres::IterationCallback {
 public:
  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    printf("camera focal length %f, distortion %f %f\n",
           cam_intrinsic[0], cam_intrinsic[1], cam_intrinsic[2]);
    return ceres::SOLVER_CONTINUE;
  }
} iter_cb;
#endif

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  int frameno = 0;
  double x1, y1, x2, y2, err;
  double cam_motion[6 * 1550];  // fixme
  double point_xyz[3 * 300000];  // fixme
  double cam_intrinsic[3];
  memset(point_xyz, 0, sizeof(point_xyz));
  memset(cam_motion, 0, sizeof(cam_motion));
  cam_intrinsic[0] = 12.39;  // focal length
  cam_intrinsic[1] = -6.71e-04;  // r^2 distortion
  cam_intrinsic[2] = 3.55e-06;  // r^4 distortion
  int pointno = 0;
  ceres::Problem problem;
  fprintf(stderr, "loading...");
  fflush(stderr);
  while (fscanf(stdin, "kf %d %lf %lf %lf %lf %lf\n",
                &frameno, &x1, &y1, &x2, &y2, &err) == 6) {
#if 0
  std::vector<std::vector<Eigen::Vector4f> > frames;
    if (frames.size() < frameno)
      frames.push_back(std::vector<Eigen::Vector4f>());
    frames[frameno-1].push_back(Eigen::Vector4f(x1, y1, x2, y2));
#endif
    ceres::CostFunction* cost_function =
        ReprojectionError::Create(x1 - 160, y1 - 120,
                                  x2 - 160, y2 - 120);
    int p = pointno*3;
    point_xyz[p + 0] = x1 - 160;
    point_xyz[p + 1] = y1 - 120;
    point_xyz[p + 2] = -1.0;
    problem.AddResidualBlock(cost_function,
                             NULL /* new ceres::CauchyLoss(0.5) */,
                             cam_intrinsic,
                             &cam_motion[(frameno - 1) * 6],
                             &point_xyz[pointno * 3]);
    pointno++;
  }
  fprintf(stderr, "done, solving...\n");

  ceres::Solver::Options options;
  // options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 4;
  // options.callbacks.push_back(&iter_cb);
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  printf("camera focal length %f, distortion %f %f\n",
         cam_intrinsic[0], cam_intrinsic[1], cam_intrinsic[2]);

  FILE *motion = fopen("motion.txt", "w");
  for (int i = 0; i < frameno; i++) {
    fprintf(motion, "%d %f %f %f %f %f %f\n", i,
            cam_motion[i*6 + 0],
            cam_motion[i*6 + 1],
            cam_motion[i*6 + 2],
            cam_motion[i*6 + 3],
            cam_motion[i*6 + 4],
            cam_motion[i*6 + 5]);
  }
  fclose(motion);

  return 0;
}

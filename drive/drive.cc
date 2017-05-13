#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <deque>

#include "cam/cam.h"
#include "car/pca9685.h"
#include "drive/controller.h"
#include "gpio/i2c.h"
#include "imu/imu.h"
#include "input/js.h"

volatile bool done = false;
uint16_t throttle_ = 614, steering_ = 614;

void handle_sigint(int signo) { done = true; }

I2C i2c;
PCA9685 pca(i2c);
IMU imu(i2c);
Eigen::Vector3f accel_(0, 0, 0), gyro_(0, 0, 0);

// asynchronous flush to sdcard
struct FlushEntry {
  int fd_;
  uint8_t *buf_;
  size_t len_;

  FlushEntry() { buf_ = NULL; }
  FlushEntry(int fd, uint8_t *buf, size_t len):
    fd_(fd), buf_(buf), len_(len) {}

  void flush() {
    if (len_ == -1) {
      fprintf(stderr, "FlushThread: closing fd %d\n", fd_);
      close(fd_);
    }
    if (buf_ != NULL) {
      if (write(fd_, buf_, len_) != len_) {
        perror("FlushThread write");
      }
      delete[] buf_;
      buf_ = NULL;
    }
  }
};

class FlushThread {
 public:
  FlushThread() {
    pthread_mutex_init(&mutex_, NULL);
    sem_init(&sem_, 0, 0);
  }

  ~FlushThread() {
    // terminate the thread?
  }

  bool Init() {
    if (pthread_create(&thread_, NULL, thread_entry, this) != 0) {
      perror("FlushThread: pthread_create");
      return false;
    }
    return true;
  }

  void AddEntry(int fd, uint8_t *buf, size_t len) {
    pthread_mutex_lock(&mutex_);
    flush_queue_.push_back(FlushEntry(fd, buf, len));
    pthread_mutex_unlock(&mutex_);
    sem_post(&sem_);
#if 0
    int semval;
    sem_getvalue(&sem_, &semval);
    fprintf(stderr, "Flusher: qsize %d sem %d\n", flush_queue_.size(), semval);
#endif
  }

 private:
  static void* thread_entry(void* arg) {
    FlushThread *self = reinterpret_cast<FlushThread*>(arg);

    fprintf(stderr, "FlushThread: started\n");

    for (;;) {
      sem_wait(&self->sem_);
      pthread_mutex_lock(&self->mutex_);
      if (!self->flush_queue_.empty()) {
        FlushEntry e = self->flush_queue_.front();
        self->flush_queue_.pop_front();
        pthread_mutex_unlock(&self->mutex_);
        e.flush();
      } else {
        pthread_mutex_unlock(&self->mutex_);
      }
    }
  }

  std::deque<FlushEntry> flush_queue_;
  pthread_mutex_t mutex_;
  pthread_t thread_;
  sem_t sem_;
};

FlushThread flush_thread_;

class Driver: public CameraReceiver {
 public:
  Driver() {
    output_fd_ = -1;
    frame_ = 0;
    autosteer_ = false;
  }

  bool StartRecording(const char *fname) {
    if (!strcmp(fname, "-")) {
      output_fd_ = fileno(stdout);
    } else {
      output_fd_ = open(fname, O_CREAT|O_TRUNC|O_WRONLY, 0666);
    }
    if (output_fd_ == -1) {
      perror(fname);
      return false;
    }
    return true;
  }

  bool IsRecording() {
    return output_fd_ != -1;
  }

  void StopRecording() {
    if (output_fd_ == -1) {
      return;
    }
    flush_thread_.AddEntry(output_fd_, NULL, -1);
    output_fd_ = -1;
  }

  ~Driver() {
    StopRecording();
  }

  void OnFrame(uint8_t *buf, size_t length) {
    if (IsRecording()) {
      struct timeval t;
      gettimeofday(&t, NULL);
      size_t flushlen = length + 4+4+2+2+6*4;
      // copy our frame, push it onto a stack to be flushed
      // asynchronously to sdcard
      uint8_t *flushbuf = new uint8_t[flushlen];
      memcpy(flushbuf, &t.tv_sec, 4);
      memcpy(flushbuf+4, &t.tv_usec, 4);
      memcpy(flushbuf+8, &throttle_, 2);
      memcpy(flushbuf+10, &steering_, 2);
      memcpy(flushbuf+12, &accel_[0], 4);
      memcpy(flushbuf+12+4, &accel_[1], 4);
      memcpy(flushbuf+12+8, &accel_[2], 4);
      memcpy(flushbuf+24, &gyro_[0], 4);
      memcpy(flushbuf+24+4, &gyro_[1], 4);
      memcpy(flushbuf+24+8, &gyro_[2], 4);
      memcpy(flushbuf+36, buf, length);
      flush_thread_.AddEntry(output_fd_, flushbuf, flushlen);
    }

    float u_a = throttle_ / 204.8 - 3.0;
    float u_s = steering_ / 204.8 - 3.0;
    controller_.UpdateState(buf, length, u_a, u_s, accel_, gyro_);

    if (autosteer_ && controller_.GetControl(&u_a, &u_s)) {
      steering_ = std::max(0, (int) ((u_s + 3.0) * 204.8));
      throttle_ = std::max(0, (int) ((u_a + 3.0) * 204.8));
      pca.SetPWM(0, steering_);
      pca.SetPWM(1, throttle_);
    }
  }

  bool autosteer_;
  DriveController controller_;

 private:
  int output_fd_;
  int frame_;
};

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  if (argc < 2) {
    fprintf(stderr, "%s [[output.yuv] [fps]\n", argv[0]);
    return 1;
  }

  int fps = 30;

  if (argc > 2) {
    fps = atoi(argv[2]);
    if (fps == 0) {
      fprintf(stderr, "invalid fps %d\n", fps);
      return 1;
    }
  }

  if (!flush_thread_.Init()) {
    return 1;
  }

  if (!Camera::Init(640, 480, fps))
    return 1;

  JoystickInput js;

  if (!i2c.Open()) {
    return 1;
  }

  if (!js.Open()) {
    return 1;
  }

  pca.Init(100);  // 100Hz output
  pca.SetPWM(0, 614);
  pca.SetPWM(1, 614);

  imu.Init();

  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d camera on @%d fps\n", tv.tv_sec, tv.tv_usec, fps);

  Driver driver;
  if (!Camera::StartRecord(&driver)) {
    return 1;
  }

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d started camera\n", tv.tv_sec, tv.tv_usec);

  int recording_num = 0;

  while (!done) {
    int t = 0, s = 0;
    uint16_t b = 0;
    if (js.ReadInput(&t, &s, &b)) {
      gettimeofday(&tv, NULL);
      if ((b & 0x40) && !driver.IsRecording()) {  // start button: start recording
        char fnamebuf[256];
        snprintf(fnamebuf, sizeof(fnamebuf), "%s-%d.yuv", argv[1], recording_num++);
        if (driver.StartRecording(fnamebuf)) {
          fprintf(stderr, "%d.%06d started recording %s\n", tv.tv_sec, tv.tv_usec, fnamebuf);
        }
      }
      if ((b & 0x30) == 0x30 && driver.IsRecording()) {
        driver.StopRecording();
        fprintf(stderr, "%d.%06d stopped recording\n", tv.tv_sec, tv.tv_usec);
      }

      if (b & 0x10) {  // not sure which button this is
        if (!driver.autosteer_) {
          fprintf(stderr, "%d.%06d autosteer ON\n", tv.tv_sec, tv.tv_usec);
          driver.autosteer_ = true;
        }
      } else {
        if (driver.autosteer_) {
          driver.autosteer_ = false;
          fprintf(stderr, "%d.%06d autosteer OFF\n", tv.tv_sec, tv.tv_usec);
        }
      }

      if (b & 0x01) {
        driver.controller_.ResetState();
        fprintf(stderr, "reset kalman filter\n");
      }

      if (!driver.autosteer_) {
        steering_ = 614.4 - 204.8*s / 32767.0;
        pca.SetPWM(0, steering_);
        throttle_ = 614.4 + 204.8*t / 32767.0;
        pca.SetPWM(1, throttle_);
      }
    }
    {
      float temp;
      imu.ReadIMU(&accel_, &gyro_, &temp);
    }
    usleep(1000);
  }

  Camera::StopRecord();
}

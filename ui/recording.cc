#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <unistd.h>
#include <vector>
#include "ui/uistate.h"

using std::vector;

Recording::Recording() {
  fd_ = -1;
  active_buf_ = 0;
  bufsize_ = 0;
  recording_ = false;
  pthread_mutex_init(&mutex_, NULL);
  sem_init(&sema_, 0, 0);
}

void* Recording::RecordingThread() {
  while (recording_) {
    sem_wait(&sema_);
    // lock and swap buffers
    pthread_mutex_lock(&mutex_);
    int b = active_buf_;
    active_buf_ ^= 1;
    pthread_mutex_unlock(&mutex_);
    if (outbuf_[b].empty())
      continue;
    // write all output
    if (writev(fd_, &outbuf_[b][0], outbuf_[b].size()) == -1) {
      perror("writev");
      break;
    }
    for (int i = 0; i < outbuf_[b].size(); i++) {
      delete[] reinterpret_cast<uint8_t*>(outbuf_[b][i].iov_base);
      bufsize_ -= outbuf_[b][i].iov_len;
    }
    outbuf_[b].clear();
  }
  close(fd_);
  fd_ = -1;
  return NULL;
}

bool Recording::StartWriting() {
  if (pthread_mutex_lock(&mutex_) != 0) {
    perror("recording_mutex");
    return false;
  }
  return true;
}

void Recording::Write(const uint8_t *buf, size_t len) {
  // allocate a copy
  uint8_t *buf2 = new uint8_t[len];
  memcpy(buf2, buf, len);
  struct iovec iov = { const_cast<uint8_t*>(buf2), len };
  outbuf_[active_buf_].push_back(iov);
  bufsize_ += len;
  totalsize_ += len;
}

bool Recording::StopWriting() {
  if (pthread_mutex_unlock(&mutex_) != 0) {
    perror("recording_mutex");
    return false;
  }
  sem_post(&sema_);
  return true;
}

void* Recording::RecordingThreadThunk(void *self) {
  Recording *r = static_cast<Recording*>(self);
  return r->RecordingThread();
}

bool Recording::StartRecording() {
  char fname[1024];
  const char tmpl[] = "/home/pi/rustlerlog-XXXXXX";
  memcpy(fname, tmpl, sizeof(tmpl));
  fd_ = mkstemp(fname);
  if (fd_ == -1) {
    perror(fname);
    return false;
  }
  gettimeofday(&start_time_, NULL);
  totalsize_ = 0;
  recording_ = true;
  if (pthread_create(&thread_, NULL, RecordingThreadThunk,
                     static_cast<void*>(this)) != 0) {
    perror("pthread_create(recording)");
    return false;
  }
  fprintf(stderr, "recording to %s @%d\n", fname, fd_);
  return true;
}

void Recording::StopRecording() {
  recording_ = false;
  sem_post(&sema_);
  if (pthread_join(thread_, NULL) == -1) {
    perror("StopRecording");
  }
  fprintf(stderr, "recording stopped.\n");
}

int Recording::GetWriteSpeed() {
  timeval tv1;
  gettimeofday(&tv1, NULL);
  double dt = (tv1.tv_usec - start_time_.tv_usec) * (1.0 / 1000000.0) +
      (tv1.tv_sec - start_time_.tv_sec);
  return static_cast<int>(totalsize_ * 1.0 / (1024.0 * dt));
}

int Recording::GetRecordTime() {
  timeval tv1;
  gettimeofday(&tv1, NULL);
  return (tv1.tv_usec - start_time_.tv_usec) / 1000 +
      (tv1.tv_sec - start_time_.tv_sec) * 1000;
}

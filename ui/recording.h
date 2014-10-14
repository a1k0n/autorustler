#ifndef UI_RECORDING_H_
#define UI_RECORDING_H_

#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>
#include <vector>

struct iovec;

struct RecordHeader {
  uint32_t ts_sec, ts_usec;
  uint32_t len;
  uint32_t recordtype;

  void Init(uint32_t l, uint32_t typ) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ts_sec = tv.tv_sec;
    ts_usec = tv.tv_usec;
    recordtype = typ;
    len = l;
  }
};

class Recording {
 public:
  Recording();

  bool StartRecording();
  void StopRecording();

  bool StartWriting();
  void Write(const uint8_t *buf, size_t len);
  bool StopWriting();

  int GetBufferSize() { return bufsize_; }
  int GetWriteSpeed();  // return data write speed in kb/sec
  int GetRecordTime();  // return recording time in milliseconds

 private:
  void* RecordingThread();
  static void* RecordingThreadThunk(void* self);

  volatile bool recording_;
  pthread_t thread_;
  pthread_mutex_t mutex_;
  sem_t sema_;
  std::vector<iovec> outbuf_[2];
  int active_buf_;
  int fd_;
  int bufsize_;

  off_t totalsize_;
  timeval start_time_;
};

#endif  // UI_RECORDING_H_

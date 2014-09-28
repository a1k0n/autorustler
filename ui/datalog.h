#include <stdio.h>

class DataLogger {
 public:
  ~DataLogger();

  bool Open(const char *fname);
  bool Close();

  // copies message into internal buffer; caller may destroy original message
  bool AddMessage(struct timeval tstamp, uint8_t msgtype,
                  const void *msgbuf, size_t buflen);

 private:
  FILE *fp;
};

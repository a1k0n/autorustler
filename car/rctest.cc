          struct timeval tv;
          gettimeofday(&tv, NULL);
          printf("%ld.%d %d %d\n", tv.tv_sec, tv.tv_usec, ch1_, ch2_);
          {
            static int frame_ = 0;
            ++frame_;
            send_cmd(fd, 127.5 + 100.0 * sin(frame_ * 0.01), ch2_);
          }
        } else {
          printf("# bad cksum %d %d\n", cksum_ - c, ~c);
        }


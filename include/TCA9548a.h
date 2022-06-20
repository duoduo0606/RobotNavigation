#ifndef TCA9548A_H
#define TCA9548A_H

#include <stdint.h>

class TCA9548a {
  public:
    TCA9548a();
    ~TCA9548a();
    int init(int id);
    void set_channel(uint8_t channel);
    void no_channel();
    float get_data();
  private:
    int fd;
};

#endif

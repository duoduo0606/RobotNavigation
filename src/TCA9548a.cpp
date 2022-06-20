/*
Based on the TCA9548A datasheet.
Author: Bryan Casanelli - bryancasanelli@gmail.com
*/

#include "TCA9548a.h"
#include <wiringPiI2C.h>
#include <wiringPi.h>

TCA9548a::TCA9548a(){
  wiringPiSetup();
}

TCA9548a::~TCA9548a(){
  no_channel();
}

int TCA9548a::init(int id){
  this->fd = wiringPiI2CSetup(id);
  if (this->fd == -1){return -1;} // Error
  return 0;                       // Ok
}

void TCA9548a::set_channel(uint8_t channel){
  wiringPiI2CWrite(this->fd,1<<channel);
}

void TCA9548a::no_channel(){
  wiringPiI2CWrite(this->fd,0);
}

float TCA9548a::get_data(){
  return wiringPiI2CRead(this->fd);
}

#ifndef __CONTROL_H
#define __CONTROL_H

#include "Config.h"
#include "imu.h"
#include "viewer.h"

void fight_init();
void int_to_5_char(int32_t _int , uint8_t* _ptr_char , uint8_t _offset_ptr);

#endif

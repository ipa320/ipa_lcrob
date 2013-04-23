#ifndef __MCONTROLLER_H__
#define __MCONTROLLER_H__
#include <stdint.h>

#define ENABLE   "EN\r\n"
#define DISABLE  "DI\r\n"
#define POSITION "POS\r\n"

void int2str(char *, int16_t);
void motor_init();
void motor_stop();
void motor_setVel(int16_t, int16_t);
int32_t motor_getPos(uint8_t motor, uint8_t *valid_val);
void motor_reqPos(uint8_t motor);

#endif

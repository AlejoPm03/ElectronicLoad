#ifndef _CONTROL_H
#define _CONTROL_H

#include <avr/io.h>
#include "conf.h"
#include "../lib/bit_utils.h"


void control(void);
void control_init(void);
void control_feedback(void);

float piIo(float r, float y);

#endif /* ifndef _CONTROL_H */

/**
 * @file machine.h
 *
 * @defgroup MACHINE State Machine Module
 *
 * @brief Implements the main state machine of the system.
 *
 */

#ifndef MACHINE_H
#define MACHINE_H 

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "dbg_vrb.h"
#include "conf.h"


#include "dbg_vrb.h"
#ifdef ADC_ON
#include "adc.h"
#endif
#ifdef USART_ON
#include "usart.h"
#endif

// Equations for mode 2 (CTC with TOP OCR2A)
// Note the resolution. For example.. at 150hz, ICR1 = PWM_TOP = 159, so it
#define MACHINE_TIMER_TOP ((F_CPU/(MACHINE_TIMER_PRESCALER))/(MACHINE_TIMER_FREQUENCY) -1)




// machine checks
void check_buffers(void);
void reset_measurements(void);

// debug functions
void print_configurations(void);
void print_system_flags(void);
void print_error_flags(void);

// machine tasks
void task_initializing(void);
void task_idle(void);
void task_running(void);
void task_discharging(void);
void task_error(void);
void task_reset(void);

// the machine itself
void set_machine_initial_state(void);
void machine_init(void);
void machine_run(void);
void set_state_error(void);
void set_state_initializing(void);
void set_state_idle(void);
void set_state_running(void);
void set_state_discharging(void);
void set_state_reset(void);

void acumulate_measurements(void);
void calculate_measurements(void);

// ISRs
ISR(TIMER2_COMPA_vect);
ISR(PCINT2_vect);

#endif /* ifndef MACHINE_H */

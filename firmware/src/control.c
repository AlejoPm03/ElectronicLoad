#include "control.h"

volatile float dt, dt_max, vi, vo, io, io_max, vo_setpoint, io_setpoint;
const float dt_min = 0.3;
volatile union control_flags_t{                                                  
    struct{                                                                      
        uint8_t enable : 1;                                                                 
        uint8_t vi_safe_range : 1;                                               
        uint8_t vo_safe_range: 1;                                                
        uint8_t vi_stable : 1;                                                   
        uint8_t dt_safe_range : 1;                                               
    };                                                                           
    uint8_t all;                                                                 
}control_flags;             

void control_init(void)
{
    TCCR1A = (0<<COM1A1) | (0<<COM1A0) 
        | (1<<COM1B1) | (0<<COM1B0) 
        | (0<<WGM11) | (0<<WGM10);
    TCCR1B = (0<<CS12) | (0<<CS11) | (1<<CS10) 
        | (1<<WGM13) | (0<<WGM12);

    ICR1 = 260;
    OCR1B = 0;

    set_bit(PWM_ENABLE_DDR, PWM_ENABLE);
    clr_bit(PWM_ENABLE_PORT, PWM_ENABLE);
    set_bit(PWM_DDR, PWM);
}


/*          PI CONTROL ALGORITHM - SERIES IMPLEMENTATION
** /desc    Algoritimo para Controlador Proporcional Integrativo Diferencial.
**  ref1: https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/902/PI-controller-equations.pdf
** /var r é o valor desejado para a saída, o 'set-point'.
** /var y é o valor da saída.
** /ret     retorna a ação de controle u.
*/

inline float piIo(float r, float y){
    // PI CONFIGURATIONS:
    const float Kp = 0.3;         // analog series proportional gain
    const float Ti = 0.01;         // analog series integration period
    const float Ts = PERIOD;        // digital sampling period

    // INTERNAL CONSTANTS COMPUTATION:
    const float a0 = -Kp;           // IIR coefficient for old sample
    const float a1 = Kp*(1+Ts/Ti);  // IIR coefficient for new sample

    // CONTROLLER STATIC VARIABLES
    static float e0 = 0;            // old error
    static float e1 = 0;            // new error
    static float u = 0;             // control action

    // Compute error:
    e0 = e1;
    e1 = r -y;

    // Compute control action:
    u += + a1*e1 + a0*e0;

    // Anti windup
    if(u < D_MIN)           u = D_MIN;
    else if(u > D_MAX)      u = D_MAX;

    return u;
}

inline void control(void){
    static float vi_old = 0;
    static uint16_t vi_stable_counter = 0;
        
    // call feedback controller   
    control_feedback();
    // apply outputs
    OCR1B = ICR1 * dt;    
}

inline void control_feedback(void)
{
    io_setpoint = IO_SETPOINT;
    
    // CURRENT CONTROL as inner loop
    // soft start -> if(io_max < IO_MAX) io_max += 0.01;
    //if(io_max > IO_MAX) io_max = IO_MAX;
    //if(io_setpoint > io_max) io_setpoint = io_max;
    //if(io_setpoint > IO_MAX) io_setpoint = IO_MAX;
    dt = piIo(io_setpoint, io);
    if(dt < dt_min) dt = dt_min;
}


#include "control.h"


struct StateVariable new_variable(long double root) {
    struct StateVariable *temp;
    temp->root = root;
    return *temp;
}


PIDControl_t new_pid_controller(PIDControl_t temp , long double kp, long double ki, long double kd) {
    
    temp->Kp = kp;
    temp->Ki = ki;
    temp->Kd = kd;

    temp->integral_term = 0;
    temp->state = 0;
    temp->setpoint = 0;

    return temp;
}

PIDControl_t set_setpoint(PIDControl_t pid, long double setpoint) {
    pid->setpoint = setpoint;
    pid->error = pid->setpoint - pid->state;

    return pid;
}

PIDControl_t set_state(PIDControl_t pid, long double state) {
    pid->prev_state = pid->state;
    pid->state = state;
    pid->error = pid->setpoint - pid->state;

    return pid;
}

PIDControl_t set_limits(PIDControl_t pid, long double min, long double max) {
    pid->effort_min = min;
    pid->effort_max = max;

    return pid;
}

PIDControl_t limit(PIDControl_t pid) {
    if (pid->effort > pid->effort_max) {
        pid->effort = pid->effort_max; 
    } else if (pid-> effort < pid->effort_min) {
        pid->effort = pid->effort_min;
    }

    return pid;
}

PIDControl_t pid_compute(PIDControl_t pid) {
    pid->error = pid->setpoint - pid->state;
    pid->integral_term += pid->Ki * pid->error;

    if (pid->integral_term > pid->effort_max) {
        pid->integral_term = pid->effort_max;
    } else if (pid->integral_term < pid->effort_min) {
        pid->integral_term = pid->effort_min;
    }

    long double d_state = pid->state - pid->prev_state;
    pid->effort = pid->Kp*pid->error + pid->integral_term - pid->Kd*d_state;

    limit(&pid);
    return pid;
}

long double get_effort(PIDControl_t pid) {
    return pid->effort;
}
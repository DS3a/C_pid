#include "control.h"

struct PIDControl new_pid_controller(long double kp, long double ki, long double kd) {
    struct PIDControl *temp;
    temp->Kp = kp;
    temp->Ki = ki;
    temp->Kd = kd;

    temp->integral_term = 0;
    temp->state = 0;
    temp->setpoint = 0;

    return *temp;
}

void set_setpoint(struct PIDControl *pid, long double setpoint) {
    pid->setpoint = setpoint;
    pid->error = pid->setpoint - pid->state;
}

void set_state(struct PIDControl *pid, long double state) {
    pid->prev_state = pid->state;
    pid->state = state;
    pid->error = pid->setpoint - pid->state;
}

void set_limits(struct PIDControl *pid, long double min, long double max) {
    pid->effort_min = min;
    pid->effort_max = max;
}

void limit(struct PIDControl *pid) {
    if (pid->effort > pid->effort_max) {
        pid->effort = pid->effort_max; 
    } else if (pid-> effort < pid->effort_min) {
        pid->effort = pid->effort_min;
    }
}

long double pid_compute(struct PIDControl *pid) {
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
    return pid->effort;
}
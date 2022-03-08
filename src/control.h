#ifndef CONTROL_H
#define CONTROL_H


/*

    THE STATE VARIABLE LIB

*/

struct StateVariable {
    long double root, first_derivative, second_derivative;
};

struct StateVariable new_variable(long double root);


/*

    THE PID CONTROL LIB

*/
struct PIDControl {
    long double Kp, Ki, Kd;
    long double error;
    long double state, setpoint, effort, prev_state;
    long double effort_min, effort_max;
    long double integral_term;
};


// initialize the Kp Ki Kd values and return the struct instance
struct PIDControl new_pid_controller(long double kp, long double ki, long double kd);


// function to set the setpoint (goalpoint)
void set_setpoint(struct PIDControl *pid, long double setpoint);


// function to set the current state of the variable
void set_state(struct PIDControl *pid, long double state);


// function to set the limits of the pid controller
void set_limits(struct PIDControl *pid, long double min, long double max);


// function to limit the controller effort
void limit(struct PIDControl *pid);


// compute the pid limit
long double pid_compute(struct PIDControl *pid);
#endif
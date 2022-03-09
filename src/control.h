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


typedef struct PIDControl *PIDControl_t;

// initialize the Kp Ki Kd values and return the struct instance
// void new_pid_controller(PIDControl_t temp, long double kp, long double ki, long double kd);

#ifdef	__cplusplus
extern "C" {
#endif

    // constructor, kinda
    PIDControl_t new_pid_controller(PIDControl_t temp , long double kp, long double ki, long double kd);

    // function to set the setpoint (goalpoint)
    PIDControl_t set_setpoint(PIDControl_t pid, long double setpoint);


    // function to set the current state of the variable
    PIDControl_t set_state(PIDControl_t pid, long double state);


    // function to set the limits of the pid controller
    PIDControl_t set_limits(PIDControl_t pid, long double min, long double max);


    // function to limit the controller effort
    PIDControl_t limit(PIDControl_t pid);


    // compute the pid effort
    PIDControl_t pid_compute(PIDControl_t pid);


    // get the pid effort
    long double get_effort(PIDControl_t pid);

#ifdef	__cplusplus
}
#endif
#endif
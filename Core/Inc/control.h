// control.h
#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include "motor.h"
#include "max22200_driver.h"

// Enumeration for the control states
typedef enum {
    CONTROL_INIT,           // State for initializing the PID controller
    CONTROL_COMPUTE_PID,    // State for computing the PID output
    CONTROL_IMPEDANCE,      // State for computing the impedance
    CONTROL_PROCESS_DATA,   // State for processing the data
    CONTROL_SET_MOTOR_SPEED // State for setting the motor speed
} Control_State;

// Structure for PID control
typedef struct {
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float kv;               // Velocity gain
    float kp_imp;           // Proportional gain for impedance control
    float kd_imp;           // Derivative gain for impedance control
    double integral;        // Integral term
    double previous_error;  // Previous error term
    double position_error;  // Current position error
    int32_t setpoint;       // Desired position
    double setvel;          // Desired velocity
    double max_control_signal; // Maximum control signal
    double min_control_signal; // Minimum control signal
    int32_t current_position;  // Current position
    int32_t previous_position; // Previous position
    Control_State state;
} pid_control;

// Function prototypes
void control_init(pid_control* self, float kp, float ki, float kd,float kv, int32_t setpoint, double setvel,Control_State state);
double compute_pid(pid_control* self, int32_t current_position);
double compute_Impedance(pid_control* self, int32_t current_position);
void set_desired_position(pid_control* self, int32_t position);
void set_current_position(pid_control* self, int32_t position);
void set_desired_velocity(pid_control* self, double velocity);
double process_data(pid_control* pid, double pos) ;
int set_motor_speed(pid_control* self,float control_signal,int mode,int motor);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void pid_step(MAX22200_StatusReg* stat_reg,pid_control* pids[],Motor* motors[]);
void impedance_step(MAX22200_StatusReg* stat_reg,pid_control* pids[],Motor* motors[]);
void move_motor()

// External variables
extern uint32_t counter[3];
extern const double PI;
extern int pos[3];
extern double outputs[3];
extern int pwm_outputs[3];

#endif /* CONTROL_H_ */

// control.c
#include "control.h"
#define ticks_per_rev 4096
#define MAX_PWM 100 // Max PWM value
#define MIN_PWM -100 // Min PWM value
const double M_PI = 3.1415926535897932384626433832795028841971693993751058209;
#define MIN_CONTROL_SIGNAL -200000
#define MAX_CONTROL_SIGNAL 200000
#define FRICTION 0
extern MAX22200_status stats_;

int PWM_ZERO;
uint8_t out[3]={0,0,0};
float friction_compensation = 0.0;

// Initialize a PID controller
void control_init(pid_control* self, float kp, float ki, float kd,float kv, int32_t setpoint, double setvel, Control_State state) {
    // Set the PID parameters
    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->kv = kv;
    self->kd_imp=0;
    self->kp_imp=0.01;
    self->integral = 0;
    self->previous_error = 0;
    self->position_error=0;
    self->setpoint = setpoint;
    self->setvel = setvel;
    self->max_control_signal=self->kp * 100000 + self->ki * 100000;
    self->min_control_signal=-self->max_control_signal;
}

// Compute the control output
double compute_pid(pid_control* self, int32_t current_position) {
    // Compute the position error
    self->position_error = self->setpoint - current_position;

    // Compute the integral term
    self->integral += self->position_error;
    if(self->integral>100000)
    {
    	self->integral=100000;
    }
    else if(self->integral<-100000)
    {
    	self->integral=-100000;
    }

    // Compute the derivative term
    double derivative = self->position_error - self->previous_error;

    // Compute the control signal
    double output = self->kp * self->position_error + self->ki * self->integral + self->kd * derivative ;

    // Store the current position error for the next iteration
    self->previous_error = self->position_error;

    return output;
}

// Compute the impedance
double compute_impedance(pid_control* self, int32_t current_position) {
    // Set the damping coefficient and stiffness
    double damping_coefficient = self->kd_imp; // B
    double stiffness = self->kp_imp; // K

    // Calculate the displacement (current_position - setpoint)
    double displacement = -1*(double)(current_position - self->setpoint);

    // Calculate the velocity (current_position - previous_position)
    double velocity = (double)(current_position - self->previous_position);

    // Store current_position as previous_position for next iteration
    self->previous_position = current_position;

    // Calculate the force (or torque in rotational systems)
    double output = damping_coefficient * (velocity-self->setvel) + stiffness * displacement;

    return output;
}

// Set the desired position
void set_desired_position(pid_control* self, int32_t position) {
    self->setpoint = position;
}

// Set the current position
void set_current_position(pid_control* self, int32_t position) {
    self->current_position = position;
}

// Set the desired velocity
void set_desired_velocity(pid_control* self, double velocity) {
    self->setvel = velocity;
}

// Process the data
double process_data(pid_control* pid, double pos) {
	pid->state=CONTROL_PROCESS_DATA;

    // Set the desired position for the PID controller
	uint32_t pos_ticks=pos* (ticks_per_rev / (2 * M_PI));
    set_desired_position(pid, pos_ticks);

    // Update the PID controller
    return compute_pid(pid, pid->current_position);
}

// Set the motor speed
int set_motor_speed(pid_control* self,float control_signal,int mode,int motor)
{
	self->state=CONTROL_SET_MOTOR_SPEED;
	// friction compensation for
	if (control_signal > 0)
		friction_compensation = FRICTION; // Change this to the friction value in positive direction
	else if (control_signal < 0)
		friction_compensation = -FRICTION; // Change this to the friction value in negative direction
	if(mode)
	{
		self->max_control_signal = self->kp_imp * 100000 ;//+ self->ki * 100000;
		    self->min_control_signal = -self->max_control_signal;
		    PWM_ZERO=0;
	}
	else{
		  self->max_control_signal = self->kp * 100000 + self->ki * 100000;
		    self->min_control_signal = -self->max_control_signal;
		    PWM_ZERO=20;

	}

    // Clamp the control signal to the min and max values
    if (control_signal < self->min_control_signal) {
        control_signal = self->min_control_signal;
    } else if (control_signal > self->max_control_signal) {
        control_signal = self->max_control_signal;
    }

    // Map the control signal to a PWM value
      float pwm_value;
      if (control_signal > 0) {
          pwm_value = mapfloat(control_signal, 0, self->max_control_signal, PWM_ZERO, MAX_PWM);
      } else {
          pwm_value = mapfloat(control_signal, self->min_control_signal, 0, MIN_PWM, -PWM_ZERO);
      }
      if((pwm_value>1 || pwm_value <-1) && motor!=0)
      {
    	  pwm_value += friction_compensation;
		  PWM_ZERO += friction_compensation;
      }

    // Ensure PWM value stays within bounds
    if (pwm_value < MIN_PWM) {
        pwm_value = MIN_PWM;
    } else if (pwm_value > MAX_PWM) {
        pwm_value = MAX_PWM;
    }

    // Set the PWM duty cycle to control the motor speed
    return (int) pwm_value;
}

// Map a float value from one range to another
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Perform a step of the PID control
void pid_step(MAX22200_StatusReg* stat_reg,pid_control* pids[],Motor* motors[])
{

    // Loop over all motors
	for(int i=0;i<3;i++)
		    {
				pids[i]->state=CONTROL_COMPUTE_PID;
		        // Set the current position
		        set_current_position(pids[i],counter[i]);

		        // Process the data and set the motor speed
		        outputs[i]=process_data(pids[i], pos[i]*PI);
		        pwm_outputs[i]=set_motor_speed(pids[i],(float)outputs[i],0,i);

		        // Check the motor position and set the direction accordingly
		        if(pwm_outputs[i]>PWM_ZERO)
		        {
		            // Check if the motor is at or above its max position
		            if ((i == 0 && counter[i] >= 97000) || (i != 0 && counter[i] >= 109000))
		            {
		                // Motor is at or above its max position, so we shouldn't try to move it further
		                continue;
		            }

		            // Set the motor direction to UP
		            out[i]=pwm_outputs[i];
		            Set_HIT(&motors[i]->cfg1_, out[i]);
		            Set_HIT(&motors[i]->cfg2_, out[i]);
		            Set_HOLD(&motors[i]->cfg1_, out[i]);
		            Set_HOLD(&motors[i]->cfg2_, out[i]);
		            Motor_writeCfgRegister(&motors[i]->cfg1_, motors[i]->channel1_);
		            Motor_writeCfgRegister(&motors[i]->cfg2_, motors[i]->channel2_);
		            Motor_setDirection(i+1, UP,stat_reg);
		        }
		        else if (pwm_outputs[i]<-PWM_ZERO)
		        {
		            // Set the motor direction to DOWN
		            out[i]=-pwm_outputs[i];
		            Set_HIT(&motors[i]->cfg1_, out[i]);
		            Set_HIT(&motors[i]->cfg2_, out[i]);
		            Set_HOLD(&motors[i]->cfg1_, out[i]);
		            Set_HOLD(&motors[i]->cfg2_, out[i]);
		            Motor_writeCfgRegister(&motors[i]->cfg1_, motors[i]->channel1_);
		            Motor_writeCfgRegister(&motors[i]->cfg2_, motors[i]->channel2_);
		            Motor_setDirection(i+1, DOWN,stat_reg);
		        }
		        else
		        {
		            // Set the motor direction to BREAK
		            Motor_setDirection(i+1, BREAK,stat_reg);
		        }
		    }
}

// Perform a step of the impedance control
void impedance_step(MAX22200_StatusReg* stat_reg,pid_control* pids[],Motor* motors[])
{
    // Loop over all motors
	for(int i=0;i<3;i++)
	    {
			pids[i]->state=CONTROL_IMPEDANCE;

	        // Set the current position and desired position
	        set_current_position(pids[i],counter[i]);
	        uint32_t pos_ticks=pos[i]*(ticks_per_rev / (2 * M_PI));
			set_desired_position(pids[i], pos_ticks);

			// Compute the impedance and set the motor speed
		   outputs[i]=compute_impedance(pids[i], pids[i]->current_position);
		   pwm_outputs[i]=set_motor_speed(pids[i],(float)outputs[i],1,i);

	        // Check the motor position and set the direction accordingly
	        if(pwm_outputs[i]>PWM_ZERO)
	        {
	            // Check if the motor is at or above its max position
	            if ((i == 0 && counter[i] >= 97000) || (i != 0 && counter[i] >= 109000))
	            {
	                // Motor is at or above its max position, so we shouldn't try to move it further
	                continue;
	            }

	            // Set the motor direction to UP
	            out[i]=pwm_outputs[i];
	            Set_HIT(&motors[i]->cfg1_, out[i]);
	            Set_HIT(&motors[i]->cfg2_, out[i]);
	            Set_HOLD(&motors[i]->cfg1_, out[i]);
	            Set_HOLD(&motors[i]->cfg2_, out[i]);
	            Motor_writeCfgRegister(&motors[i]->cfg1_, motors[i]->channel1_);
	            Motor_writeCfgRegister(&motors[i]->cfg2_, motors[i]->channel2_);
	            Motor_setDirection(i+1, UP,stat_reg);
	        }
	        else if (pwm_outputs[i]<-PWM_ZERO)
	        {
	            // Set the motor direction to DOWN
	            out[i]=-pwm_outputs[i];
	            Set_HIT(&motors[i]->cfg1_, out[i]);
	            Set_HIT(&motors[i]->cfg2_, out[i]);
	            Set_HOLD(&motors[i]->cfg1_, out[i]);
	            Set_HOLD(&motors[i]->cfg2_, out[i]);
	            Motor_writeCfgRegister(&motors[i]->cfg1_, motors[i]->channel1_);
	            Motor_writeCfgRegister(&motors[i]->cfg2_, motors[i]->channel2_);
	            Motor_setDirection(i+1, DOWN,stat_reg);
	        }
	        else
	        {
	            // Set the motor direction to BREAK
//	            Motor_setDirection(i+1, BREAK,stat_reg);
	        }
	    }
}

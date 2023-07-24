/*
 * fsm.c
 *
 *  Created on: Jun 13, 2023
 *      Author: nir
 *
 *  This codebase is for a Finite State Machine (FSM). It deals with
 *  initializing various components, executing commands, switching between states,
 *  and handling errors.
 */

#include "fsm.h"
#include "fdcan.h"

/* Global variable declarations */
extern MAX22200_StatusReg status_Reg;//status register for the max22200
extern MAX22200_status stats_;//status structs for read/write operation on the max22200
extern pid_control pid1;//pid struct for motor1(back)
extern pid_control pid2;//pid struct for motor2(front right)
extern pid_control pid3;//pid struct for motor2(front left)
extern pid_control* pids[3];//pointer to an array of pid struct for multiple motor pid operations
extern Encoder encoders[3];//encoders array for multiple encoder readings operations - mainly used in pid
extern CFG_CH_Register regs[6];//config register structs array, [0,1] for motor 1, [2,3] for motor 2 , [4,5] for motor 3
extern Motor m1;//motor 1 struct consists of 2 channels-0,1, cfg registers for both channels
extern Motor m2;//motor 2 struct consists of 2 channels-2,3, cfg registers for both channels
extern Motor m3;//motor 3 struct consists of 2 channels-4,5, cfg registers for both channels
extern Motor* motors[3];//pointer array for all the motors for multiple motor operations
extern FSMStruct state;//struct for managing the fsm state
extern float kp,kd,ki;//pid constants, for debuging mostly
extern bool watchdog1,watchdog2,watchdog3;//watchdog for each motor, not implemented well yet.

int count_init=0; // To count the number of initializations
FSMErrorStatus errorStatus = {0};  // Initialize error status to 0

/**
 * This function initializes the FSM.
 * It sets up the states, initializes the FDCAN, timers, encoders, motors and control PIDs.
 * If any initialization fails, it updates the FSM to a fault state.
 */
void init_fsm()
{
    /* Set the initial state */
    state.state = INIT_MODE;//currently in INIT Mode
    state.next_state = MOTOR_MODE;//next mode is motor control ie PID/Impedance
    state.ready = 0;//not ready to move to next mode

    /* Set up the FDCAN */
    fdcan_init(&hfdcan2);
//    set_fingerAddress();


    // Start FDCAN and handle possible failure
    if (HAL_FDCAN_Start(&hfdcan2)!= HAL_OK){
        errorStatus.fdcanCommFailure = 1;
        update_fsm(&state, FAULT_CMD);
        return;
    }

    /* Activate FDCAN notifications and handle possible failure */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        errorStatus.fdcanCommFailure = 1;
        update_fsm(&state, FAULT_CMD);
        return;
    }

    /* Start the watchdog timer and handle possible failure */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK){
        errorStatus.timerFailure = 1;
        update_fsm(&state, FAULT_CMD);
        return;
    }

    /* Start the encoder timers and handle possible failure */
    if(HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL)!=HAL_OK ||
       HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL)!=HAL_OK ||
       HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL)!=HAL_OK) {
        errorStatus.timerFailure = 1;
        update_fsm(&state, FAULT_CMD);
        return;
    }

    /* Initialize encoders */
    encoder_init(&encoders[0], &htim1, 1);
    encoder_init(&encoders[1], &htim2, 1);
    encoder_init(&encoders[2], &htim3, 1);

    /* Initialize MAX22200 status register and MAX22200 */
    MAX22200_init_statreg(&stats_, &status_Reg);
    MAX22200_init(&stats_);

    /* Initialize Motors */
    MAX22000_init_cfg_reg(&regs[0], &regs[1]);
    MAX22000_init_cfg_reg(&regs[2], &regs[3]);
    MAX22000_init_cfg_reg(&regs[4], &regs[5]);
    Motor_init(&m1,&regs[0],&regs[1],1);
    Motor_init(&m2,&regs[2],&regs[3],2);
    Motor_init(&m3,&regs[4],&regs[5],3);

    /* Calibrate motors and handle possible failure */
    calibration(&status_Reg, motors);
    if(!calibrated){
        errorStatus.calibrationFailure = 1;
        update_fsm(&state, FAULT_CMD);
        return;
    }

    /* Initialize control PID */
    control_init(&pid1,kp,ki,kd,0,0,0,CONTROL_INIT);
    pids[0]=&pid1;
    control_init(&pid2,kp,ki,kd,0,0,0,CONTROL_INIT);
    pids[1]=&pid2;
    control_init(&pid3,kp,ki,kd,0,0,0,CONTROL_INIT);
    pids[2]=&pid3;

    /* Increment initialization count and set the FSM to ready */
    count_init++;
    state.ready = 1;
}
// Finite State Machine (FSM) Runner
// This function is run every timer6 interrupt cycle
void run_fsm(FSMStruct * fsmstate)
{
    // State transition management
    // Safely exit the old state if the next state is different
    if(fsmstate->next_state != fsmstate->state)
    {
        fsm_exit_state(fsmstate);

        // If the previous state is ready, enter the new state
        if(fsmstate->ready)
        {
            fsmstate->state = fsmstate->next_state;
            fsm_enter_state(fsmstate);
        }
    }

    // Process tasks based on the current state
    switch(fsmstate->state)
    {
        case INIT_MODE:
            if(count_init==0)
            {
                init_fsm();
            }
            break;

        case MENU_MODE:
            // menu();
            break;

        case CALIBRATION_MODE:
            calibration(&status_Reg, motors);
            break;

        case MOTOR_MODE:
            watchdog();
            controller_faults();

            // pid_step(&status_Reg, pids, motors);
            impedance_step(&status_Reg, pids, motors);
            break;

        case SETUP_MODE:
            // setup();
            break;

        case ENCODER_MODE:
            // read_encoder();
            break;

        case FAULT_MODE:
            Errors_Handler(errorStatus, &state);
            break;

        case WATCHDOG_MODE:
            // Handle watchdog mode
            break;
    }
}

// FSM State Enter
// Called when entering a new state. Do necessary setup.
void fsm_enter_state(FSMStruct * fsmstate)
{
    switch(fsmstate->state)
    {
        case MENU_MODE:
            // Setup for MENU_MODE
            break;

        case SETUP_MODE:
            // Setup for SETUP_MODE
            break;

        case ENCODER_MODE:
            // Setup for ENCODER_MODE
            break;

        case MOTOR_MODE:
            // Setup for MOTOR_MODE
            break;

        case CALIBRATION_MODE:
            // Setup for CALIBRATION_MODE
            break;

        case FAULT_MODE:
            // Setup for FAULT_MODE
            break;

        case WATCHDOG_MODE:
            // Setup for WATCHDOG_MODE
            break;
    }
}

// FSM State Exit
// Called when exiting the current state. Do necessary cleanup.
void fsm_exit_state(FSMStruct * fsmstate)
{
    switch(fsmstate->state)
    {
        case INIT_MODE:
            fsmstate->ready = 1;
            break;

        case MENU_MODE:
            fsmstate->ready = 1;
            break;

        case SETUP_MODE:
            fsmstate->ready = 1;
            break;

        case ENCODER_MODE:
            fsmstate->ready = 1;
            break;

        case MOTOR_MODE:
            fsmstate->ready = 1;
            break;

        case CALIBRATION_MODE:
            fsmstate->ready = 1;
            break;

        case FAULT_MODE:
            fsmstate->ready = 1;
            break;

        case WATCHDOG_MODE:
            fsmstate->ready = 1;
            break;
    }
}

// FSM State Update
// Only run when new state-change information is received on FDCAN input.
void update_fsm(FSMStruct * fsmstate, int fsm_input)
{
    // If MENU_CMD is received, set next state to MENU_MODE and not ready
    if(fsm_input == MENU_CMD)
    {
        fsmstate->next_state = MENU_MODE;
        fsmstate->ready = 0;
        return;
    }

    // Process state change command based on the current state
    switch(fsmstate->state)
    {
        case INIT_MODE:
            switch (fsm_input)
            {
                case MOTOR_CMD:
                    fsmstate->next_state = MOTOR_MODE;
                    fsmstate->ready = 0;
                    break;

                case FAULT_CMD:
                    fsmstate->next_state = INIT_MODE;
                    fsmstate->ready = 0;
                    break;
            }
            break;

        case MENU_MODE:
            switch (fsm_input)
            {
                case CALIBRATION_CMD:
                    fsmstate->next_state = CALIBRATION_MODE;
                    fsmstate->ready = 0;
                    break;

                case MOTOR_CMD:
                    fsmstate->next_state = MOTOR_MODE;
                    fsmstate->ready = 0;
                    break;

                case ENCODER_CMD:
                    fsmstate->next_state = ENCODER_MODE;
                    fsmstate->ready = 0;
                    break;

                case SETUP_CMD:
                    fsmstate->next_state = SETUP_MODE;
                    fsmstate->ready = 0;
                    break;

                case ZERO_CMD:
                    // Handle ZERO_CMD
                    break;

                case FAULT_CMD:
                    // Handle FAULT_CMD
                    break;

                case WATCHDOG_CMD:
                    // Handle WATCHDOG_CMD
                    break;
            }
            break;

        case SETUP_MODE:
            // Handle state change in SETUP_MODE
            break;

        case ENCODER_MODE:
            // Handle state change in ENCODER_MODE
            break;

        case MOTOR_MODE:
            // Handle state change in MOTOR_MODE
            break;
    }
}

 // Controller Faults Function
 // This function checks if the MAX controller has encountered any faults
 void controller_faults(void)
 {
     // Reads fault pin from MAX controller
     // If pin is reset (i.e., GPIO_PIN_RESET), there's a controller fault
     if(HAL_GPIO_ReadPin(MAX_FAULT_GPIO_Port, MAX_FAULT_Pin) == GPIO_PIN_RESET)
     {
         // Update error status to signify controller fault
         errorStatus.controllerFault = 1;

         // Update FSM to FAULT_MODE
         update_fsm(&state, FAULT_MODE);
     }
 }

 // Watchdog Function
 // This function acts as a watchdog timer for three motors
 void watchdog()
 {
     // Checks if motors are calibrated
     if(calibrated)
     {
         // If the counter for motor 1 exceeds 97000
         // It sets the motor direction to BREAK and updates the FSM to FAULT_MODE
         if(counter[0]>97000)
         {
             Motor_setDirection(1, BREAK,&status_Reg);
             watchdog1=true;
             errorStatus.watchdogTimeout = 1;
             update_fsm(&state, FAULT_MODE);
         }
         // Similarly for motor 2 and counter > 109000
         if(counter[1]>109000)
         {
             Motor_setDirection(2, BREAK,&status_Reg);
             watchdog2=true;
             errorStatus.watchdogTimeout = 1;
             update_fsm(&state, FAULT_MODE);
         }
         // Similarly for motor 3 and counter > 109000
         if(counter[2]>109000)
         {
             Motor_setDirection(3, BREAK,&status_Reg);
             watchdog3=true;
             errorStatus.watchdogTimeout = 1;
             update_fsm(&state, FAULT_MODE);
         }
     }
 }

 // Error Handler
 // This function handles errors based on the FSMErrorStatus structure
 void Errors_Handler(FSMErrorStatus errorStatus, FSMStruct* fsmstate)
 {
     // Checks if there's an initialization failure
     if(errorStatus.initFailure)
     {
         // Handle initialization failure - e.g., log, notify user, attempt recovery
         update_fsm(&state, INIT_CMD);
     }

     // Checks if there's an FDCAN communication failure
     if(errorStatus.fdcanCommFailure)
     {
         // Handle FDCAN communication errors - e.g., log, notify user, attempt recovery
     }

     // Checks if there's a timer failure
     if(errorStatus.timerFailure)
     {
         // Handle Timer errors - e.g., log, notify user, attempt recovery
     }

     // Checks if there's a calibration failure
     if(errorStatus.calibrationFailure)
     {
         // Reset calibration counters and start calibration again
         counter_calibration[0]=0;
         counter_calibration[1]=0;
         counter_calibration[2]=0;
         calibration(&status_Reg, motors);
         // Handle calibration failure - e.g., log, notify user, attempt recovery
     }

     // Checks if there's a controller fault
     if(errorStatus.controllerFault)
     {
         // Reads the status and fault registers from the MAX controller
         stats_.result=MAX22200_read_register(MAX22200_STATUS, &stats_.status);
         stats_.result=MAX22200_read_register(MAX22200_FAULT, &stats_.status);
         // Handle controller fault - e.g., log, notify user, attempt recovery
     }

     // Checks if there's a watchdog timeout
     if(errorStatus.watchdogTimeout)
     {
         // Handle watchdog timeout - e.g., log, notify user, attempt recovery
     }
 }



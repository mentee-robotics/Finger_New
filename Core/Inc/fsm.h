#ifndef INC_FSM_H_
#define INC_FSM_H_

// Include necessary libraries and header files
#include "stdint.h"
#include "stm32g4xx_hal.h"
#include "calibration.h"
#include "motor.h"
#include "control.h"
#include "max22200_driver.h"
#include "encoder.h"
#include "fdcan.h"
#include "tim.h"

// Define enumeration for FSM modes
typedef enum {
    INIT_MODE,          // Initialization mode
    MENU_MODE,          // Menu mode
    CALIBRATION_MODE,   // Calibration mode
    MOTOR_MODE,         // Motor mode
    SETUP_MODE,         // Setup mode
    ENCODER_MODE,       // Encoder mode
    FAULT_MODE,         // Fault mode
    WATCHDOG_MODE,      // Watchdog mode
}FSM_MODE;

// Define enumeration for FSM commands
typedef enum {
    INIT_CMD,           // Initialization command
    MENU_CMD,           // Menu command
    CALIBRATION_CMD,    // Calibration command
    MOTOR_CMD,          // Motor command
    SETUP_CMD,          // Setup command
    ENCODER_CMD,        // Encoder command
    FAULT_CMD,          // Fault command
    WATCHDOG_CMD,       // Watchdog command
    ZERO_CMD,           // Zero command
}FSM_CMD;

// Define the FSM state structure
typedef struct {
    uint8_t state;             // Current state
    uint8_t next_state;        // Next state
    uint8_t state_change;      // Flag to indicate a state change
    uint8_t ready;             // Flag to indicate readiness for transition
    char cmd_buff[8];          // Command buffer
    char cmd_id;               // Command identifier
}FSMStruct;

// Define error status structure
typedef struct {
    uint8_t initFailure : 1;       // Initialization failure flag
    uint8_t timerFailure : 1;      // Timer failure flag
    uint8_t fdcanCommFailure : 1;  // FDCAN communication failure flag
    uint8_t calibrationFailure : 1;// Calibration failure flag
    uint8_t controllerFault : 1;   // Controller fault flag
    uint8_t watchdogTimeout : 1;   // Watchdog timeout flag
} FSMErrorStatus;

// Function prototypes
void init_fsm();
void run_fsm(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, int fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_pid_mode(void);
void process_user_input(FSMStruct * fsmstate);
void controller_faults(void);
void watchdog();
void Errors_Handler(FSMErrorStatus errorStatus,FSMStruct* fsmstate);

#endif /* INC_FSM_H_ */

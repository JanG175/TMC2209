/**
 * @file AS5600.c
 * @author JanG175
 * @brief ESP IDF component for the TMC2209
 * 
 * @copyright Apache 2.0
 */

#include <stdio.h>
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define GCONF        0x00    // R/W    Global configuration flags
#define GSTAT        0x01    // R/W    (W clears) Global status flags
#define IFCNT        0x02    // R      Counter for write access
#define SLAVECONF    0x03    // W      Delay for read access
#define OTP_PROG     0x04    // W      Write access programs OTP memory
#define OTP_READ     0x05    // R      Access to OTP memory
#define IOIN         0x06    // R      Reads the state of all input pins
#define FACTORY_CONF 0x07    // R/W    FCLKTRIM and OTTRIM defaults
#define IHOLD_IRUN   0x10    // W      Driver current control
#define TPOWERDOWN   0x11    // W      Delay time to motor current power down
#define TSTEP        0x12    // R      Actual measured time between two microsteps
#define TPWMTHRS     0x13    // W      Upper velocity for StealthChop voltage mode 
#define VACTUAL      0x22    // W      Moving the motor by UART control.
#define MSCNT        0x6A    // R      Microstep counter
#define MSCURACT     0x6B    // R      Actual microstep current
#define CHOPCONF     0x6C    // R/W    Chopper and driver configuration
#define DRV_STATUS   0x6F    // R      Driver status flags and current level read
#define PWMCONF	     0x70    // R/W    StealthChop PWM chopper configuration
#define PWM_SCALE    0x71    // R      Results of StealthChop amplitude regulator
#define PWM_AUTO     0x72    // R      Generated values for PWM_GRAD/PWM_OFS

#define SYNC         0x05   // reversed: sync [1010] + reserved [0000]

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_ID TIMER_0

#define UART_TIMEOUT_MS (100 / portTICK_PERIOD_MS)

#define CW_DIR 0
#define CCW_DIR 1

#define FULL_ROT 200

typedef struct TMC2209_t
{
    uart_port_t uart;
    uint32_t baudrate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t* step_pin;
    gpio_num_t* dir_pin;
    gpio_num_t* en_pin;
} TMC2209_t;

typedef struct callback_arg_t
{
    gpio_num_t step_pin;
    uint32_t motor_num;
} callback_arg_t;


void TMC2209_init(TMC2209_t TMC);

void TMC2209_deinit(TMC2209_t TMC);

void TMC2209_enable(TMC2209_t TMC, uint32_t motor_num, uint32_t enable);

void TMC2209_set_dir(TMC2209_t TMC, uint32_t motor_num, uint32_t cw);

void TMC2209_set_period(uint32_t motor_num, uint32_t period_us);

void TMC2209_start(TMC2209_t TMC, uint32_t motor_num, uint32_t start);

void TMC2209_step_move(TMC2209_t TMC, int64_t* steps, uint32_t* period_us);

void TMC2209_uart_move(TMC2209_t TMC, uint8_t address, int32_t speed);

int32_t TMC2209_uart_get_position(TMC2209_t TMC, uint8_t address);

void TMC2209_uart_conf(TMC2209_t TMC, uint8_t address);

void TMC2209_uart_write_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg, int32_t data);

int32_t TMC2209_uart_read_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg);

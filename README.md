# TMC2209 library function list:
## init functions:
* void TMC2209_init(TMC2209_t TMC) - init TMC UART and timers
* void TMC2209_deinit(TMC2209_t TMC) - deinit TMC UART and timers
## step/dir mode:
* void TMC2209_enable(TMC2209_t TMC, uint32_t motor_num, uint32_t enable) - set enable pin
* void TMC2209_set_dir(TMC2209_t TMC, uint32_t motor_num, uint32_t cw) - set dir pin
* void TMC2209_set_period(uint32_t motor_num, uint32_t period_us) - set period for step signal
* void TMC2209_start(TMC2209_t TMC, uint32_t motor_num, uint32_t start) - start/stop step signal
* void TMC2209_step_move(TMC2209_t TMC, uint64_t* steps, uint32_t* period_us) - move all motors by desired number of steps with desired period and direction (sign in steps variable)
## UART mode:
* void TMC2209_uart_move(TMC2209_t TMC, uint8_t address, int32_t speed) - move continuously with desired speed
* int32_t TMC2209_uart_get_position(TMC2209_t TMC, uint8_t address) - get position (based on voltage phase)
* void TMC2209_uart_conf(TMC2209_t TMC, uint8_t address) - configure TMC2209 registers
* void TMC2209_uart_write_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg, int32_t data) - create write datagram
* int32_t TMC2209_uart_read_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg) - create read datagram
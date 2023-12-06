/**
 * @file AS5600.c
 * @author JanG175
 * @brief ESP IDF component for the TMC2209
 * 
 * @copyright Apache 2.0
 */

#include <stdio.h>
#include "TMC2209.h"

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static gptimer_handle_t gptimer[] = {NULL, NULL, NULL};
static int64_t steps_left[] = {0, 0, 0};

static const uint32_t timer_N = sizeof(gptimer) / sizeof(gptimer[0]); // number of timers

static const char* TAG = "TMC2209";


/**
 * @brief calculate CRC for TMC2209 UART communication
 * 
 * @param datagram pointer to datagram
 * @param datagramLength length of datagram
 * @return CRC
 */
static uint8_t TMC2209_uart_calc_CRC(uint8_t* datagram, uint8_t datagramLength)
{
    uint8_t* crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;

    // execute for all bytes of a message
    for (int32_t i = 0; i < (datagramLength - 1); i++)
    {
        currentByte = datagram[i]; // retrieve a byte to be sent from array
        for (int32_t j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }

            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte

    return *crc;
}


/**
 * @brief write datagram to register via UART
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void TMC2209_uart_write_register(TMC2209_t TMC, uint8_t* datagram, uint8_t len)
{
    uart_write_bytes(TMC.uart, (const uint8_t*)datagram, len);
    ESP_ERROR_CHECK(uart_wait_tx_done(TMC.uart, UART_TIMEOUT_MS));
}


/**
 * @brief read datagram from register via UART
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void TMC2209_uart_read_register(TMC2209_t TMC, uint8_t* datagram, uint8_t len)
{
    uint32_t buf = 0;
    uint8_t data[len];

    buf = uart_read_bytes(TMC.uart, data, len, UART_TIMEOUT_MS);
    uart_flush(TMC.uart);

    if (buf == len)
    {
        for (int i = 0; i < buf; i++)
        {
            datagram[i] = data[i];
        }
    }
    else
    {
        ESP_LOGE(TAG, "UART read error");

        for (int i = 0; i < len; i++)
            datagram[i] = 0;
    }
}


/**
 * @brief callback function for timers
 * 
 * @param timer timer handle
 * @param edata event data
 * @param user_ctx user context
 * @return true - if high priority task was woken up; false - otherwise
 */
static bool clk_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;

    callback_arg_t cb_arg = *(callback_arg_t*)user_ctx;
    gpio_num_t step_pin = cb_arg.step_pin;
    uint32_t motor_num = cb_arg.motor_num;

    if (steps_left[motor_num] > 0)
    {
        if (gpio_get_level(step_pin) == 0)
            gpio_set_level(step_pin, 1);
        else
            gpio_set_level(step_pin, 0);

        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num]--;
        portEXIT_CRITICAL(&spinlock);
    }
    else
        ESP_ERROR_CHECK(gptimer_stop(timer));

    return (high_task_awoken == pdTRUE);
}


/**
 * @brief initialize TMC2209 UART and timers
 * 
 * @param TMC struct with TMC2209 connection parameters
 */
void TMC2209_init(TMC2209_t TMC)
{
    // configure UART
    uart_config_t uart_config = {
        .baud_rate = TMC.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(TMC.uart, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TMC.uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TMC.uart, TMC.tx_pin, TMC.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    callback_arg_t* cb_arg = malloc(sizeof(callback_arg_t) * timer_N); // allocate memory for callback arguments

    for (uint32_t i = 0; i < timer_N; i++)
    {
        // configure step and dir pins

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pin_bit_mask = ((1 << TMC.step_pin[i]) | (1 << TMC.dir_pin[i]) | (1 << TMC.en_pin[i]));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        TMC2209_enable(TMC, i, 1);
        TMC2209_set_dir(TMC, i, CW_DIR);

        // configure timers

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000 // 1 us
        };

        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer[i]));

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000000, // 1 s
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true
        };

        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[i], &alarm_config));

        gptimer_event_callbacks_t timer_cbs = {
            .on_alarm = clk_timer_callback
        };

        cb_arg[i].step_pin = TMC.step_pin[i];
        cb_arg[i].motor_num = i;

        ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer[i], &timer_cbs, (void*)&cb_arg[i]));

        ESP_ERROR_CHECK(gptimer_enable(gptimer[i]));
        ESP_ERROR_CHECK(gptimer_stop(gptimer[i]));

        TMC2209_enable(TMC, i, 0); // enable motor
    }
}


// deinit TMC UART and timers
void TMC2209_deinit(TMC2209_t TMC)
{
    for (uint32_t i = 0; i < timer_N; i++)
    {
        ESP_ERROR_CHECK(gptimer_disable(gptimer[i]));
        ESP_ERROR_CHECK(gptimer_del_timer(gptimer[i]));
    }

    ESP_ERROR_CHECK(uart_driver_delete(TMC.uart));
}


/**
 * @brief set enable pin
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param motor_num motor number
 * @param enable 0 - enable, 1 - disable
 */
void TMC2209_enable(TMC2209_t TMC, uint32_t motor_num, uint32_t enable)
{
    if (enable == 0)
        gpio_set_level(TMC.en_pin[motor_num], 0);
    else if (enable == 1)
        gpio_set_level(TMC.en_pin[motor_num], 1);
}


/**
 * @brief set direction pin
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param motor_num motor number
 * @param dir direction (CW_DIR or CCW_DIR)
 */
void TMC2209_set_dir(TMC2209_t TMC, uint32_t motor_num, uint32_t dir)
{
    if (dir == CW_DIR)
        gpio_set_level(TMC.dir_pin[motor_num], 0);
    else if (dir == CCW_DIR)
        gpio_set_level(TMC.dir_pin[motor_num], 1);
}


/**
 * @brief set period for step signal
 * 
 * @param motor_num motor number
 * @param period_us period in us
 */
void TMC2209_set_period(uint32_t motor_num, uint32_t period_us)
{
    if (period_us < 200)
        ESP_LOGW(TAG, "period_us too small, motor might not work properly");

    period_us = period_us / 2; // 1 period = 2 gpio switches

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[motor_num], &alarm_config));
}


/**
 * @brief start/stop step signal
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param motor_num motor number
 * @param start 0 - stop, 1 - start
 */
void TMC2209_start(TMC2209_t TMC, uint32_t motor_num, uint32_t start)
{
    if (start == 0)
    {
        ESP_ERROR_CHECK(gptimer_stop(gptimer[motor_num]));
        gpio_set_level(TMC.step_pin[motor_num], 0);
    }
    else if (start == 1)
    {
        ESP_ERROR_CHECK(gptimer_start(gptimer[motor_num]));
    }
}


/**
 * @brief move all motors by desired number of steps with desired period and direction (sign in steps variable)
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param steps array of steps for each motor
 * @param period_us array of periods for each motor
 */
void TMC2209_step_move(TMC2209_t TMC, int64_t* steps, uint32_t* period_us)
{
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
    {
        TMC2209_start(TMC, motor_num, 0);

        // set direction
        if (steps[motor_num] < 0)
        {
            TMC2209_set_dir(TMC, motor_num, 0);
            steps[motor_num] = -steps[motor_num];
        }
        else
            TMC2209_set_dir(TMC, motor_num, 1);

        // set period
        TMC2209_set_period(motor_num, period_us[motor_num]);

        // set number of steps
        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num] = 2 * steps[motor_num];
        portEXIT_CRITICAL(&spinlock);
    }

    // start all motors one by one
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
        TMC2209_start(TMC, motor_num, 1);

    bool end_wait = false;

    // wait until all motors stop
    while (end_wait == false)
    {
        int64_t steps_left_status = 0;

        for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
            steps_left_status = steps_left_status + steps_left[motor_num];

        if (steps_left_status <= 0)
            end_wait = true;

        vTaskDelay(1);
    }

    // for debug
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
        ESP_LOGI(TAG, "%lu: %lld", motor_num, steps_left[motor_num] / 2);
}


/**
 * @brief move continuously with desired speed
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param address TMC2209 address
 * @param speed speed in % (from -100 to 100)
 */
void TMC2209_uart_move(TMC2209_t TMC, uint8_t address, int32_t speed)
{
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;

    speed = (int32_t)((float)speed / 100.0f * 8388607.0f); // 8388607 = 2^23 - 1 (max value for VACTUAL)

    TMC2209_uart_write_datagram(TMC, address, VACTUAL, speed);
}


/**
 * @brief get position (based on output voltage phase)
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param address TMC2209 address
 * @return position
 */
int32_t TMC2209_uart_get_position(TMC2209_t TMC, uint8_t address)
{
    int32_t pos = TMC2209_uart_read_datagram(TMC, address, MSCNT);

    return pos;
}


/**
 * @brief config TMC2209 registers
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param address TMC2209 address
 */
void TMC2209_uart_conf(TMC2209_t TMC, uint8_t address)
{
    int32_t gconf = TMC2209_uart_read_datagram(TMC, address, GCONF);

    // use microstep register for microstep setting
    int32_t mask = 1 << 7; // mstep_reg_select
    gconf = gconf | mask; // mstep_reg_select = 1

    TMC2209_uart_write_datagram(TMC, address, GCONF, gconf);

    int32_t chopconf = TMC2209_uart_read_datagram(TMC, address, CHOPCONF);
    printf("%ld\n", chopconf);

    // micro step resolution - FULLSTEP
    mask = 1 << 27; // mres3
    chopconf = chopconf | mask; // mres3 = 1
    mask = 1 << 26; // mres2
    chopconf = chopconf & ~mask; // mres2 = 0
    mask = 1 << 25; // mres1
    chopconf = chopconf & ~mask; // mres1 = 0
    mask = 1 << 24; // mres0
    chopconf = chopconf & ~mask; // mres0 = 0

    TMC2209_uart_write_datagram(TMC, address, CHOPCONF, chopconf);
}


/**
 * @brief create write datagram
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param address TMC2209 address
 * @param reg register address
 * @param data data to write
 */
void TMC2209_uart_write_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg, int32_t data)
{
    uint8_t len_w = 8;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    reg = reg | 0x80; // write bit

    uint8_t dataHH = (data >> 24) & 0xFF;
    uint8_t dataHL = (data >> 16) & 0xFF;
    uint8_t dataLH = (data >> 8) & 0xFF;
    uint8_t dataLL = data & 0xFF;

    datagram[0] = SYNC;                                   // sync + reserved
    datagram[1] = address;                                // 8 bit slave address (0-3)
    datagram[2] = reg;                                    // 7 bit register and write bit (1)
    datagram[3] = dataHH;                                 // data byte 3
    datagram[4] = dataHL;                                 // data byte 2
    datagram[5] = dataLH;                                 // data byte 1
    datagram[6] = dataLL;                                 // data byte 0
    datagram[7] = TMC2209_uart_calc_CRC(datagram, len_w); // CRC

    TMC2209_uart_write_register(TMC, datagram, len_w);

    uart_flush(TMC.uart);
}


/**
 * @brief create read datagram
 * 
 * @param TMC struct with TMC2209 connection parameters
 * @param address TMC2209 address
 * @param reg register address
 * @return data from register
 */
int32_t TMC2209_uart_read_datagram(TMC2209_t TMC, uint8_t address, uint8_t reg)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    uint8_t len_r = 8;
    uint8_t response[len_w + len_r];

    for (uint32_t i = 0; i < len_w + len_r; i++)
        response[i] = 0;

    datagram[0] = SYNC;                                   // sync + reserved
    datagram[1] = address;                                // 8 bit slave address (0-3)
    datagram[2] = reg;                                    // 7 bit register and read bit (0)
    datagram[3] = TMC2209_uart_calc_CRC(datagram, len_w); // CRC

    TMC2209_uart_write_register(TMC, datagram, len_w);

    TMC2209_uart_read_register(TMC, response, len_w + len_r);

    uint8_t dataHH = response[7];
    uint8_t dataHL = response[8];
    uint8_t dataLH = response[9];
    uint8_t dataLL = response[10];

    int32_t data = ((int32_t)dataHH << 24) + ((int32_t)dataHL << 16) + ((int32_t)dataLH << 8) + (int32_t)dataLL;

    return data;
}

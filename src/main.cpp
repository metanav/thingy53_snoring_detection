#include "ei_device_thingy53.h"
#include "ble/ei_ble_com.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "sensors/ei_microphone.h"
#include <drivers/uart.h>
#include <logging/log.h>
#include <nrfx_clock.h>
#include <zephyr.h>

#define LOG_MODULE_NAME main

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct device *uart;

void ei_putchar(char c)
{
    uart_fifo_fill(uart, (const uint8_t*)&c, 1);
}

int main(void)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());

    /* output of printf is output immediately without buffering */ 
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Switch CPU core clock to 128 MHz */
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

    /* Initialize board uart */
    uart = device_get_binding("CDC_ACM_0");
    if (!uart) {
        LOG_ERR("Failed to init CDC_ACM_0\n");
    }

    /* Setup the microphone */
    if (ei_microphone_init() == false) {
         LOG_ERR("Microphone init failed!");
    }

    // init BLE stack
    ei_ble_com_init();

    // we have to do this later because the BLE stack is not available on boot
    dev->init_device_id();

    dev->set_state(eiStateFinished);

    // continuous=true, debug=false
    ei_start_impulse(true, false);
    dev->set_serial_channel(UART);

    ei_printf("Inferencing\n");
    while(1) {
        if (!is_inference_running()) {
            ei_printf("Inferencing Stopped\n");
        } else {
            ei_printf("Inferencing\n");
        }
        ei_sleep(1000);
    }
}

K_THREAD_DEFINE(inference_thread_id, CONFIG_EI_INFERENCE_THREAD_STACK,
                ei_inference_thread, NULL, NULL, NULL,
                CONFIG_EI_INFERENCE_THREAD_PRIO, 0, 0);


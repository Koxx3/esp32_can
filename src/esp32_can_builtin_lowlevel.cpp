/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_intr_alloc.h"
#include "soc/dport_reg.h"
#include "soc/periph_defs.h"
#include "soc/twai_struct.h"
#include "hal/twai_ll.h"
#include <math.h>

#include "driver/gpio.h"

#include "can_regdef.h"
#include "can_config.h"
#include "esp32_can_builtin_lowlevel.h"
#include "esp32_can.h"
#include "dport_access.h"
#include "driver/can.h"
#include "driver/twai.h"

volatile uint32_t biIntsCounter = 0;
volatile uint32_t biReadFrames = 0;
volatile uint32_t needReset = 0;
QueueHandle_t lowLevelRXQueue;

static portMUX_TYPE builtincan_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define CANBI_ENTER_CRITICAL() portENTER_CRITICAL(&builtincan_spinlock)
#define CANBI_EXIT_CRITICAL() portEXIT_CRITICAL(&builtincan_spinlock)

esp_err_t CAN_read_frame(CAN_frame_t &__frame)
{

    // Wait for message to be received
    twai_message_t message;

    esp_err_t res = twai_receive(&message, 10 / portTICK_PERIOD_MS);
    if (res == ESP_OK)
    {
        // printf("Y\n");
    }
    else
    {
        // printf("N %d %s\n", res, esp_err_to_name(res));
        return res;
    }

    // printf("ID is %d\n", message.identifier);
    if (!(message.rtr))
    {
        __frame.MsgID = message.identifier;
        __frame.FIR.B.DLC = message.data_length_code;
        __frame.FIR.B.RTR = message.rtr ? CAN_RTR : CAN_no_RTR;
        // __frame.FIR.B.FF = message.
        for (int i = 0; i < message.data_length_code; i++)
        {
            // printf("Data byte %d = %d\n", i, message.data[i]);
            __frame.data.u8[i] = message.data[i];
        }
    }


    // xSemaphoreGive(proc_sem);

    // xQueueSend(lowLevelRXQueue, &__frame, 0);
    // return 1;
    return res;
}

bool CAN_TX_IsBusy()
{
    return !(TWAI.status_reg.tbs);
}

void CAN_SetListenOnly(bool mode)
{
    CAN_stop();
    TWAI.mode_reg.lom = mode ? 1 : 0;
    CAN_init();
}

bool CAN_GetListenOnlyMode()
{
    return TWAI.mode_reg.lom;
}

int CAN_write_frame(const CAN_frame_t *p_frame)
{
    // Configure message to transmit
    twai_message_t message;
    message.identifier = p_frame->MsgID;
    message.extd = 1; // p_frame->FIR.B.FF;
    message.rtr = (p_frame->FIR.B.RTR == CAN_RTR);
    message.data_length_code = p_frame->FIR.B.DLC;
    // printf("id = %04x / extd = %02x / rtr = %02x / data_length_code = %02x\n", message.identifier, message.extd, message.rtr, message.data_length_code);

    for (int i = 0; i < p_frame->FIR.B.DLC; i++)
    {
        message.data[i] = p_frame->data.u8[i];
    }

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1)) == ESP_OK)
    {
        // printf("Message queued for transmission\n");
        // printf(">");
    }
    else
    {
        printf("Failed to queue message for transmission\n");
    }

    return 0;
}

int CAN_init()
{

    printf("CAN_init - enter\n");

    // CANBI_ENTER_CRITICAL();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(/*CAN_cfg.tx_pin_id, CAN_cfg.rx_pin_id*/ GPIO_NUM_26, GPIO_NUM_12, TWAI_MODE_NORMAL);
    g_config.alerts_enabled = true;
    g_config.rx_queue_len = 50;
    g_config.tx_queue_len = 50;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    printf("CAN_init - config set \n");

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return 0;
    }
    /*
    #define TWAI_ALERT_TX_IDLE                  0x00000001  //< Alert(1): No more messages to transmit
    #define TWAI_ALERT_TX_SUCCESS               0x00000002  //< Alert(2): The previous transmission was successful
    #define TWAI_ALERT_RX_DATA                  0x00000004  //< Alert(4): A frame has been received and added to the RX queue
    #define TWAI_ALERT_BELOW_ERR_WARN           0x00000008  //< Alert(8): Both error counters have dropped below error warning limit
    #define TWAI_ALERT_ERR_ACTIVE               0x00000010  //< Alert(16): TWAI controller has become error active
    #define TWAI_ALERT_RECOVERY_IN_PROGRESS     0x00000020  //< Alert(32): TWAI controller is undergoing bus recovery
    #define TWAI_ALERT_BUS_RECOVERED            0x00000040  //< Alert(64): TWAI controller has successfully completed bus recovery
    #define TWAI_ALERT_ARB_LOST                 0x00000080  //< Alert(128): The previous transmission lost arbitration
    #define TWAI_ALERT_ABOVE_ERR_WARN           0x00000100  //< Alert(256): One of the error counters have exceeded the error warning limit
    #define TWAI_ALERT_BUS_ERROR                0x00000200  //< Alert(512): A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
    #define TWAI_ALERT_TX_FAILED                0x00000400  //< Alert(1024): The previous transmission has failed (for single shot transmission)
    #define TWAI_ALERT_RX_QUEUE_FULL            0x00000800  //< Alert(2048): The RX queue is full causing a frame to be lost
    #define TWAI_ALERT_ERR_PASS                 0x00001000  //< Alert(4096): TWAI controller has become error passive
    #define TWAI_ALERT_BUS_OFF                  0x00002000  //< Alert(8192): Bus-off condition occurred. TWAI controller can no longer influence bus
    #define TWAI_ALERT_RX_FIFO_OVERRUN          0x00004000  //< Alert(16384): An RX FIFO overrun has occurred
    #define TWAI_ALERT_TX_RETRIED               0x00008000  //< Alert(32768): An message transmission was cancelled and retried due to an errata workaround
    #define TWAI_ALERT_PERIPH_RESET             0x00010000  //< Alert(65536): The TWAI controller was reset
    #define TWAI_ALERT_ALL                      0x0001FFFF  //< Bit mask to enable all alerts during configuration
    #define TWAI_ALERT_NONE                     0x00000000  //< Bit mask to disable all alerts during configuration
    #define TWAI_ALERT_AND_LOG                  0x00020000  //< Bit mask to enable alerts to also be logged when they occur. Note that logging from the ISR is disabled if CONFIG_TWAI_ISR_IN_IRAM is enabled (see docs).
    */

    twai_reconfigure_alerts(
        TWAI_ALERT_ERR_ACTIVE |
            TWAI_ALERT_BUS_ERROR |
            TWAI_ALERT_TX_FAILED |
            TWAI_ALERT_RX_QUEUE_FULL |
            TWAI_ALERT_ERR_PASS |
            TWAI_ALERT_BUS_OFF |
            TWAI_ALERT_RX_FIFO_OVERRUN |
            TWAI_ALERT_TX_RETRIED |
            TWAI_ALERT_PERIPH_RESET,
        NULL);

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return 0;
    }

    // CANBI_EXIT_CRITICAL();

    return true;
}

// allocates a small queue used to buffer frames that were received in the
// interrupt handler but not yet processed by the rest of the code
void CAN_initRXQueue()
{
    lowLevelRXQueue = xQueueCreate(12, sizeof(CAN_frame_t));
}

int CAN_stop()
{

    // Stop the TWAI driver
    if (twai_stop() == ESP_OK)
    {
        printf("Driver stopped\n");
    }
    else
    {
        printf("Failed to stop driver\n");
        return 0;
    }

    // Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK)
    {
        printf("Driver uninstalled\n");
    }
    else
    {
        printf("Failed to uninstall driver\n");
        return 0;
    }

    return 0;
}

/*
static void ctrl_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
    ESP_LOGI(EXAMPLE_TAG, "Starting transmissions");
    xSemaphoreGive(tx_task_sem);    //Start transmit task

    //Prepare to trigger errors, reconfigure alerts to detect change in error state
    twai_reconfigure_alerts(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);
    for (int i = 3; i > 0; i--) {
        ESP_LOGW(EXAMPLE_TAG, "Trigger TX errors in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(EXAMPLE_TAG, "Trigger errors");
    trigger_tx_error = true;

    while (1) {
        uint32_t alerts;
        twai_read_alerts(&alerts, portMAX_DELAY);
        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
            ESP_LOGI(EXAMPLE_TAG, "Surpassed Error Warning Limit");
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {
            ESP_LOGI(EXAMPLE_TAG, "Entered Error Passive state");
        }
        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGI(EXAMPLE_TAG, "Bus Off state");
            //Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
            twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
            for (int i = 3; i > 0; i--) {
                ESP_LOGW(EXAMPLE_TAG, "Initiate bus recovery in %d", i);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            twai_initiate_recovery();    //Needs 128 occurrences of bus free signal
            ESP_LOGI(EXAMPLE_TAG, "Initiate bus recovery");
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            //Bus recovery was successful, exit control task to uninstall driver
            ESP_LOGI(EXAMPLE_TAG, "Bus Recovered");
            break;
        }
    }
    //No need call twai_stop(), bus recovery will return to stopped state
    xSemaphoreGive(ctrl_task_sem);
    vTaskDelete(NULL);
}
*/

uint32_t CAN_isFaulted2()
{
    uint32_t alerts;
    twai_read_alerts(&alerts, 10);
    return alerts;
}
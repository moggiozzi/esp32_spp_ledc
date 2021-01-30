#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "soc/uart_reg.h" // uart flags defs
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "soc/uart_struct.h"
#include "esp_log.h"
#include "esp_attr.h" //IRAM_ATTR
#include "esp_system.h" // esp_restart()

#define dbg_printf(tag,fmt,...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)

#define MB_QUEUE_LENGTH             (256)

QueueHandle_t xMbUartQueue;
static TaskHandle_t xMbTaskHandle;

void mbrtuTmr_startIntercharTimer(void);

#define UART_TIMEOT_EVENT (UART_EVENT_MAX+1)
IRAM_ATTR portBASE_TYPE uart_addTimeoutEventFromIsr(void)
{
    portBASE_TYPE hPTaskAwoken = 0;

    uart_event_t xEvent = {.type = UART_TIMEOT_EVENT, .size = 0};
    if (xMbUartQueue)
    	xQueueSendFromISR(xMbUartQueue, (void * )&xEvent, &hPTaskAwoken);
    return  hPTaskAwoken;
}

static void vUartTask(void* pvParameters)
{
    uart_event_t xEvent;
    int event;
    for(;;) {
        if (xQueueReceive(xMbUartQueue, (void*)&xEvent, pdMS_TO_TICKS(5000)) == pdTRUE) { // portMAX_DELAY
            event = xEvent.type;
            switch(event) {
                case UART_DATA:
                    dbg_printf(__func__, "Receive data, len: %d.", xEvent.size);
                    mbrtuTmr_startIntercharTimer();
                    break;
                case UART_FIFO_OVF:
                	dbg_printf(__func__, "hw fifo overflow.");
                    break;
                case UART_BUFFER_FULL:
                	dbg_printf(__func__, "ring buffer full.");
                    break;
                case UART_BREAK:
                	dbg_printf(__func__, "uart rx break.");
                    break;
                case UART_PARITY_ERR:
                	dbg_printf(__func__, "uart parity error.");
                    break;
                case UART_FRAME_ERR:
                	dbg_printf(__func__, "uart frame error.");
                    break;
                case UART_TIMEOT_EVENT:
                	dbg_printf(__func__, "uart timeout.");
                	//todo handle data
                	break;
                default:
                	dbg_printf(__func__, "uart event type: %d.", xEvent.type);
                    break;
            }
        } else {
        	dbg_printf(__func__, "no events");
        }
    }
    vTaskDelete(NULL);
}

#define MB_TIMER_PRESCALLER     (80) //TIMER_BASE_CLK == APB_CLK_FREQ == 80 MHZ
#define MB_TIMER_WITH_RELOAD    (TIMER_AUTORELOAD_DIS)
#define TIMEOUT_US (4000)
DRAM_ATTR static const uint16_t usTimerIndex =      0;
DRAM_ATTR static const uint16_t usTimerGroupIndex = 0;

IRAM_ATTR void vTimerGroupIsr(void *param)
{
	timer_spinlock_take(usTimerGroupIndex);
    timer_intr_t timer_intr = timer_group_get_intr_status_in_isr(usTimerGroupIndex);
    if ((timer_intr & TIMER_INTR_T0) != 0)
    {
		//assert((int)param == usTimerIndex);
		timer_group_clr_intr_status_in_isr(usTimerGroupIndex, usTimerIndex);
		uart_addTimeoutEventFromIsr();
    }
    timer_spinlock_give(usTimerGroupIndex);
}

int8_t mbrtuTmr_init(void)
{
	dbg_printf(__func__, "");
    esp_err_t xErr;
    timer_config_t config;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = MB_TIMER_WITH_RELOAD;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = MB_TIMER_PRESCALLER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    // Configure timer
    xErr = timer_init(usTimerGroupIndex, usTimerIndex, &config);
    if (xErr != ESP_OK)
        dbg_printf(__func__, "timer init failure, timer_init() returned (0x%x).", (uint32_t)xErr);
    // Stop timer counter
    xErr = timer_pause(usTimerGroupIndex, usTimerIndex);
    if (xErr != ESP_OK)
        dbg_printf(__func__, "stop timer failure, timer_pause() returned (0x%x).", (uint32_t)xErr);
    // Reset counter value
    xErr = timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0x00000000ULL);
    if (xErr != ESP_OK)
        dbg_printf(__func__, "timer set value failure, timer_set_counter_value() returned (0x%x).", (uint32_t)xErr);
    xErr = timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(TIMEOUT_US));
    if (xErr != ESP_OK)
        dbg_printf(__func__, "failure to set alarm failure, timer_set_alarm_value() returned (0x%x).", (uint32_t)xErr);
    // Register ISR for timer
    dbg_printf(__func__, "timer_isr_register");
    xErr = timer_isr_register(usTimerGroupIndex, usTimerIndex, vTimerGroupIsr, (void*)(uint32_t)usTimerIndex, ESP_INTR_FLAG_IRAM, NULL);
    if (xErr != ESP_OK)
    	dbg_printf(__func__, "timer set value failure, timer_isr_register() returned (0x%x).", (uint32_t)xErr);
    dbg_printf(__func__, "timer init");

    // for test
    mbrtuTmr_startIntercharTimer();

    return true;
}

void mbrtuTmr_startIntercharTimer(void)
{
	dbg_printf(__func__, "");
    timer_pause(usTimerGroupIndex, usTimerIndex);
    timer_set_counter_value(usTimerGroupIndex, usTimerIndex, 0ULL);
    timer_set_alarm_value(usTimerGroupIndex, usTimerIndex, (uint32_t)(TIMEOUT_US));
    timer_set_alarm(usTimerGroupIndex, usTimerIndex, 1);
    timer_enable_intr(usTimerGroupIndex, usTimerIndex);
    timer_start(usTimerGroupIndex, usTimerIndex);
}


/********************************* REBOOT CODE ******************************/
static TimerHandle_t s_delayed_restart_timer = NULL;
static void restart(TimerHandle_t timer) {
	dbg_printf(__func__, "reboot...");
	esp_restart();
}
/***************************************************************************/


void test(void) {
	s_delayed_restart_timer = xTimerCreate("RestartTimer", 1, pdFALSE, NULL, restart);
	xTimerChangePeriod(s_delayed_restart_timer, pdMS_TO_TICKS(15*1000), portMAX_DELAY);
	xTimerStart(s_delayed_restart_timer, portMAX_DELAY);

    xMbUartQueue = xQueueCreate(MB_QUEUE_LENGTH, sizeof(uart_event_t));
    xTaskCreate(vUartTask, "uart_task", 4*1024, NULL, 10, &xMbTaskHandle);
    mbrtuTmr_init();
}

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"
#include "esphome/components/uart/uart_component_esp_idf.h"
#define ESPHOME_UART uart::IDFUARTComponent

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  auto uartComp = static_cast<ESPHOME_UART *>(this->parent_);
  uart_port_t uart_num = static_cast<uart_port_t>(uartComp->get_hw_serial_number());

  // Tweak the fifo settings so data is available as soon as the first byte is recieved.
  // If not it will wait either until fifo is filled or a certain time has passed.
  uart_intr_config_t uart_intr;
  uart_intr.intr_enable_mask =
      UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M;  // only these IRQs - no BREAK, PARITY or OVERFLOW
  uart_intr.rxfifo_full_thresh =
      1;  // UART_FULL_THRESH_DEFAULT,  //120 default!! aghh! need receive 120 chars before we see them
  uart_intr.rx_timeout_thresh =
      10;  // UART_TOUT_THRESH_DEFAULT,  //10 works well for my short messages I need send/receive
  uart_intr.txfifo_empty_intr_thresh = 10;  // UART_EMPTY_THRESH_DEFAULT
  uart_intr_config(uart_num, &uart_intr);

  // Creating UART event Task
  xTaskCreatePinnedToCore(LinBusListener::uartEventTask_,
                          "uart_event_task",                      // name
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,   // stack size (in words)
                          this,                                   // input params
                          24,                                     // priority
                          &this->uartEventTaskHandle_,            // handle
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE  // core
  );
  if (this->uartEventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- UART%d Event Task not created!", uart_num);
  }

  // Creating LIN msg event Task
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",         // name
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,                     // stack size (in words)
                          this,                     // input params
                          2,                        // priority
                          &this->eventTaskHandle_,  // handle
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE                         // core
  );

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- LIN message Task not created!");
  }
}

void LinBusListener::uartEventTask_(void *pvParameters) {
  LinBusListener *self = (LinBusListener *)pvParameters;
  auto *uartComp = self->parent_->get_uart_parent();
  
  // Wir lesen jetzt direkt, solange Daten im Buffer sind, 
  // anstatt auf ein Queue-Event zu warten, das in Arduino 3.0 gesperrt ist.
  while (true) {
    size_t available = uartComp->available();
    if (available > 0) {
      uint8_t data;
      while (uartComp->read_byte(&data)) {
        self->handle_char_(data);
      }
    }
    // Kurze Pause, um den Watchdog nicht zu triggern (1ms)
    delay(1); 
  }
}

void LinBusListener::eventTask_(void *args) {
  LinBusListener *instance = (LinBusListener *) args;
  for (;;) {
    instance->process_lin_msg_queue(QUEUE_WAIT_BLOCKING);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef QUEUE_WAIT_BLOCKING
#undef ESPHOME_UART

#endif  // USE_ESP32_FRAMEWORK_ARDUINO

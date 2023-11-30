The dtm (direct test mode) module can be ported to general BLE example of Apollo4 by following the steps.

1. Modify the config-template.ini of project locating in boards/(YOUR_BOARD)/examples/ble/(YOUR_PROJECT):
    1.1 Add /dtm_module to the include path
         Include = src
                  ...
                  %(SoftwareRoot)s/ambiq_ble/apps/dtm_module

    1.2 Add /dtm_module to the VPath
         VPath = src
                  ...
                  %(SoftwareRoot)s/ambiq_ble/apps/dtm_module

    1.3 Add dtm_main.c to VPath Sources
         Sources =
                  ...
                  dtm_main.c

2. Edit the radio_task.c:
    2.1 Include the necessary .h files
         #include "dtm_api.h"

    2.2 Call the dtm_mode entrance api in RadioTask():
         void RadioTask(void *pvParameters)
         {
          ...
          while(1)
          {
          #if defined(ENABLE_DTM_MODE_FEATURE) && (ENABLE_DTM_MODE_FEATURE == 1)
               if ( g_bDtmModeRunning )
               {
                   reset_ble_and_enter_dtm();
                   dtm_process();
               }
          #endif

               wsfOsDispatcher();
          }
         }

3. Edit the ble_freertos_xx.c, for example, ble_freertos_amota.c:
    3.1 Include the necessary .h files
          #include "dtm_api.h"

    3.2 Design avaiable interface calling ui_switch_to_dtm() to switch to DTM from the current application. 
          *If the button on your board is used to switch, you can refer to button_init.c to define the ISR of GPIO and register the handler of button action.
          *Then initialize the gpio before task running.
          int main(void)
          {
          ...
          #if defined(AM_PART_APOLLO4B)
          #if defined(ENABLE_DTM_MODE_FEATURE) && (ENABLE_DTM_MODE_FEATURE == 1)
              dtm_enter_button_init();
          #endif
          #endif
          
           run_tasks();
           ...
          }

4. Implement the serial interface functions defined in dtm_api.h in your application. If UART is used for the serial communication, you can refer to uart_ble_bridge.c locating in boards/(YOUR_BOARD)/examples/ble/uart_ble_bridge/src.
    void serial_interface_init(void);
    void serial_interface_deinit(void);
    void serial_data_read(uint8_t* pui8Data, uint32_t* ui32Length);
    void serial_data_write(uint8_t* pui8Data, uint32_t ui32Length);
    void serial_irq_enable(void);
    void serial_irq_disable(void);
    void serial_task(void);

Note:
1. If GPIO button is used to switch, be careful to define the ISR of GPIO to avoid the conflit between different GPIO you use in the same application. 


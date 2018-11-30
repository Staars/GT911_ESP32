#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "Goodix.h"
#include "GoodixFW.h"

#define INT_PIN                GPIO_NUM_12
#define RST_PIN                GPIO_NUM_14
#define GOODIX_SDA             GPIO_NUM_21
#define GOODIX_SCL             GPIO_NUM_22

#define LOG_TAG "GOODIX"

Goodix touch = Goodix();

extern "C" {
  
void handleTouch(int8_t contacts, GTPoint *points) {
  ESP_LOGD(LOG_TAG,"Contacts: %d", contacts);
  for (uint8_t i = 0; i < contacts; i++) {
    // ESP_LOGI(LOG_TAG,"C%d: #%d %d,%d s:%d", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
   printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);

  }
}

void touchStart() {
 if (touch.begin(INT_PIN, RST_PIN)!=true) {
   ESP_LOGE(LOG_TAG,"Module reset failed");
  } else {
    ESP_LOGI(LOG_TAG,"Module reset OK");
  }
  ESP_LOGI(LOG_TAG,"Check ACK on addr request on 0x%x",touch.i2cAddr);
}

uint8_t dumpCFG(){
  uint8_t buffer[GOODIX_CONFIG_911_LENGTH];
  uint8_t i2c_err = touch.read(GOODIX_REG_CONFIG_DATA, buffer, GOODIX_CONFIG_911_LENGTH);
  if (i2c_err != ESP_OK){
    ESP_LOGD(LOG_TAG,"---error---");
    ESP_LOGD(LOG_TAG,"----------");
  }
  else{  
  ESP_LOGD(LOG_TAG,"-no--error--");
  uint8_t t=0;
  printf("Consider to make a backup now:\r\n");
  printf("uint8_t g911xCurrentFW[] = {");
  for (uint8_t i=0; i<(GOODIX_CONFIG_911_LENGTH-1); i++) {
    printf("0x%02x, ", buffer[i]);
    t++;
    if (t>=16) {
      printf("\r\n");
      t=0;
    }
  }
  printf("0x%02x};\r\n",buffer[GOODIX_CONFIG_911_LENGTH-1]);
  ESP_LOGD(LOG_TAG,"----------");
  }
  return i2c_err;
}


void i2c_setup(){
  i2c_config_t i2c_master_config;
  i2c_master_config.mode = I2C_MODE_MASTER;
  i2c_master_config.sda_io_num = (gpio_num_t)GOODIX_SDA;
  i2c_master_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_master_config.scl_io_num = (gpio_num_t)GOODIX_SCL;
  i2c_master_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_master_config.master.clk_speed = 400000;
  i2c_param_config(I2C_NUM_0, &i2c_master_config);
  i2c_driver_install(I2C_NUM_0, i2c_master_config.mode, 0, 0, 0);
}

void i2c_scan()
{
    uint8_t foundDevices = 0;
  	for(int address = 1; address < 127; address++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
      foundDevices++;
		}
		i2c_cmd_link_delete(cmd);
	  }
    if (foundDevices == 0)
    {
      printf("-> found NO devices");
    }
}

// void gt911_reset(void){
//   gpio_pad_select_gpio(RST_PIN);
//   gpio_set_direction((gpio_num_t)RST_PIN, GPIO_MODE_OUTPUT);
//   gpio_set_level((gpio_num_t)RST_PIN, 1);
//   vTaskDelay(300 / portTICK_PERIOD_MS);
//   gpio_set_level((gpio_num_t)RST_PIN, 0);
//   vTaskDelay(300 / portTICK_PERIOD_MS);
// }

void loop_task(void *pvParameter)
{
    ESP_LOGI(LOG_TAG,": touch started"); 
    while(1) { 
		vTaskDelay(10 / portTICK_RATE_MS);	
      touch.loop();
    }
}


void app_main() {
  vTaskDelay(300 / portTICK_PERIOD_MS);
  esp_log_level_set(LOG_TAG, ESP_LOG_DEBUG);
  ESP_LOGI(LOG_TAG,": Goodix GT911x touch driver");
  // gt911_reset();
  vTaskDelay(300 / portTICK_PERIOD_MS);
  i2c_setup();  // setup I2C outside of the library
  ESP_LOGI(LOG_TAG,": Goodix I2C Setup complete");
  i2c_scan(); // just for basic debugging

  touch.setHandler(handleTouch);
  touchStart();
  
  dumpCFG(); // you can copy/paste this into GoodixFW.h

  vTaskDelay(300 / portTICK_PERIOD_MS);

  xTaskCreate(&loop_task, "loop_task", 8192, NULL, 5, NULL);
}
}
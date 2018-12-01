#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "Goodix.h"
#include "GoodixFW.h"

#define INT_PIN                12
#define RST_PIN                14
#define GOODIX_SDA             21
#define GOODIX_SCL             22
#define GOODIX_SPEED           100000

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

uint8_t dumpRegID(){
  uint8_t buffer[11];
  uint8_t i2c_err = touch.read(GOODIX_REG_ID, buffer, 11);
  if (i2c_err != ESP_OK){
    ESP_LOGD(LOG_TAG,"---error---");
  }
  else{  
  ESP_LOGD(LOG_TAG,"-no--error--");
  printf("Product-ID: %c%c%c%c\r\n",buffer[0],buffer[1],buffer[2],buffer[3]);
  printf("Firmware-Version: %x%x\r\n",buffer[5],buffer[4]);
  uint16_t res = buffer[6] | buffer[7] << 8;
  printf("X-Resolution: %d\r\n",res);
  res = buffer[8] | buffer[9] << 8;
  printf("Y-Resolution: %d\r\n",res);
  printf("Vendor-ID: %x\r\n",buffer[10]);
  }
  return i2c_err;  
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
  vTaskDelay(300 / portTICK_PERIOD_MS);
  touch.i2cSetup(GOODIX_SDA, GOODIX_SCL, GOODIX_SPEED); 
  ESP_LOGI(LOG_TAG,": Goodix I2C Setup complete");
  i2c_scan(); // just for basic debugging

  touch.setHandler(handleTouch);
  touchStart();
  
  dumpCFG(); // you can copy/paste this into GoodixFW.h
  dumpRegID(); 

  vTaskDelay(300 / portTICK_PERIOD_MS);

  xTaskCreate(&loop_task, "loop_task", 8192, NULL, 5, NULL);
}
}
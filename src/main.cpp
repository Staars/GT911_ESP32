// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"


#include "Goodix.h"
#include "GoodixFW.h"



#define INT_PIN GPIO_NUM_12
#define RST_PIN GPIO_NUM_14

#define GOODIX_SDA             GPIO_NUM_21
#define GOODIX_SCL             GPIO_NUM_22

#define GOODIX "GOODIX"



Goodix touch = Goodix();
char *myID;


extern "C" {
  
void handleTouch(int8_t contacts, GTPoint *points) {
  ESP_LOGD(GOODIX,"Contacts: %d", contacts);
 
  for (uint8_t i = 0; i < contacts; i++) {
    // ESP_LOGI(GOODIX,"C%d: #%d %d,%d s:%d", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
   printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);

  }
}

void touchStart() {
 if (touch.begin(INT_PIN, RST_PIN)!=true) {
   ESP_LOGE(GOODIX,"Module reset failed");
  } else {
    ESP_LOGI(GOODIX,"Module reset OK");
  }
  ESP_LOGI(GOODIX,"Check ACK on addr request on 0x%x",touch.i2cAddr);
}


int32_t dumpRegs(uint16_t reg, uint8_t len){
    
    uint8_t buffer[200];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ESP_LOGD(GOODIX,": set reg");
    i2c_master_write_byte(cmd, (uint8_t)(( 0x5d << 1 ) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
     i2c_master_write_byte(cmd, (uint8_t)(reg >> 8), I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, (uint8_t)(reg & 0xff), I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t esp_i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    ESP_LOGD(GOODIX,"i2c master read error: %d", esp_i2c_err);
    i2c_cmd_link_delete(cmd);
  // make a break and continue with the read
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)(( 0x5d << 1 ) | I2C_MASTER_READ), I2C_MASTER_ACK);
    ESP_LOGD(GOODIX,": 0x5d << 1 ) | I2C_MASTER_READ");
    ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, (size_t)len - 1 , I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &buffer[len], I2C_MASTER_NACK));
    i2c_master_stop(cmd);
    esp_i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    ESP_LOGD(GOODIX,"i2c master read error: %d", esp_i2c_err);
    i2c_cmd_link_delete(cmd);

if (esp_i2c_err != ESP_OK){
  ESP_LOGD(GOODIX,"---error---");
  ESP_LOGD(GOODIX,"----------");
}
else{  
  ESP_LOGD(GOODIX,"-no--error--");
  uint8_t t=0;
  printf("Consider to make a backup now:\r\n");
  printf("uint8_t g911xCurrentFW[] = {");
  for (uint8_t i=0; i<len; i++) {
    printf("0x%02x, ", buffer[i]);
    t++;
    if (t>=16) {
    printf("\r\n");
    t=0;
    }
  }
  printf("};\r\n");
  ESP_LOGD(GOODIX,"----------");

}
    // success = true;
  return esp_i2c_err;
  }
  // return success;



void my_setup(){
  i2c_config_t i2c_master_config;
  i2c_master_config.mode = I2C_MODE_MASTER;
  i2c_master_config.sda_io_num = (gpio_num_t)GOODIX_SDA;
  i2c_master_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_master_config.scl_io_num = (gpio_num_t)GOODIX_SCL;
  i2c_master_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_master_config.master.clk_speed = 400000;
  i2c_param_config((i2c_port_t)0, &i2c_master_config);
  i2c_driver_install((i2c_port_t)0, i2c_master_config.mode, 0, 0, 0);
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
    touch.setHandler(handleTouch);
    touchStart();
    ESP_LOGI(GOODIX,": touch started");
  
    while(1) { 
		vTaskDelay(10 / portTICK_RATE_MS);	
      touch.loop();
    }
}



void app_main() {
  vTaskDelay(300 / portTICK_PERIOD_MS);
  esp_log_level_set(GOODIX, ESP_LOG_DEBUG);
  ESP_LOGI(GOODIX,": Goodix GT911x touch driver");
  // gt911_reset();
  vTaskDelay(300 / portTICK_PERIOD_MS);
  my_setup(); // This must move to Goodix.cpp later !!
  ESP_LOGI(GOODIX,": Goodix I2C Setup complete");
  i2c_scan(); // just for basic debugging
  
  dumpRegs(GOODIX_REG_CONFIG_DATA, GOODIX_CONFIG_911_LENGTH); // you can copy/paste this into GoodixFW.h

  vTaskDelay(300 / portTICK_PERIOD_MS);

  xTaskCreate(&loop_task, "loop_task", 8192, NULL, 5, NULL);
}
}
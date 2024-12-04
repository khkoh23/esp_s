/*
 * 
 *
 * 
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"
#include "driver/i2c_slave.h"
#include "driver/twai.h"
#include "driver/uart.h"

#define JETSON_24 GPIO_NUM_0	// strapping pin
#define DI_1 GPIO_NUM_1
#define DI_0 GPIO_NUM_2
#define DO_4 GPIO_NUM_3			// strapping pin
#define DI_13 GPIO_NUM_4
#define DI_14 GPIO_NUM_5
#define DI_15 GPIO_NUM_6
#define DI_16 GPIO_NUM_7
#define RS3_DE (GPIO_NUM_8)
#define SPI_CS_1 GPIO_NUM_9 	// si8380 (DI10,11,17,19-23)
#define SPI_CS_2 GPIO_NUM_10 	// si8380 (DI2-9)
#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define PASS_1 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS3_TX (GPIO_NUM_17)
#define RS3_RX (GPIO_NUM_18)
#define DI_18 GPIO_NUM_19		// usb-jtag
#define DI_12 GPIO_NUM_20		// usb-jtag
#define PASS_2 GPIO_NUM_21
// GPIO22-25 not available
// GPIO26-32 for SPI0/1 flash and PSRAM
// GPIO33-37 for SPIIO4~SPIIO7, SPIDQS Octal flash or Octal PSRAM
#define DO_0 GPIO_NUM_35
#define DO_1 GPIO_NUM_36
#define DO_2 GPIO_NUM_37
#define DO_3 GPIO_NUM_38
#define CAN2_TX GPIO_NUM_39
#define CAN2_RX GPIO_NUM_40
#define DO_8 GPIO_NUM_41
#define DO_9 GPIO_NUM_42
// GPIO_NUM_43
// GPIO_NUM_44
// GPIO_NUM_45					// strapping pin
#define DO_5 GPIO_NUM_46		// strapping pin
#define BOOTKEY (GPIO_NUM_47)
#define BMS GPIO_NUM_48

#define DMX_UART_NUM (UART_NUM_2) // dmx uart
#define BUF_SIZE 512
#define LOW_BAT_THRESHOLD 20

static const char *BOOTKEY_TAG = "bootkey_gpio";
static const char *FUNCTIONAL_TAG = "functional_gpio";
static const char *ESTOP_PCNT_TAG = "estop_pcnt";
static const char *SLAVE_SPI_DI_TAG = "slave_spi_di";
static const char *SLAVE_DO_TAG = "slave_do";
static const char *I2C_SLAVE_TAG = "i2c_slave";
static const char *DMX_RS485_TAG = "dmx_rs485";
static const char *NC_TWAI_TAG = "nc_twai";
//static const char *AMR_STATE_TAG = "amr_state";


/* -------------------- global -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

/* shoalbot_amr_state I will decide the state based on priority, then tell DMX
0: SHTDWN 1: ESTOP 2: ERROR 3: LOWBAT 4: CHRGNG 5: IDLE 6: SHWBAT
11: BLOCK 12: LEFT 13: RIGHT 14: MOVE 15: FAIL 16: AUTOCHRGNG
*/
int8_t shoalbot_amr_state = 5;
int8_t prev_amr_state;

bool slave_gpio_di[8] = {};
uint8_t slave_spi_di0, slave_spi_di1;
uint8_t slave_to_master_buffer[6] = {
	0b00000000, // at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
	0b00000000, // at slave: DI_15, DI_14, DI_13, DI_12, DI_11, DI_10, DI_09, DI_08
	0b00000000, // at slave: DI_7, DI_6, DI_5, DI_4, DI_3, DI_2, DI_1, DI_0
	0b00000000, // at slave: DO_9. DO_8. DO_5. DO_4. DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, x, x, BOOTKEY
	0b00000000, // esp_reset_reason (SLAVE)
};
uint8_t slave_do = 0; // MSB to LSB: DO_9, DO_8, DO_5 to DO_0
uint8_t slave_pass1_pass2_bms; //MSB to LSB: x, x, x, x, x, PASS_2, PASS_1, BMS
//uint8_t navigation_intent_index;
uint8_t battery_level = 0;
bool new_i2c_incoming;
bool new_slave_gpio_do = false;

bool isBootkeylongpressed;
bool isEstopengaged;
bool isManualchargeengaged;
bool isResetpressed;
bool isBumperengaged;
bool isBatterylow;
bool isBlock;
bool isLeft;
bool isRight;
bool isMove;
bool isFail;
bool isAutochargengaged;
uint16_t showbat_counter;
uint8_t indexBattery;
bool isShowbat;
int pulse_count = 0, prev_pulse_count;
static uint16_t _StartDMXAddr; // First adress listened
static uint16_t _NbChannels; // Number of channels listened from the start address
static uint8_t * dmx_data; // stores the validated dmx data
static uint8_t * tmp_dmx_data; // stores the received dmx data
uint8_t start_channel = 33; // 1 for AMR1, 33 for AMR2
uint8_t temp_brightness = 30;
uint8_t temp_counter = 50;
uint8_t horse_sequence;
bool led_wave_dir;
esp_reset_reason_t slave_reason;
// uint8_t crash_i; // malicious counter


/* ---------------FUNCTION PROTOTYPES---------------- */
void amr_state_machine(void* arg);
void check_bootkey(void* arg);
void check_functional_inputs(void* arg);
void check_estop(void* arg);
uint8_t read_spi_1();
uint8_t read_spi_2();
void slave_gpio_spi_di_task(void* arg);
esp_err_t i2c_read();
void i2c_task(void *arg);
void createBuffer(bool TmpBufferCreate);
void dmx_write(uint16_t channel, uint8_t value);
void dmx_write_all(uint8_t * data, uint16_t start, size_t size);
void dmx_uart_task(void* arg);
void set_led(const uint8_t color_cmd);
void slave_gpio_do_task(void* arg);


/* -------------------- amr state -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void amr_state_machine(void* arg) { 
	while(1) {
		if (isBootkeylongpressed) shoalbot_amr_state = 0;
		// if (isEstopengaged) shoalbot_amr_state = 1; // Remove after test
		else if (isEstopengaged) shoalbot_amr_state = 1;
		else if (isBumperengaged) shoalbot_amr_state = 2;
		else if ((prev_amr_state == 1) | (prev_amr_state == 2)) { // to check reset for releasing from ESTOP and ERROR state
			if (isResetpressed) shoalbot_amr_state = 5;
			else shoalbot_amr_state = prev_amr_state;
		}
		else if (isManualchargeengaged) shoalbot_amr_state = 4;
		else if (isAutochargengaged) shoalbot_amr_state = 16;
		else if (isBatterylow) shoalbot_amr_state = 3; 
		else if (isBlock) shoalbot_amr_state = 11;
		else if (isLeft) shoalbot_amr_state = 12;
		else if (isRight) shoalbot_amr_state = 13;
		else if (isMove) shoalbot_amr_state = 14;
		else if (isFail) shoalbot_amr_state = 15;
		else if (isShowbat) shoalbot_amr_state = 6;
		else shoalbot_amr_state = 5;
		slave_to_master_buffer[4] = shoalbot_amr_state;

		switch (shoalbot_amr_state) {
			case 0: // SHTDWN
				gpio_set_level(PASS_1, 0); 
				gpio_set_level(PASS_2, 0); 
				gpio_set_level(BMS, 0); // cut-off BMS
				gpio_set_level(DO_0, 0); // Lidar 24V
				gpio_set_level(DO_1, 0); // LED	24V
				gpio_set_level(DO_2, 0); // Relay K2 for Auto Charging
				gpio_set_level(DO_3, 0); // Relay K3 for Kinco Power
				break;
			case 1: // ESTOP
				gpio_set_level(PASS_1, 0); 
				gpio_set_level(PASS_2, 0); 
				set_led(101); // red color wavering
				break;
			case 2: // ERROR
				gpio_set_level(PASS_1, 0); 
				gpio_set_level(PASS_2, 0); 
				set_led(102); // dark red color blinking
				break;
			case 3: // LOWBAT
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(103); // dark red color horse lamp
				break;
			case 4: // CHRGNG
				gpio_set_level(PASS_1, 0); 
				gpio_set_level(PASS_2, 0); 
				set_led(104); // orange color wavering
				break;
			case 5: // IDLE
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				// gpio_set_level(BMS, 1);
				set_led(105); // white color wavering
				showbat_counter += 1;
				if (showbat_counter > 1023) isShowbat = true; // 2047
				break;
			case 6: // SHWBAT
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				switch (indexBattery) {
					case 1: // <50
						set_led(106); // dark orange horse lamp
						break;
					case 2: // >=50 and <80
						set_led(107); // dark yellow color horse lamp
						break;
					case 3: // >=80
						set_led(108); // dark green color horse lamp
						break;
					default:
						set_led(105); // white color wavering
						break;
				}
				showbat_counter -= 2; // 4
				if (showbat_counter < 15) isShowbat = false;
				break;
			case 11: // BLOCK
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(111); // pink purple color horse lamp
				break;
			case 12: // LEFT
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(112); // orange color (blinking) left
				break;
			case 13: // RIGHT
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(113); // orange color (blinking) right
				break;
			case 14: // MOVE
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(114); // blue color wavering
				break;
			case 15: // FAIL
				gpio_set_level(PASS_1, 1); 
				gpio_set_level(PASS_2, 1); 
				set_led(111); // pink purple color horse lamp
				break;
			case 16: // AUTOCHRGNG
				gpio_set_level(PASS_1, 0); 
				gpio_set_level(PASS_2, 0); 
				set_led(104); // orange color wavering
				break;
			default:
				break;
		}
		prev_amr_state = shoalbot_amr_state;
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}


/* -------------------- bootkey -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/
void check_bootkey(void* arg) { 
	bool bootkey_pressed = false;
	uint32_t bootkey_start = 0; // start timer when bootkey is pressed
	while(1) {
		if (gpio_get_level(BOOTKEY)) {
			//ESP_LOGI(BOOTKEY_TAG, "BOOTKEY is pressed");
			if (!bootkey_pressed) {
				bootkey_pressed = true;
				bootkey_start = esp_log_timestamp();
			}
			else if (bootkey_pressed) {
				uint32_t elapsed_time = esp_log_timestamp() - bootkey_start;
				if (elapsed_time > 500 ) { // 0.5 seconds
					//ESP_LOGI(BOOTKEY_TAG, "BOOTKEY pressed for 5 seconds");
					isBootkeylongpressed = true;
				}
				else isBootkeylongpressed = false;
			}
		}
		else {
			bootkey_pressed = false;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- functional inputs -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void check_functional_inputs(void* arg) { 
	// slave_to_master_buffer[0] -> at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
	while(1) {
		if ((slave_to_master_buffer[0] & 0b10000000) >> 7) { // manual charging DI_23
			//ESP_LOGI(FUNCTIONAL_TAG, "Manual charging is engaged");
			isManualchargeengaged = true;
		}
		else {
			isManualchargeengaged = false;
		}
		if ((slave_to_master_buffer[0] & 0b00100000) >> 5) { // reset DI_21
			//ESP_LOGI(FUNCTIONAL_TAG, "Reset is pressed");
			isResetpressed = true;
		}
		else {
			isResetpressed = false;
		}
		if (((slave_to_master_buffer[0] & 0b00000001) >> 0) | ((slave_to_master_buffer[0] & 0b00000100) >> 2)) { // front bumper DI_16, back bumper DI_18 
			//ESP_LOGI(FUNCTIONAL_TAG, "Bumper(any) is engaged");
			isBumperengaged = true;
		}
		else {
			isBumperengaged = false;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- estop -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

pcnt_unit_config_t unit_config = {
	.low_limit = -100,
    .high_limit = 100,
};
pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
};
pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num = DI_0,
};
pcnt_channel_handle_t pcnt_chan_a = NULL;
pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num = DI_1,
};
pcnt_channel_handle_t pcnt_chan_b = NULL;

void check_estop(void* arg) {
	while(1) {
		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        // ESP_LOGI(ESTOP_PCNT_TAG, "Pulse count: %d", pulse_count);
		if ((pulse_count - prev_pulse_count) < 2) { // no new estop pulse is detected for the cycle
		// if ((pulse_count - prev_pulse_count) == 0) { // no new estop pulse is detected for the cycle
			// ESP_LOGI(ESTOP_PCNT_TAG, "ESTOP !");
			isEstopengaged = true;
		}
		else {
			isEstopengaged = false;
		}
		prev_pulse_count = pulse_count;
		if (pulse_count>95) { // a high enough odd number
			ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
			prev_pulse_count = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- slave gpio do -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void slave_gpio_do_task(void* arg) {
//	gpio_set_level(DO_3, 1); // beginning
	while(1) {
		if(new_slave_gpio_do) {
			gpio_set_level(DO_0, slave_do & 0x01);
			gpio_set_level(DO_1, slave_do & 0x02);
			gpio_set_level(DO_2, slave_do & 0x04);
			gpio_set_level(DO_3, slave_do & 0x08); 
			gpio_set_level(DO_4, slave_do & 0x10);
			gpio_set_level(DO_5, slave_do & 0x20);
			gpio_set_level(DO_8, slave_do & 0x40);
			gpio_set_level(DO_9, slave_do & 0x80);
			slave_to_master_buffer[3] = slave_do; 
			gpio_set_level(BMS, slave_pass1_pass2_bms & 0x80);
			new_slave_gpio_do = false;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}


/* -------------------- slave gpio spi di -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/
spi_device_handle_t handle_1, handle_2;
spi_bus_config_t bus_config = {
	.mosi_io_num = SPI_MOSI,
	.miso_io_num = SPI_MISO,
	.sclk_io_num = SPI_SCLK,
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,	
};
spi_device_interface_config_t si8380s_1_cfg = {
	.command_bits = 0,
	.address_bits = 0,
	.dummy_bits = 0,
	.mode = 2,
	.clock_source = SPI_CLK_SRC_DEFAULT,
	.duty_cycle_pos = 128,
	.cs_ena_pretrans = 0,
	.cs_ena_posttrans = 0,
	.clock_speed_hz = 1000000, // 1 Mhz
	.spics_io_num = SPI_CS_2,
	.queue_size = 5,
};
spi_device_interface_config_t si8380s_2_cfg = {
	.command_bits = 0,
	.address_bits = 0,
	.dummy_bits = 0,
	.mode = 2,
	.clock_source = SPI_CLK_SRC_DEFAULT,
	.duty_cycle_pos = 128,
	.cs_ena_pretrans = 0,
	.cs_ena_posttrans = 0,
	.clock_speed_hz = 1000000, // 1 Mhz
	.spics_io_num = SPI_CS_1,
	.queue_size = 5,
};
spi_transaction_t t_1, t_2;
uint8_t sendbuf_1[1] = {0x00}, sendbuf_2[1] = {0x00};
uint8_t recvbuf_1[1] = {0x00}, recvbuf_2[1] = {0x00};

/* si8380 (DI2-9)
*/
uint8_t read_spi_1(void) { 
	t_1.length = 8;
	t_1.tx_buffer = sendbuf_1;
	t_1.rx_buffer = recvbuf_1;
	sendbuf_1[0] |= (0b1 << 6);
	ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
	sendbuf_1[0] = 0x00; // CHAN_STATUS
	ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
	sendbuf_1[0] = 0x00;
	ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
	return recvbuf_1[0];
}

/* si8380 (DI10,11,17,19-23)
*/
uint8_t read_spi_2(void) { 
	t_2.length = 8;
	t_2.tx_buffer = sendbuf_2;
	t_2.rx_buffer = recvbuf_2;
	sendbuf_2[0] |= (0b1 << 6);
	ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
	sendbuf_2[0] = 0x00; // CHAN_STATUS
	ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
	sendbuf_2[0] = 0x00;
	ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
	return recvbuf_2[0];
}

void slave_gpio_spi_di_task(void* arg) {
	while(1) {
		slave_gpio_di[0] = gpio_get_level(DI_0);
		slave_gpio_di[1] = gpio_get_level(DI_1);
		slave_gpio_di[2] = gpio_get_level(DI_12);
		slave_gpio_di[3] = gpio_get_level(DI_13);
		slave_gpio_di[4] = gpio_get_level(DI_14);
		slave_gpio_di[5] = gpio_get_level(DI_15);
		slave_gpio_di[6] = gpio_get_level(DI_16);
		slave_gpio_di[7] = gpio_get_level(DI_18);
		slave_spi_di0 = read_spi_1(); // ESP_LOGI(SLAVE_SPI_DI_TAG, "SPI0: 0x%02X", slave_spi_di0);
		slave_spi_di1 = read_spi_2(); // ESP_LOGI(SLAVE_SPI_DI_TAG, "SPI1: 0x%02X", slave_spi_di1);
		// slave_to_master_buffer[2] -> at slave: DI_7, DI_6, DI_5, DI_4, DI_3, DI_2, DI_1, DI_0
		// slave_to_master_buffer[1] -> at slave: DI_15, DI_14, DI_13, DI_12, DI_11, DI_10, DI_09, DI_08
		// slave_to_master_buffer[0] -> at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
		slave_to_master_buffer[2] = 0x00 | slave_gpio_di[0] | (slave_gpio_di[1] << 1);
		slave_to_master_buffer[2] |= (slave_spi_di0 << 2);
		slave_to_master_buffer[1] = 0x00 | (slave_gpio_di[2] << 4) | (slave_gpio_di[3] << 5) | (slave_gpio_di[4] << 6) | (slave_gpio_di[5] << 7);
		slave_to_master_buffer[1] |= (slave_spi_di0 >> 6);
		slave_to_master_buffer[0] = 0x00 | (slave_gpio_di[6]) | (slave_gpio_di[7] << 2);
		slave_to_master_buffer[0] |= ((slave_spi_di1 & 0x04) >> 1) | (slave_spi_di1 & 0xF8);
		vTaskDelay(pdMS_TO_TICKS(10)); 
	}
	vTaskDelete(NULL);
}


/* -------------------- slave i2c -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}
i2c_slave_config_t slv_conf = {
	.i2c_port = I2C_NUM_0,
	.sda_io_num = I2C_SDA,
	.scl_io_num = I2C_SCL,
	.clk_source = I2C_CLK_SRC_DEFAULT,
	.send_buf_depth = 256,
	.slave_addr = 0x0A,
	.addr_bit_len = I2C_ADDR_BIT_LEN_7,
};
i2c_slave_dev_handle_t slv_handle;
i2c_slave_event_callbacks_t cbs = {
	.on_recv_done = i2c_slave_rx_done_callback,
};
i2c_slave_rx_done_event_data_t rx_data;

esp_err_t i2c_read() { 
	uint8_t* master_to_slave_buffer = (uint8_t *)(malloc(4)); // Allocated Memory to receive data from Master ESP
	if (master_to_slave_buffer == NULL) {
		ESP_LOGE(I2C_SLAVE_TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	memset(master_to_slave_buffer, 0, 5);  // Initialize buffer
	QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    if (receive_queue == NULL) {
		free(master_to_slave_buffer);
        ESP_LOGE(I2C_SLAVE_TAG, "Failed to create queue");
        return 0; 
    }
    esp_err_t err = i2c_slave_register_event_callbacks(slv_handle, &cbs, receive_queue);
    if (err != ESP_OK) {
		free(master_to_slave_buffer);
        vQueueDelete(receive_queue);
        return err;
    }
    err = i2c_slave_receive(slv_handle, master_to_slave_buffer, 10);
    if (err != ESP_OK) {
		free(master_to_slave_buffer);
        vQueueDelete(receive_queue);
        return err;
    }
    if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(100)) == pdPASS) {
		// for (int i = 0; i < 5; i++) {
        //     ESP_LOGI(I2C_SLAVE_TAG, "Byte %d: 0x%02X", i, master_to_slave_buffer[i]);
        // }
		if(*(master_to_slave_buffer) == 0xBB) {
			slave_do = *(master_to_slave_buffer + 1);
			slave_pass1_pass2_bms = *(master_to_slave_buffer + 2);
			battery_level = *(master_to_slave_buffer + 3);
			new_i2c_incoming = true;
		}
		else if(*(master_to_slave_buffer) == 0xFF) {
			esp_err_t err = i2c_slave_transmit(slv_handle, slave_to_master_buffer, 6, -1);
			if(err != ESP_OK) {
				return err;
			}
		}
    } 
	// ESP_LOGI(SLAVE_SPI_DI_TAG, "State 0: 0x%02X\n", slave_to_master_buffer[0]);
	// ESP_LOGI(SLAVE_SPI_DI_TAG, "State 1: 0x%02X\n", slave_to_master_buffer[1]);
	// ESP_LOGI(SLAVE_SPI_DI_TAG, "State 2: 0x%02X\n", slave_to_master_buffer[2]);
	// ESP_LOGI(SLAVE_SPI_DI_TAG, "State 3: 0x%02X\n", slave_to_master_buffer[3]);
	// ESP_LOGI(SLAVE_SPI_DI_TAG, "State 4: 0x%02X\n", slave_to_master_buffer[4]);
	free(master_to_slave_buffer); // free allocated memory
    vQueueDelete(receive_queue); // delete the queue to free resources
    return ESP_OK;
}

void i2c_task(void *arg) {
	while(1) {
		i2c_read();
		if(new_i2c_incoming) {
			// ESP_LOGI(I2C_SLAVE_TAG, "Slave do: 0x%04X\n", slave_do);
			// ESP_LOGI(I2C_SLAVE_TAG, "Navigation intent index: %d", navigation_intent_index);
			// ESP_LOGI(I2C_SLAVE_TAG, "Battery level: %d", battery_level);
			if (battery_level < LOW_BAT_THRESHOLD) isBatterylow = true;
			else isBatterylow = false;
			if (battery_level >= 80) indexBattery = 3;
			else if (battery_level >= 50 && battery_level < 80) indexBattery = 2;
			else if (battery_level < 50) indexBattery = 1;
			else indexBattery = 0;
			isBlock = false; isLeft = false; isRight = false; isMove = false; isFail = false; isAutochargengaged = false;
			// switch (navigation_intent_index) {
			// 	case 11:
			// 		isBlock = true; 
			// 		break;
			// 	case 12:
			// 		isLeft = true; 
			// 		break;
			// 	case 13:
			// 		isRight = true; 
			// 		break;
			// 	case 14:
			// 		isMove = true; 
			// 		break;
			// 	case 15:
			// 		isFail = true;
			// 		break;
			// 	case 16:
			// 		isAutochargengaged = true;
			// 		break;
			// 	default:
			// 		isBlock = false;
			// 		isLeft = false;
			// 		isRight = false;
			// 		isMove = false;
			// 		isFail = false; 
			// 		isAutochargengaged = false;
			// }
			new_slave_gpio_do = true;
			new_i2c_incoming = false;
		}
	}
	vTaskDelete(NULL);
}


/* -------------------- DMX -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uart_config_t uart_config = {
	.baud_rate = 250000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
};
static QueueHandle_t dmx_rx_queue; 

void createBuffer(bool TmpBufferCreate) {
    // Manage deletion of the Tmp Buffeur if not any more needed 
    if((!TmpBufferCreate) && (tmp_dmx_data != NULL)) {
        free(tmp_dmx_data);
        tmp_dmx_data = NULL;
    }
    // Check if buffer is already existing
    if(dmx_data != NULL) {
        dmx_data = (uint8_t *) realloc(dmx_data, sizeof(uint8_t)*(_NbChannels+1));
    } else {
         dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
    }
    memset(dmx_data,0,sizeof(uint8_t)*(_NbChannels+1));
    // Manage the temporrary buffer if needed
    if(TmpBufferCreate) {
        if(tmp_dmx_data != NULL) {
            tmp_dmx_data = (uint8_t *) realloc(tmp_dmx_data, sizeof(uint8_t)*(_NbChannels+1));
        } else {
            tmp_dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
        }
    }
}

void dmx_write(uint16_t channel, uint8_t value) {
    if(channel < 1 || channel > 512) return;     // restrict acces to dmx array to valid values
    dmx_data[channel] = value;
}

void dmx_write_all(uint8_t * data, uint16_t start, size_t size) {
    if(start < 1 || start > 512 || start + size > 513) return; // restrict acces to dmx array to valid values
    memcpy((uint8_t *)dmx_data + start, data, size);
}

void dmx_uart_task(void* arg) {
    uint8_t start_code = 0x00;
    for(;;) {
        uart_wait_tx_done(DMX_UART_NUM, 1000); // wait till uart is ready
        uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV); // set line to inverse, creates break signal
        esp_rom_delay_us(184); // wait break time
        uart_set_line_inverse(DMX_UART_NUM, 0); // disable break signal
        esp_rom_delay_us(24); // wait mark after break
		//ESP_LOGI(DMX_RS485_TAG, "Write data to uart");
        uart_write_bytes(DMX_UART_NUM, (const char*) &start_code, 1); // write start code
        uart_write_bytes(DMX_UART_NUM, (const char*) dmx_data+1, 512); // transmit the dmx data
    }
}

void set_led(const uint8_t color_cmd) {
	uint8_t zero_array[32] = {0};
	dmx_write_all(zero_array, start_channel, 32);
	switch (color_cmd) {
		case 1: // RED RGB: 255,0,0
			for (int i = start_channel; i <= start_channel + 32; i+=4) {
				dmx_write(i, 255);
			}
			break;
		case 2: // YELLOW RGB: 255,222,0
			for (int i = start_channel; i <= start_channel + 32; i+=4) {
				dmx_write(i, 255);
				dmx_write(i + 1, 222);
			}
			break;
		case 3: // BLUE RGB: 0,0,255
			for (int i = start_channel; i <= start_channel + 32; i+=4) {
				dmx_write(i + 2, 255);
			}
			break;
		case 4: // GREEN RGB: 0,255,0
			for (int i = start_channel; i <= start_channel + 32; i+=4) {
				dmx_write(i + 1, 255);
			}
			break;
		case 5: // MIX
			// RED
			dmx_write(start_channel + 0, 255);
			dmx_write(start_channel + 4, 255);
			// BLUE
			dmx_write(start_channel + 10, 255);
			dmx_write(start_channel + 14, 255);
			// GREEN
			dmx_write(start_channel + 17, 255);
			dmx_write(start_channel + 21, 255);
			// YELLOW
			dmx_write(start_channel + 24, 255);
			dmx_write(start_channel + 25, 222);
			dmx_write(start_channel + 28, 255);
			dmx_write(start_channel + 29, 222);
			break;	
		case 101: // red color wavering (RGB: 255,0,0)
			if (led_wave_dir == true) {
				for (int i = start_channel; i <= start_channel + 31; i+=4) {
					dmx_write(i, temp_brightness);
				}
				temp_brightness += 2;
				if (temp_brightness > 253) led_wave_dir = false;
				break;
			}
			else {
				for (int i = start_channel; i <= start_channel + 31; i+=4) {
                	dmx_write(i, temp_brightness);
            	}
				temp_brightness -= 2;
				if (temp_brightness < 30) led_wave_dir = true;
				break;
			}
			break;
		case 102: // dark red color blinking (RGB: 70,0,0)
       		if (led_wave_dir == true) {
				for (int i = start_channel; i <= start_channel + 31; i+=4) {
    	        	dmx_write(i, 70);
        		}
				temp_counter += 1;
				if (temp_counter > 100) led_wave_dir = false;
				break;
			} 
			else {
				temp_counter -= 1;
				if (temp_counter < 50) led_wave_dir = true;
				break;
			}
         	break;
		case 103: // dark red color horse lamp (RGB: 70,0,0)
			switch (horse_sequence) {
				case 0:
					for (int i = start_channel; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 1;
					break;
				case 1:
					for (int i = start_channel + 4; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 2;
					break;
				case 2:
					for (int i = start_channel + 8; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 3;
					break;
				case 3:
					for (int i = start_channel + 12; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 0;
					break;
			}
			break;
		case 104: // orange color wavering (RGB: 255,128,0)
			if (led_wave_dir == true) {
				for (int i = start_channel; i <= start_channel + 31; i+=4) {
					dmx_write(i, temp_brightness);
					dmx_write(i + 1, temp_brightness*0.5);
				}
				temp_brightness += 2;
				if (temp_brightness > 235) led_wave_dir = false;
				break;
			}
			else {
				for (int i = start_channel; i <= start_channel + 31; i+=4) {
                	dmx_write(i, temp_brightness);
					dmx_write(i + 1, temp_brightness*0.5);
            	}
				temp_brightness -= 2;
				if (temp_brightness < 30) led_wave_dir = true;
				break;
			}
			break;
		case 105: // white color wavering (RGBA: 0,0,0,255)
			if (led_wave_dir == true) {
				for (int i = start_channel + 3; i <= start_channel + 31; i+=4) {
					dmx_write(i, temp_brightness);
				}
				temp_brightness += 2;
				if (temp_brightness > 253) led_wave_dir = false;
				break;
			}
			else {
				for (int i = start_channel + 3; i <= start_channel + 31; i+=4) {
                	dmx_write(i, temp_brightness);
            	}
				temp_brightness -= 2;
				if (temp_brightness < 30) led_wave_dir = true;
				break;
			}
			break;
		case 106: // dark orange color horse lamp (RGB: 70,45,0)
			switch (horse_sequence) {
				case 0:
					for (int i = start_channel; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 45);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 1;
					break;
				case 1:
					for (int i = start_channel + 4; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 45);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 2;
					break;
				case 2:
					for (int i = start_channel + 8; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 45);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 3;
					break;
				case 3:
					for (int i = start_channel + 12; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 45);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 0;
					break;
			}
			break;
		case 107: // dark yellow color horse lamp (RGB: 70,70,0)
			switch (horse_sequence) {
				case 0:
					for (int i = start_channel; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 1;
					break;
				case 1:
					for (int i = start_channel + 4; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 2;
					break;
				case 2:
					for (int i = start_channel + 8; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 3;
					break;
				case 3:
					for (int i = start_channel + 12; i <= start_channel + 31; i+=16) {
						dmx_write(i, 70);
						dmx_write(i + 1, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 0;
					break;
			}
			break;
		case 108: // dark green color horse lamp (RGB: 35,70,0)
			switch (horse_sequence) {
				case 0:
					for (int i = start_channel; i <= start_channel + 31; i+=16) {
						dmx_write(i, 35);
						dmx_write(i + 1, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 1;
					break;
				case 1:
					for (int i = start_channel + 4; i <= start_channel + 31; i+=16) {
						dmx_write(i, 35);
						dmx_write(i + 1, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 2;
					break;
				case 2:
					for (int i = start_channel + 8; i <= start_channel + 31; i+=16) {
						dmx_write(i, 35);
						dmx_write(i + 1, 70);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 3;
					break;
				case 3:
					for (int i = start_channel + 12; i <= start_channel + 31; i+=16) {
						dmx_write(i, 35);
						dmx_write(i + 1, 70);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 0;
					break;
			}
			break;
		case 111: // pink purple color horse lamp (RGB: 255,0,255)
			switch (horse_sequence) {
				case 0:
					for (int i = start_channel; i <= start_channel + 31; i+=16) {
						dmx_write(i, 255);
						dmx_write(i + 2, 255);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 1;
					break;
				case 1:
					for (int i = start_channel + 4; i <= start_channel + 31; i+=16) {
						dmx_write(i, 255);
						dmx_write(i + 2, 255);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 2;
					break;
				case 2:
					for (int i = start_channel + 8; i <= start_channel + 31; i+=16) {
						dmx_write(i, 255);
						dmx_write(i + 2, 255);
        			}
					temp_counter += 1;
					if (temp_counter > 65) horse_sequence = 3;
					break;
				case 3:
					for (int i = start_channel + 12; i <= start_channel + 31; i+=16) {
						dmx_write(i, 255);
						dmx_write(i + 2, 255);
        			}
					temp_counter -= 1;
					if (temp_counter < 50) horse_sequence = 0;
					break;
			}
			break;
		case 112: // orange color blinking left (RGB: 255,128,0)
       		if (led_wave_dir == true) {
				for (int i = start_channel + 16; i <= start_channel + 31; i+=4) {
    	        	dmx_write(i, 255);
					dmx_write(i+1, 128);
        		}
				temp_counter += 1;
				if (temp_counter > 100) led_wave_dir = false;
				break;
			} 
			else {
				temp_counter -= 1;
				if (temp_counter < 50) led_wave_dir = true;
				break;
			}
			break;
		case 113: // orange color blinking right (RGB: 255,128,0)
			if (led_wave_dir == true) {
				for (int i = start_channel; i <= start_channel + 15; i+=4) {
    	        	dmx_write(i, 255);
					dmx_write(i+1, 128);
        		}
				temp_counter += 1;
				if (temp_counter > 100) led_wave_dir = false;
				break;
			} 
			else {
				temp_counter -= 1;
				if (temp_counter < 50) led_wave_dir = true;
				break;
			}
			break;
		case 114: // blue color wavering (RGB: 0,0,255)
			if (led_wave_dir == true) {
				for (int i = start_channel + 2; i <= start_channel + 31; i+=4) {
					dmx_write(i, temp_brightness);
				}
				temp_brightness += 2;
				if (temp_brightness > 253) led_wave_dir = false;
				break;
			}
			else {
				for (int i = start_channel + 2; i <= start_channel + 31; i+=4) {
                	dmx_write(i, temp_brightness);
            	}
				temp_brightness -= 2;
				if (temp_brightness < 30) led_wave_dir = true;
				break;
			}
			break;
		default:
			for (int i = 1; i <= 31; i++) dmx_write(i, 10);
			break;
	}
	vTaskDelay(pdMS_TO_TICKS(5));
}


/* -------------------- twai -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN2_TX, CAN2_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// void nc_twai_init(void) {
//     twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN2_TX, CAN2_RX, TWAI_MODE_NORMAL);
//     twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//     twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//     if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
// 		ESP_LOGI(NC_TWAI_TAG, "Driver installed");
// 	}
//     else {
//         ESP_LOGE(NC_TWAI_TAG, "Failed to install driver");
//         return;
//     }
//     if (twai_start() == ESP_OK) {
// 		ESP_LOGI(NC_TWAI_TAG, "Driver started");
// 	}
// 	else {
//         ESP_LOGE(NC_TWAI_TAG, "Failed to start driver");
//         return;
//     }
// }

// twai_message_t message = {
//     .extd = 0, // Standard vs extended format
//     .rtr = 0, // Data vs RTR frame
//     .ss = 1, // Whether the message is single shot (i.e., does not repeat on error)
//     .self = 0, // Whether the message is a self reception request (loopback)
//     .dlc_non_comp = 0, // DLC is less than 8
//     .identifier = 0xAAAA, // Message ID and payload
//     .data_length_code = 8,
//     .data = {0, 1, 2, 3},
// };

// void transmit_TWAI(void* arg) {
//     while(1) {
//         if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) { 
// 			// ESP_LOGI(NC_TWAI_TAG, "Message queued for transmission");
//         } 
// 		else { 
// 			// ESP_LOGE(NC_TWAI_TAG, "Failed to queue message for transmission");   
//         }
// 		vTaskDelay(pdMS_TO_TICKS(50));
//     }
//     vTaskDelete(NULL);
// }

// void receive_TWAI(void* arg) {
// 	twai_message_t message_rcv;
// 	while(1) {
// 		if (twai_receive(&message_rcv, pdMS_TO_TICKS(1000)) == ESP_OK) {
// 			for(int i = 0; i < message_rcv.data_length_code; i++) { 
// 				// ESP_LOGI(NC_TWAI_TAG, "Message data[%d]: %d", i, message_rcv.data[i]);
// 				message.data[i] = message_rcv.data[i];
// 			}
// 		} 
// 	}
// 	vTaskDelete(NULL);
// }

void nc_twai_task(void* arg) { // echo back received twai message, with identifier +1
	while(1) {
		twai_message_t receive_message, transmit_message;
		if (twai_receive(&receive_message, pdMS_TO_TICKS(100)) == ESP_OK) {
			transmit_message.identifier = receive_message.identifier + 1;
			for (int twai_i = 0; twai_i < receive_message.data_length_code; twai_i++) {
				transmit_message.data[twai_i] = receive_message.data[twai_i];
			}
			ESP_ERROR_CHECK(twai_transmit(&transmit_message, pdMS_TO_TICKS(100)));
		}
		else {}
		// if (crash_i == 9) {  // check for crash
		// 	printf("Now esp is crashed\n");
		// 	assert(0);
		// }
		// else crash_i ++;
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}


/* -------------------- app main -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void app_main(void) {
	slave_reason = esp_reset_reason();
	slave_to_master_buffer[5] = (uint8_t) slave_reason;
	
//	gpio_reset_pin(BMS);
//	gpio_reset_pin(PASS_1);
//	gpio_reset_pin(PASS_2);
	gpio_reset_pin(DI_12);
	gpio_reset_pin(DI_13);
	gpio_reset_pin(DI_14);
	gpio_reset_pin(DI_15);
	gpio_reset_pin(DI_16);
	gpio_reset_pin(DI_18);
	gpio_reset_pin(BOOTKEY);
	gpio_set_direction(DO_0, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_3, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_4, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_5, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_8, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_9, GPIO_MODE_OUTPUT);
	gpio_set_direction(BMS, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DI_12, GPIO_MODE_INPUT);
	gpio_set_direction(DI_13, GPIO_MODE_INPUT);
	gpio_set_direction(DI_14, GPIO_MODE_INPUT);
	gpio_set_direction(DI_15, GPIO_MODE_INPUT);
	gpio_set_direction(DI_16, GPIO_MODE_INPUT);
	gpio_set_direction(DI_18, GPIO_MODE_INPUT);
	gpio_set_direction(BOOTKEY, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BOOTKEY, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(I2C_SDA, GPIO_PULLUP_ONLY); 
	gpio_set_pull_mode(I2C_SCL, GPIO_PULLUP_ONLY); 
	vTaskDelay(pdMS_TO_TICKS(50));

//	gpio_set_level(BMS, 1);

//    nc_twai_init();
//	xTaskCreatePinnedToCore(transmit_TWAI, "transmit_TWAI", 2048, NULL, 1, NULL, 0);
//	xTaskCreatePinnedToCore(receive_TWAI, "receive_TWAI", 2048, NULL, 1, NULL, 0);

//	ESP_LOGI(NC_TWAI_TAG, "Install twai driver");
//	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
//	ESP_LOGI(NC_TWAI_TAG, "Start twai driver");
//	ESP_ERROR_CHECK(twai_start());

	ESP_LOGI(BOOTKEY_TAG, "Create bootkey task");
	xTaskCreate(check_bootkey, "check_bootkey", 4096, NULL, 5, NULL);

//	ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt unit");
//	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Set glitch filter");
//	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt channels");
//	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
//	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Set edge and level actions for pcnt channels");
//	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Enable pcnt unit");
//	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Clear pcnt unit");
//	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
//	ESP_LOGI(ESTOP_PCNT_TAG, "Start pcnt unit");
//	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
	
	ESP_LOGI(SLAVE_SPI_DI_TAG, "Initialize spi bus");
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
	ESP_LOGI(SLAVE_SPI_DI_TAG, "Add device to spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &si8380s_1_cfg, &handle_1));
	ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &si8380s_2_cfg, &handle_2));
	ESP_LOGI(SLAVE_SPI_DI_TAG, "Create slave spi di task");
	xTaskCreate(slave_gpio_spi_di_task, "slave_gpio_spi_di_task", 4096, NULL, 5, NULL);

	ESP_LOGI(I2C_SLAVE_TAG, "Add i2c slave device");
	ESP_ERROR_CHECK(i2c_new_slave_device(&slv_conf, &slv_handle));
	ESP_LOGI(I2C_SLAVE_TAG, "Create i2c task");
	xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);

//	ESP_LOGI(DMX_RS485_TAG, "Configure uart parameters");
//	ESP_ERROR_CHECK(uart_param_config(DMX_UART_NUM, &uart_config));
//	ESP_LOGI(DMX_RS485_TAG, "Set uart pins");
//	ESP_ERROR_CHECK(uart_set_pin(DMX_UART_NUM, RS3_TX, RS3_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
//	ESP_LOGI(DMX_RS485_TAG, "Install uart driver");
//	ESP_ERROR_CHECK(uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0));
//	esp_rom_gpio_pad_select_gpio(RS3_DE);
// 	gpio_set_direction(RS3_DE, GPIO_MODE_OUTPUT);
//	gpio_set_level(RS3_DE, 1);
//	_StartDMXAddr = 1;
//	_NbChannels = 512;
//	createBuffer(false);
//	ESP_LOGI(DMX_RS485_TAG, "Create dms uart task");
//	xTaskCreate(dmx_uart_task, "dmx_uart_task", 4096, NULL, 5, NULL);

	ESP_LOGI(SLAVE_DO_TAG, "Create slave do task");		
	xTaskCreatePinnedToCore(slave_gpio_do_task, "slave_gpio_do_task", 4096, NULL, 5, NULL, 1);
	
//	ESP_LOGI(AMR_STATE_TAG, "Create amr state task");
//	xTaskCreate(amr_state_machine, "amr_state_machine", 4096, NULL, 7, NULL);

//	ESP_LOGI(FUNCTIONAL_TAG, "Create check functional input task");
//	xTaskCreate(check_functional_inputs, "check_functional_inputs", 4096, NULL, 5, NULL);
	
	vTaskDelay(pdMS_TO_TICKS(2000)); // this delay is neccessary to let Master start sending PulseA/B before check estop routine
//	ESP_LOGI(ESTOP_PCNT_TAG, "Create check estop task");
//	xTaskCreate(check_estop, "check_estop", 4096, NULL, 9, NULL);

//	ESP_LOGI(NC_TWAI_TAG, "Create nc twai task");
//	xTaskCreate(nc_twai_task, "nc_uart_task", 4096, NULL, 1, NULL);

    esp_intr_dump(NULL);
}
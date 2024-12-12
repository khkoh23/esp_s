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

static const char *BOOTKEY_TAG = "bootkey_gpio";
static const char *ESTOP_PCNT_TAG = "estop_pcnt";
static const char *SLAVE_SPI_DI_TAG = "slave_spi_di";
static const char *SLAVE_DO_TAG = "slave_do";
static const char *I2C_SLAVE_TAG = "i2c_slave";
static const char *DMX_UART_TAG = "dmx_uart";
static const char *NC_TWAI_TAG = "nc_twai";
static const char *AMR_STATE_TAG = "amr_state";


/* -------------------- global -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

bool slave_gpio_di[8] = {};
uint8_t slave_spi_di0, slave_spi_di1;
uint8_t slave_to_master_buffer[6] = {
	0b00000000, // at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
	0b00000000, // at slave: DI_15, DI_14, DI_13, DI_12, DI_11, DI_10, DI_09, DI_08
	0b00000000, // at slave: DI_7, DI_6, DI_5, DI_4, DI_3, DI_2, DI_1, DI_0
	0b00000000, // at slave: DO_9. DO_8. DO_5. DO_4. DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, x, x, BOOTKEY
	0b00000000, // interval count
};
uint8_t slave_do = 0; // MSB to LSB: DO_9, DO_8, DO_5 to DO_0
uint8_t slave_pass1_pass2_bms; //MSB to LSB: x, x, x, x, x, PASS_2, PASS_1, BMS
uint8_t battery_level = 0;
bool new_i2c_incoming;
bool new_slave_gpio_do = false;

int pulse_count = 0, prev_pulse_count;
const int dmx_uart_buffer_size = 127;
const uint8_t dmx_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks


/* ---------------FUNCTION PROTOTYPES---------------- */
void check_estop(void* arg);
uint8_t read_spi_1();
uint8_t read_spi_2();
void slave_gpio_spi_di_task(void* arg);
esp_err_t i2c_read();
void i2c_task(void *arg);
void slave_gpio_do_task(void* arg);


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
		slave_to_master_buffer[5] = pulse_count - prev_pulse_count;
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
			gpio_set_level(BMS, slave_pass1_pass2_bms & 0x01);
			gpio_set_level(PASS_1, slave_pass1_pass2_bms & 0x02);
			gpio_set_level(PASS_2, slave_pass1_pass2_bms & 0x04);
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
		slave_to_master_buffer[2] = 0x00 | slave_gpio_di[0] | (slave_gpio_di[1] << 1);
		slave_to_master_buffer[2] |= (slave_spi_di0 << 2);
		slave_to_master_buffer[1] = 0x00 | (slave_gpio_di[2] << 4) | (slave_gpio_di[3] << 5) | (slave_gpio_di[4] << 6) | (slave_gpio_di[5] << 7);
		slave_to_master_buffer[1] |= (slave_spi_di0 >> 6);
		slave_to_master_buffer[1] |= (slave_spi_di1 & 0x03) << 2; // DI_11 and DI_10
		slave_to_master_buffer[0] = 0x00 | (slave_gpio_di[6]) | (slave_gpio_di[7] << 2);
		slave_to_master_buffer[0] |= ((slave_spi_di1 & 0x04) >> 1) | (slave_spi_di1 & 0xF8);
		slave_to_master_buffer[4] = 0x00 | gpio_get_level(BOOTKEY); // connectivity test only
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
	free(master_to_slave_buffer); // free allocated memory
    vQueueDelete(receive_queue); // delete the queue to free resources
    return ESP_OK;
}

void i2c_task(void *arg) {
	while(1) {
		i2c_read();
		if(new_i2c_incoming) {
			new_slave_gpio_do = true;
			new_i2c_incoming = false;
		}
	}
	vTaskDelete(NULL);
}


/* -------------------- DMX -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uart_config_t dmx_uart_config = {
	.baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_DEFAULT,
};

static void dmx_uart_help_echo_send(const uart_port_t port, const char* str, uint8_t length) {
    if (uart_write_bytes(port, str, length) != length) {
    }
}

void dmx_uart_task(void *arg) {  
	while (1) {
		ESP_ERROR_CHECK(uart_flush(UART_NUM_2));
		uint8_t* data = (uint8_t*) malloc(dmx_uart_buffer_size); // Allocate buffers for UART
		int len = uart_read_bytes(UART_NUM_2, data, dmx_uart_buffer_size, (100/portTICK_PERIOD_MS)); // Read data from UART
		if (len > 0) { // Write data back to UART
			dmx_uart_help_echo_send(UART_NUM_2, "\r\n", 2);
			char prefix[] = "Shoalbot's RS485_3 received: [";
			dmx_uart_help_echo_send(UART_NUM_2, prefix, (sizeof(prefix) - 1));
			for (int i = 0; i < len; i++) {
				dmx_uart_help_echo_send(UART_NUM_2, (const char*) &data[i], 1); 
				if (data[i] == '\r') { // Add a Newline character if get a return charater from paste
					dmx_uart_help_echo_send(UART_NUM_2, "\n", 1);
				}
			}
			dmx_uart_help_echo_send(UART_NUM_2, "]\r\n", 3);
		}
		else { // Echo a "." to show alive while waiting for input
			dmx_uart_help_echo_send(UART_NUM_2, ".", 1); 
			ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, (10/portTICK_PERIOD_MS)));
		}
		free(data);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- nc twai -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN2_TX, CAN2_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

twai_message_t transmit_message = {
	.data_length_code = 8,
	.data = {0, 0, 0, 0, 0, 0, 0, 0},
};

void nc_twai_task(void* arg) { // echo back received twai message, with identifier +2 and each data bytes +2
	while(1) {
		twai_message_t receive_message;
		if (twai_receive(&receive_message, pdMS_TO_TICKS(100)) == ESP_OK) {
			transmit_message.identifier = receive_message.identifier + 2;
			for (int twai_i = 0; twai_i < receive_message.data_length_code; twai_i++) {
				transmit_message.data[twai_i] = receive_message.data[twai_i] + 2;
			}
			ESP_ERROR_CHECK(twai_transmit(&transmit_message, pdMS_TO_TICKS(100)));
		}
		else {}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- app main -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void app_main(void) {
	ESP_ERROR_CHECK(gpio_reset_pin(BMS));
	ESP_ERROR_CHECK(gpio_reset_pin(PASS_1));
	ESP_ERROR_CHECK(gpio_reset_pin(PASS_2));
	ESP_ERROR_CHECK(gpio_reset_pin(CAN2_TX)); //neccessary reset MTCK
	ESP_ERROR_CHECK(gpio_reset_pin(CAN2_RX)); //MTDO
	ESP_ERROR_CHECK(gpio_reset_pin(DO_8)); //MTDI
	ESP_ERROR_CHECK(gpio_reset_pin(DO_9)); //MTMS
	ESP_ERROR_CHECK(gpio_reset_pin(DI_12));
	ESP_ERROR_CHECK(gpio_reset_pin(DI_13));
	ESP_ERROR_CHECK(gpio_reset_pin(DI_14));
	ESP_ERROR_CHECK(gpio_reset_pin(DI_15));
	ESP_ERROR_CHECK(gpio_reset_pin(DI_16));
	ESP_ERROR_CHECK(gpio_reset_pin(DI_18));
	ESP_ERROR_CHECK(gpio_reset_pin(BOOTKEY));

	ESP_LOGI(SLAVE_DO_TAG, "Configure gpio direction");	
	ESP_ERROR_CHECK(gpio_set_direction(DO_0, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_1, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_2, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_3, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_4, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_5, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_8, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_9, GPIO_MODE_OUTPUT));
	ESP_LOGI(AMR_STATE_TAG, "Configure gpio direction");
	ESP_ERROR_CHECK(gpio_set_direction(BMS, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(PASS_1, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(PASS_2, GPIO_MODE_OUTPUT));
	ESP_LOGI(SLAVE_SPI_DI_TAG, "Configure gpio direction");
	ESP_ERROR_CHECK(gpio_set_direction(DI_12, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DI_13, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DI_14, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DI_15, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DI_16, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DI_18, GPIO_MODE_INPUT));
	ESP_LOGI(BOOTKEY_TAG, "Configure gpio direction and pull-up resistor");
	ESP_ERROR_CHECK(gpio_set_direction(BOOTKEY, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_pull_mode(BOOTKEY, GPIO_PULLUP_ONLY));
	ESP_LOGI(I2C_SLAVE_TAG, "Configure gpio pull-up resistor");
	ESP_ERROR_CHECK(gpio_set_pull_mode(I2C_SDA, GPIO_PULLUP_ONLY));
	ESP_ERROR_CHECK(gpio_set_pull_mode(I2C_SCL, GPIO_PULLUP_ONLY));
	vTaskDelay(pdMS_TO_TICKS(50));

	ESP_LOGI(NC_TWAI_TAG, "Install twai driver");
	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(NC_TWAI_TAG, "Start twai driver");
	ESP_ERROR_CHECK(twai_start());

	ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt unit");
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
	ESP_LOGI(ESTOP_PCNT_TAG, "Set glitch filter");
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
	ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt channels");
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
	ESP_LOGI(ESTOP_PCNT_TAG, "Set edge and level actions for pcnt channels");
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_LOGI(ESTOP_PCNT_TAG, "Enable pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
	ESP_LOGI(ESTOP_PCNT_TAG, "Clear pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	ESP_LOGI(ESTOP_PCNT_TAG, "Start pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
	
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

	ESP_LOGI(DMX_UART_TAG,"Install dmx uart driver");
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, dmx_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
	ESP_LOGI(DMX_UART_TAG,"Configure dmx uart parameter");
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &dmx_uart_config));
	ESP_LOGI(DMX_UART_TAG,"Assign signals of dmx uart peripheral to gpio pins");
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, RS3_TX, RS3_RX, RS3_DE, UART_PIN_NO_CHANGE));
	ESP_LOGI(DMX_UART_TAG,"Set dmx uart to rs485 half duplex mode");
	ESP_ERROR_CHECK(uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX));
	ESP_LOGI(DMX_UART_TAG,"Set dmx uart read threshold timeout for TOUT feature");
	ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_2, dmx_uart_read_tout));
	ESP_LOGI(DMX_UART_TAG, "Discard all data in the nc uart rx buffer");
	ESP_ERROR_CHECK(uart_flush(UART_NUM_2));
	ESP_LOGI(DMX_UART_TAG, "Create dmx uart task");
	xTaskCreate(dmx_uart_task, "dmx_uart_task", 4096, NULL, 1, NULL);
	ESP_LOGI(SLAVE_DO_TAG, "Create slave do task");		
	xTaskCreatePinnedToCore(slave_gpio_do_task, "slave_gpio_do_task", 4096, NULL, 5, NULL, 1);
	vTaskDelay(pdMS_TO_TICKS(2000)); // this delay is neccessary to let Master start sending PulseA/B before check estop routine
	ESP_LOGI(ESTOP_PCNT_TAG, "Create check estop task");
	xTaskCreate(check_estop, "check_estop", 4096, NULL, 9, NULL);

	ESP_LOGI(NC_TWAI_TAG, "Create nc twai task");
	xTaskCreate(nc_twai_task, "nc_twai_task", 4096, NULL, 1, NULL);

    esp_intr_dump(NULL);
}
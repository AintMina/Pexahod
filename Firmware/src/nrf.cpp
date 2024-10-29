#include "nrf.h"
#include "nrf_msg.h"
#include "pins.h"
#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    #include "nrf24_driver.h"
}



// const uint8_t address[6] = {0x30, 0x30, 0x30, 0x30, 0x30};
const uint8_t address[6] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
struct nrf_message_t nrf_package;


// provides access to driver functions
nrf_client_t my_nrf;

void nrf_init() {
    // GPIO pin numbers
    pin_manager_t my_pins;
    my_pins.sck = 6;
    my_pins.copi = 19; 
    my_pins.cipo = 20; 
    my_pins.csn = 21; 
    my_pins.ce = 26;

    /**
     * nrf_manager_t can be passed to the nrf_client_t
     * initialise function, to specify the NRF24L01 
     * configuration. If NULL is passed to the initialise 
     * function, then the default configuration will be used.
     */
    nrf_manager_t my_config;
    // RF Channel 
    my_config.channel = 120;
    // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
    my_config.address_width = AW_5_BYTES;
    // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
    my_config.dyn_payloads = DYNPD_ENABLE;
    // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
    my_config.data_rate = RF_DR_1MBPS;
    // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
    my_config.power = RF_PWR_NEG_12DBM;
    // retransmission count: ARC_NONE...ARC_15RT
    my_config.retr_count = ARC_10RT;
    // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
    my_config.retr_delay = ARD_500US;

    // SPI baudrate
    uint32_t my_baudrate = 5000000;


    // initialise my_nrf
    volatile int temp = nrf_driver_create_client(&my_nrf);
    // configure GPIO pins and SPI
    my_nrf.configure(&my_pins, my_baudrate);
    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);

    my_nrf.rx_destination(DATA_PIPE_0, address);

    // // set to RX Mode
    my_nrf.receiver_mode();
}

void nrf_main(void *pvParameters) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    nrf_init();

    // data pipe number a packet was received on
    uint8_t pipe_number = 0;

	while (1) {
        if (my_nrf.is_packet(&pipe_number)) {
            // read payload
            my_nrf.read_packet(&nrf_package, sizeof(nrf_package));
			uint8_t crc = nrf_calculate_crc(&nrf_package);
        }

		for (int i = 0; i < 255; i++) {
            set_led(1, 0, 0, i);
            vTaskDelay(1);
        }
        for (int i = 0; i < 255; i++) {
            set_led(1, 0, 0, 255-i);
            vTaskDelay(1);
        }
		
		vTaskDelay(1000);
	}
}
/*
 * Accelerometer_AIS3624DQ.cpp
 *
 * Created: 19-12-2017 20:35:16
 * Author : Nishant Sood
 */ 


extern "C" {
	#include "asf.h"
	};
	
#include "AIS3624DQ.h"

void configure_usart(void);

//! [module_inst]
struct usart_module usart_instance;
//! [module_inst]

//! [setup]
void configure_usart(void)
{
	//! [setup_config]
	struct usart_config config_usart;
	//! [setup_config]
	//! [setup_config_defaults]
	usart_get_config_defaults(&config_usart);
	//! [setup_config_defaults]

	//! [setup_change_config]
	config_usart.baudrate    = 38400;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	//! [setup_change_config]

	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE,&config_usart);

	//! [setup_set_config]
	while (usart_init(&usart_instance,
	EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	//! [setup_set_config]

	//! [setup_enable]
	usart_enable(&usart_instance);
	//! [setup_enable]
}
//! [setup]


AIS3624DQ _AIS3624DQ(0x18);

void configure_port_pins(void)
{
	//! [setup_1]
	struct port_config config_port_pin;
	//! [setup_1]
	//! [setup_2]
	port_get_config_defaults(&config_port_pin);
	//! [setup_2]

	//! [setup_3]
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	//! [setup_3]
	//! [setup_4]
	port_pin_set_config(BUTTON_0_PIN, &config_port_pin);
	//! [setup_4]

	//! [setup_5]
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	//! [setup_5]
	//! [setup_6]
	port_pin_set_config(LED_0_PIN, &config_port_pin);
	//! [setup_6]
}

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	board_init();
	delay_init();
	configure_usart();
	uint8_t dummy;
	int16_t Xaxis, Yaxis, Zaxis;

	//Configure GPIOs for LED blinking
    configure_port_pins();
	/* LED blinking routine
	port_pin_set_output_level(LED_0_PIN, true);
	delay_ms(500);
	port_pin_set_output_level(LED_0_PIN, false);
	delay_ms(500);
	*/
	_AIS3624DQ.begin();
	uint8_t writeBuffer[1] = { AIS3624DQ_ACC_WHOM_AM_I}; // read WHO AM I dummy (just to test i2c comm with Accel..)
	_AIS3624DQ.readRegister8(writeBuffer, &dummy);
	printf("\r\nGot: %x\r\n", dummy);
		
    /* Replace with your application code */
	printf("setup done!\r\n");
    while (1) 
    {	
				
		if(_AIS3624DQ.XYZdataAvailable()){	// Make sure fresh data is available		
			
			_AIS3624DQ.readRawAccelX(&Xaxis);
			_AIS3624DQ.readRawAccelY(&Yaxis);
			_AIS3624DQ.readRawAccelZ(&Zaxis);
		    printf("x: %d      y: %d      z: %d\r\n", Xaxis, Yaxis, Zaxis);
			/*
			//Read G values
			float XaxisF, YaxisF, ZaxisF;
			XaxisF = _AIS3624DQ.readFloatAccelX();
			YaxisF = _AIS3624DQ.readFloatAccelY();
			ZaxisF = _AIS3624DQ.readFloatAccelZ();
			printf("Xaxis: %.2f Yaxis: %.2f Zaxis: %.2f\r\n", Xaxis, Yaxis, Zaxis);
			//delay_ms(32); // readable speed
			*/
			
		}			
    }
}

/*
 * AIS3624DQ.cpp
 *
 * Created: 20-12-2017 13:28:10
 *  Author: Nishant Sood
 */ 

#include "AIS3624DQ.h"
#include <stdint.h>

extern "C" {
	#include <asf.h>
	#include <system_interrupt.h>
};

#define SDA 


/* Init software module. */
//! [dev_inst]
struct i2c_master_module i2c_master_instance;

/* Initialize config structure and software module. */
//! [init_conf]
struct i2c_master_config config_i2c_master;

/* Number of times to try to send packet if failed. */
//! [timeout]
#define TIMEOUT 0

//! [initialize_i2c]
void configure_i2c_master(void)
{
	
	i2c_master_get_config_defaults(&config_i2c_master);

	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 10000;
	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master);
	//! [enable_module]
	i2c_master_enable(&i2c_master_instance);
	
}

AIS3624DQcore :: AIS3624DQcore(uint8_t i2cADD){ //Core constructor
	
	I2CAddress =  i2cADD;
}

status_t AIS3624DQcore::readRegister8(uint8_t reg[], uint8_t* data_){ // reading 8bit registers
	
	//! [timeout_counter]
	uint16_t timeout = 0;
	struct i2c_master_packet packet = {
		.address     = AIS3624DQ_add,
		.data_length = 1,
		.data        = reg,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	
	/* Write buffer to slave until success. */
	//! [write_packet]
	while (i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == TIMEOUT) {
			break;
		}
	}

	/* Read from slave until success. */
	//! [read_packet]
	static uint8_t readBuffer[1];
	packet.data = readBuffer;
		
	/* Write buffer to slave until success. */
	//! [write_packet]
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == TIMEOUT) {
			break;
		}
	}
	
	*data_ = readBuffer[0];
	//printf("read: %x\r\n", readBuffer[0]);
	
}

status_t AIS3624DQcore :: writeRegister(uint8_t add, uint8_t val){ //write 8bit registers
	
	
}

status_t AIS3624DQ :: begin(){
	
   if (beginCore() == IMU_SUCCESS)
   {
	   static uint8_t writeBuffer[2] = { AIS3624DQ_ACC_CTRL_REG1, 0x27};
	   struct i2c_master_packet packet = {
		   .address     = AIS3624DQ_add,
		   .data_length = 2,
		   .data        = writeBuffer,
		   .ten_bit_address = false,
		   .high_speed      = false,
		   .hs_master_code  = 0x0,
	   };
	   
	   /* Write buffer to slave until success. */
	   //! [write_packet]
	   i2c_master_write_packet_wait(&i2c_master_instance, &packet);
	   
	   static uint8_t writeBuffer2[2]= { AIS3624DQ_ACC_CTRL_REG4, 0x30 };
	   packet = {
		   .address     = AIS3624DQ_add,
		   .data_length = 2,
		   .data        = writeBuffer2,
		   .ten_bit_address = false,
		   .high_speed      = false,
		   .hs_master_code  = 0x0,
	   };
	   
	   /* Write buffer to slave until success. */
	   //! [write_packet]
	   i2c_master_write_packet_wait(&i2c_master_instance, &packet);
	   	   
   }
}

// Check the corresponding bit for new data available
bool AIS3624DQ :: XYZdataAvailable(){
	
	uint8_t dummy;
	uint8_t writeBuffer[1] = { AIS3624DQ_ACC_STAT_REG };
	readRegister8(writeBuffer, &dummy);	
	if (dummy & 0x04 != 0)
	{
		return true;
	} 
	else
	{
		return false;
	}
}

// read raw 16-bit values for x,y,z axis of a accel in the next 3 functions
void AIS3624DQ::readRawAccelX( int16_t* data_){
	uint8_t myBuffer[2];
	uint8_t writeBuffer[1] = { AIS3624DQ_ACC_OUTX_L_XL };
	status_t returnError   = readRegister8(writeBuffer,  &myBuffer[0]); //LB
    uint8_t	writeBuffer1[1]= { AIS3624DQ_ACC_OUTX_H_XL };
	returnError            = readRegister8(writeBuffer1, &myBuffer[1]); //HB
	int16_t output         = (((int16_t)((int16_t)myBuffer[0] |  (int16_t)myBuffer[1] << 8))); //(LB | HB)
	
	*data_ = output;

}

void AIS3624DQ::readRawAccelY( int16_t* data_){
	uint8_t myBuffer[2];
	uint8_t writeBuffer[1] = { AIS3624DQ_ACC_OUTY_L_XL };
	status_t returnError   = readRegister8(writeBuffer,  &myBuffer[0]); //LB
	uint8_t writeBuffer1[1]= { AIS3624DQ_ACC_OUTY_H_XL };
	returnError            = readRegister8(writeBuffer1, &myBuffer[1]); //HB
	int16_t output         = (((int16_t)((int16_t)myBuffer[0] |  (int16_t)myBuffer[1] << 8))); //(LB | HB)

	*data_ = output;

}

void AIS3624DQ::readRawAccelZ( int16_t* data_){
	uint8_t myBuffer[2];
	uint8_t writeBuffer[1] = { AIS3624DQ_ACC_OUTZ_L_XL };
	status_t returnError   = readRegister8(writeBuffer,  &myBuffer[0]); //LB 
	uint8_t writeBuffer1[1]= { AIS3624DQ_ACC_OUTZ_H_XL };
	returnError            = readRegister8(writeBuffer1, &myBuffer[1]); //HB
	int16_t output         = (((int16_t)((int16_t)myBuffer[0] |  (int16_t)myBuffer[1] << 8))); //(LB | HB) 
	
	*data_ = output;

}

// Linear acceleration sensitivity adjustment using Co-efficient listed in Datasheet
float AIS3624DQ::calcAccel( int16_t input){
	
	float output = (float)(((float)input * 3.91 * (Accel_setting.accelRange >> 1)) / 1000.0); // 3.91 (typical) is the Linear acceleration sensitivity (refer to datasheet)
	return output;
}

// convert the respective axis raw values to Gs in the next 3 functions
float AIS3624DQ::readFloatAccelX( void ){
	
	int16_t Xaxis;
	readRawAccelX(&Xaxis);
	float output = calcAccel(Xaxis);
	return output;
}

float AIS3624DQ::readFloatAccelY( void ){
	
	int16_t Yaxis;
	readRawAccelY(&Yaxis);
	float output = calcAccel(Yaxis);
	return output;
}

float AIS3624DQ::readFloatAccelZ( void ){
	
	int16_t Zaxis;
	readRawAccelZ(&Zaxis);
	float output = calcAccel(Zaxis);
	return output;
}

AIS3624DQ :: AIS3624DQ(uint8_t i2cADD) : AIS3624DQcore(i2cADD) {
	
	
}


// this begins the low level comm bus
status_t AIS3624DQcore::beginCore(){
	
  status_t returnError = IMU_SUCCESS;
  configure_i2c_master();
  
  return returnError;
}
	


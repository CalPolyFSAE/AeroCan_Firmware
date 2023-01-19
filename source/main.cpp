#include "pin_mux.h"
#include "clock_config.h"
#include "MKE18F16.h"

#include "can.h"
#include "spi.h"
#include "fsl_lpspi.h"

#include "Pressure_Comp.h"
#include "crc.h"


#include "float.h"

using namespace BSP;


//Global Variables
uint8_t flag;
int32_t data[20];
uint16_t can_Data[20];
uint8_t EEPROM_Data_Array[451];


#define ADDRESS 0x500

#define PERIOD 50 // ms between cycles

void sendcan(uint8_t offset, uint32_t value){
    can::CANlight::frame f;
    f.ext = 1;
    f.id = ADDRESS + offset;
    f.data[0] = value & 0xff;
    f.data[1] = (value>>8) & 0xff;
    f.data[2] = (value>>16) & 0xff;
    can::CANlight::StaticClass().tx(0, f);    
}


int main(void) {


		//Board bootup as well as SPI and CAN initialization
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    SysTick_Config(60000);
    // Construct SPI driver with default driver settings
    spi::spi_config conf;
    spi::SPI::ConstructStatic(&conf);
    spi::SPI& spi = spi::SPI::StaticClass();
		//Construct CAN driver 
    can::can_config c;
    can::CANlight::ConstructStatic(&c);
    can::CANlight& can = can::CANlight::StaticClass();
    can::CANlight::canx_config c0;
    can.init(0, &c0);


/*

-------------------------------------------------------------------------------------------------------------

*********** The following code will begin EEPROM SPI Communication and commands to read EEPROM values ***********

-------------------------------------------------------------------------------------------------------------

*/

    // Set up master configuration, including CS pin
    spi::SPI::masterConfig mconf;
    mconf.baudRate = 100000;
    mconf.cphase = kLPSPI_ClockPhaseFirstEdge;
    mconf.cpol = kLPSPI_ClockPolarityActiveHigh;




    // Initialize SPI module 1 as master
    //SPI module 0 controls sensors 1-10
    spi.initMaster(0, &mconf);

  uint8_t txE[8] = {0x03, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  uint8_t rxE[8];
  
  int counter_TR = 0;


  if(counter_TR >= 0 && counter_TR < 255){

    EEPROM_Data_Array[counter_TR] = rxE[2];
    txE[2]+= 0x01;

    counter_TR++;
  }
  else if (counter_TR >= 255 && counter_TR < 451){

   EEPROM_Data_Array[counter_TR] = rxE[2];
   txE[2]+= 0x01;

   counter_TR++;
  }


    //Configure PortB and pin 6, followed by Create tx/rx arrays:
    //Anticipated command is 4 MOSI command bytes, followed by
    //4 MISO data bytes. Total message length is 8
    //Sensor 1
    spi.xcvrs[0].csport = gpio::PortB;
    spi.xcvrs[0].cspin = 5;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);


   // Compensate_Pressure_Init(rxE);

    //Sensor 2
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 3;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 3
    spi.xcvrs[0].csport = gpio::PortD;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 4
    spi.xcvrs[0].csport = gpio::PortD;
    spi.xcvrs[0].cspin = 5;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 5
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 0;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 6
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 16;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 7
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 14;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 8  
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 9;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 9
    spi.xcvrs[0].csport = gpio::PortA;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 10
    spi.xcvrs[0].csport = gpio::PortE;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, txE, rxE, 8);

    while(spi.xcvrs[0].transmitting);


    // Initialize SPI module 1 as master
    //SPI module 1 controls sensors 11-20
    spi.initMaster(1, &mconf);

    //Sensor 11
    spi.xcvrs[1].csport = gpio::PortD;
    spi.xcvrs[1].cspin = 16;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 12 config
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 9;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 13
    spi.xcvrs[1].csport = gpio::PortB;
    spi.xcvrs[1].cspin = 6;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 14
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 10;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 15
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 1;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 16
    spi.xcvrs[1].csport = gpio::PortA;
    spi.xcvrs[1].cspin = 13;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 17
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 6;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 18
    spi.xcvrs[1].csport = gpio::PortC;
    spi.xcvrs[1].cspin = 7;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 19	
    spi.xcvrs[1].csport = gpio::PortA;
    spi.xcvrs[1].cspin = 1;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 20
    spi.xcvrs[1].csport = gpio::PortD;
    spi.xcvrs[1].cspin = 3;
    spi.mastertx(1, txE, rxE, 8);

    while(spi.xcvrs[1].transmitting);


						//spi.mastertx(0, txE, rxE, 8);

/*
            //transmit ones and log rx buffer into data array 
            //sensor 1
            spi.xcvrs[0].csport = gpio::PortB;
            spi.xcvrs[0].cspin = 5;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[0] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(0, data[0]);
*/


/*

-------------------------------------------------------------------------------------------------------------

*********** The following code will begin ADC SPI Communication  *********** 

-------------------------------------------------------------------------------------------------------------

*/

    // Set up master configuration, including CS pin
   // spi::SPI::masterConfig mconf;
    //mconf.baudRate = 100000;
    mconf.cphase = kLPSPI_ClockPhaseSecondEdge;
    mconf.cpol = kLPSPI_ClockPolarityActiveHigh;

    // Initialize SPI module 1 as master
    //SPI module 0 controls sensors 1-10
    spi.initMaster(0, &mconf);


		//WREG Commands
    uint8_t tx[8] = {0x06, 0x44, 0x44, 0x08, 0xff, 0xff, 0xff, 0xff};
    uint8_t rx[8];

    //Configure PortB and pin 6, followed by Create tx/rx arrays:
    //Anticipated command is 4 MOSI command bytes, followed by
    //4 MISO data bytes. Total message length is 8
    //Sensor 1
    spi.xcvrs[0].csport = gpio::PortB;
    spi.xcvrs[0].cspin = 5;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 2
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 3;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 3
    spi.xcvrs[0].csport = gpio::PortD;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 4
    spi.xcvrs[0].csport = gpio::PortD;
    spi.xcvrs[0].cspin = 5;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 5
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 0;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 6
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 16;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 7
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 14;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 8  
    spi.xcvrs[0].csport = gpio::PortC;
    spi.xcvrs[0].cspin = 9;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);

    //Sensor 9
    spi.xcvrs[0].csport = gpio::PortA;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);


    //Sensor 10
    spi.xcvrs[0].csport = gpio::PortE;
    spi.xcvrs[0].cspin = 7;
    spi.mastertx(0, tx, rx, 8);

    while(spi.xcvrs[0].transmitting);


    // Initialize SPI module 1 as master
    //SPI module 1 controls sensors 11-20
    spi.initMaster(1, &mconf);

    //Sensor 11
    spi.xcvrs[1].csport = gpio::PortD;
    spi.xcvrs[1].cspin = 16;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 12 config
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 9;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 13
    spi.xcvrs[1].csport = gpio::PortB;
    spi.xcvrs[1].cspin = 6;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 14
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 10;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 15
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 1;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 16
    spi.xcvrs[1].csport = gpio::PortA;
    spi.xcvrs[1].cspin = 13;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 17
    spi.xcvrs[1].csport = gpio::PortE;
    spi.xcvrs[1].cspin = 6;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 18
    spi.xcvrs[1].csport = gpio::PortC;
    spi.xcvrs[1].cspin = 7;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);


    //Sensor 19	
    spi.xcvrs[1].csport = gpio::PortA;
    spi.xcvrs[1].cspin = 1;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);

    //Sensor 20
    spi.xcvrs[1].csport = gpio::PortD;
    spi.xcvrs[1].cspin = 3;
    spi.mastertx(1, tx, rx, 8);

    while(spi.xcvrs[1].transmitting);


/*

-------------------------------------------------------------------------------------------------------------

*********** The following code will begin reading ADC values *********** 

-------------------------------------------------------------------------------------------------------------

*/

    flag = 1;

    while(1) {

        if(flag){


            //transmit ones and log rx buffer into data array 
            //sensor 1
            spi.xcvrs[0].csport = gpio::PortB;
            spi.xcvrs[0].cspin = 5;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[0] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(0, data[0]);

            //transmit ones and log rx buffer into data array 
            //sensor 2
            spi.xcvrs[0].csport = gpio::PortC;
            spi.xcvrs[0].cspin = 3;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[1] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(1, data[1]);


            //transmit ones and log rx buffer into data array 
            //sensor 3
            spi.xcvrs[0].csport = gpio::PortD;
            spi.xcvrs[0].cspin = 7;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[2] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(2, data[2]);

            //transmit ones and log rx buffer into data array 
            //sensor 4
            spi.xcvrs[0].csport = gpio::PortD;
            spi.xcvrs[0].cspin = 5;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[3] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(3, data[3]);

            //transmit ones and log rx buffer into data array 
            //sensor 5
            spi.xcvrs[0].csport = gpio::PortC;
            spi.xcvrs[0].cspin = 0;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[4] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(4, data[4]);

            //transmit ones and log rx buffer into data array 
            //sensor 6
            spi.xcvrs[0].csport = gpio::PortC;
            spi.xcvrs[0].cspin = 16;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[5] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(5, data[5]);

            //transmit ones and log rx buffer into data array 
            //sensor 7
            spi.xcvrs[0].csport = gpio::PortC;
            spi.xcvrs[0].cspin = 14;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[6] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(6, data[6]);
            //transmit ones and log rx buffer into data array 
            //sensor 8
            spi.xcvrs[0].csport = gpio::PortC;
            spi.xcvrs[0].cspin = 9;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[7] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(7, data[7]);

            //transmit ones and log rx buffer into data array 
            //sensor 9
            spi.xcvrs[0].csport = gpio::PortA;
            spi.xcvrs[0].cspin = 7;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[8] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(8, data[8]);
            //transmit ones and log rx buffer into data array 
            //sensor 10
            spi.xcvrs[0].csport = gpio::PortE;
            spi.xcvrs[0].cspin = 7;

            spi.mastertx(0, tx+5, rx, 3);
            while(spi.xcvrs[0].transmitting);
            data[9] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(9, data[9]);

            //transmit ones and log rx buffer into data array 
            //sensor 11
            spi.xcvrs[1].csport = gpio::PortD;
            spi.xcvrs[1].cspin = 16;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[10] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

						//can_Data[10] = (data[10] & 0xff00u);

            sendcan(10, data[10]);


            //transmit ones and log rx buffer into data array 
            //sensor 12
            spi.xcvrs[1].csport = gpio::PortE;
            spi.xcvrs[1].cspin = 9;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[11] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

					//	can_Data[11] = (data[11] >> 8);

            sendcan(11, data[11]);



            //transmit ones and log rx buffer into data array 
            //sensor 13
            spi.xcvrs[1].csport = gpio::PortB;
            spi.xcvrs[1].cspin = 6;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[12] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

						//can_Data[12] = (data[12] & 0xff00u);

            sendcan(12, data[12]);

            //transmit ones and log rx buffer into data array 
            //sensor 14
            spi.xcvrs[1].csport = gpio::PortE;
            spi.xcvrs[1].cspin = 10;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[13] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(13, data[13]);

            //transmit ones and log rx buffer into data array 
            //sensor 15
            spi.xcvrs[1].csport = gpio::PortE;
            spi.xcvrs[1].cspin = 1;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[14] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(14, data[14]);

            //transmit ones and log rx buffer into data array 
            //sensor 16
            spi.xcvrs[1].csport = gpio::PortA;
            spi.xcvrs[1].cspin = 13;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[15] = rx[2] + (rx[1] << 8) + (rx[0] <<16);


            sendcan(15, data[15]);

            //transmit ones and log rx buffer into data array 
            //sensor 17
            spi.xcvrs[1].csport = gpio::PortE;
            spi.xcvrs[1].cspin = 6;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[16] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(16, data[16]);


            //transmit ones and log rx buffer into data array 
            //sensor 18
            spi.xcvrs[1].csport = gpio::PortC;
            spi.xcvrs[1].cspin = 7;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[17] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(17, data[17]);

            //transmit ones and log rx buffer into data array 
            //sensor 19
            spi.xcvrs[1].csport = gpio::PortA;
            spi.xcvrs[1].cspin = 1;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[18] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(18, data[18]);

            //transmit ones and log rx buffer into data array 
            //sensor 20
            spi.xcvrs[1].csport = gpio::PortD;
            spi.xcvrs[1].cspin = 3;

            spi.mastertx(1, tx+5, rx, 3);
            while(spi.xcvrs[1].transmitting);
            data[19] = rx[2] + (rx[1] << 8) + (rx[0] <<16);

            sendcan(19, data[19]);

            flag = 0;
        }
    }
    return 0;
}

extern "C" {
    void SysTick_Handler(void){
        static uint16_t timer = 0;
        timer++;
        if(timer == PERIOD){
            flag = 1;
            timer = 0;
        }
    }
}

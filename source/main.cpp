#include "pin_mux.h"
#include "clock_config.h"
#include "MKE18F16.h"

#include "can.h"
#include "spi.h"
#include "fsl_lpspi.h"


extern "C" {
  #include "Pressure_Comp.h"
  #include "crc.h"
}


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

    Compensate_Pressure_Init(rxE);

    // Initialize SPI module 1 as master
    //SPI module 1 controls sensors 11-20
    //spi.initMaster(1, &mconf);

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
    //spi.initMaster(0, &mconf);


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

    // Initialize SPI module 1 as master
    //SPI module 1 controls sensors 11-20
    //spi.initMaster(1, &mconf);

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

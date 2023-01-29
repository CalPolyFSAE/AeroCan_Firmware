#include "pin_mux.h"
#include "clock_config.h"
#include "MKE18F16.h"

#include "can.h"
#include "spi.h"
#include "fsl_lpspi.h"
#include "gpio.h"

#include "Pressure_Comp.h"
#include "crc.h"


#include "float.h"

using namespace BSP;


//Global Variables
uint8_t flag;
uint32_t _presdata[20];
uint32_t _tempdata[20];
uint16_t can_Data[20];
uint8_t EEPROM_Data_Array[451];

typedef struct Sensor_Tag
{
  gpio::GPIO_port port;
  int pin;
} Sensor_Struct;


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

void sendtx(spi::SPI* spi, gpio::GPIO_port port, int pin, uint8_t* tx, uint8_t* rx, int xcvr, size_t size) {

    spi->xcvrs[xcvr].csport = port;
    spi->xcvrs[xcvr].cspin = pin;
    spi->mastertx(xcvr, tx, rx, size);

    while(spi->xcvrs[xcvr].transmitting);
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

    Sensor_Struct sensor1 = {gpio::PortB, 5};
    Sensor_Struct sensor2 = {gpio::PortC, 3};
    Sensor_Struct sensor3 = {gpio::PortD, 7};
    Sensor_Struct sensor4 = {gpio::PortD, 5};
    Sensor_Struct sensor5 = {gpio::PortC, 0};
    Sensor_Struct sensor6 = {gpio::PortC, 16};
    Sensor_Struct sensor7 = {gpio::PortC, 14};
    Sensor_Struct sensor8 = {gpio::PortC, 9};
    Sensor_Struct sensor9 = {gpio::PortA, 7};
    Sensor_Struct sensor10 = {gpio::PortE, 7};
    Sensor_Struct sensor11 = {gpio::PortD, 16};
    Sensor_Struct sensor12 = {gpio::PortE, 9};
    Sensor_Struct sensor13 = {gpio::PortB, 6};
    Sensor_Struct sensor14 = {gpio::PortE, 10};
    Sensor_Struct sensor15 = {gpio::PortE, 1};
    Sensor_Struct sensor16 = {gpio::PortA, 13};
    Sensor_Struct sensor17 = {gpio::PortE, 6};
    Sensor_Struct sensor18 = {gpio::PortC, 7};
    Sensor_Struct sensor19 = {gpio::PortA, 1};
    Sensor_Struct sensor20 = {gpio::PortD, 3};


    //Configure PortB and pin 6, followed by Create tx/rx arrays:
    //Anticipated command is 4 MOSI command bytes, followed by
    //4 MISO data bytes. Total message length is 8
    sendtx(&spi, sensor1.port, sensor1.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor2.port, sensor2.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor3.port, sensor3.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor4.port, sensor4.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor5.port, sensor5.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor6.port, sensor6.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor7.port, sensor7.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor8.port, sensor8.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor9.port, sensor9.pin, txE, rxE, 0, 8);
    sendtx(&spi, sensor10.port, sensor10.pin, txE, rxE, 0, 8);
    spi.initMaster(1, &mconf);
    sendtx(&spi, sensor11.port, sensor11.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor12.port, sensor12.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor13.port, sensor13.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor14.port, sensor14.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor15.port, sensor15.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor16.port, sensor16.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor17.port, sensor17.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor18.port, sensor18.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor19.port, sensor19.pin, txE, rxE, 1, 8);
    sendtx(&spi, sensor20.port, sensor20.pin, txE, rxE, 1, 8);

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
    uint8_t tx_t[8] = {0x06, 0x44, 0x46, 0x08, 0xff, 0xff, 0xff, 0xff};
    uint8_t rx[8];

/*

-------------------------------------------------------------------------------------------------------------

*********** The following code will begin reading ADC values *********** 

-------------------------------------------------------------------------------------------------------------

*/

    flag = 1;

    while(1) {

        if(flag){
            //Configure PortB and pin 6, followed by Create tx/rx arrays:
            //Anticipated command is 4 MOSI command bytes, followed by
            //4 MISO data bytes. Total message length is 8
            //Sensor 1
            
            sendtx(&spi, sensor1.port, sensor1.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor2.port, sensor2.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor3.port, sensor3.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor4.port, sensor4.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor5.port, sensor5.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor6.port, sensor6.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor7.port, sensor7.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor8.port, sensor8.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor9.port, sensor9.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor10.port, sensor10.pin, tx, rx, 0, 8);
            sendtx(&spi, sensor11.port, sensor11.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor12.port, sensor12.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor13.port, sensor13.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor14.port, sensor14.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor15.port, sensor15.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor16.port, sensor16.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor17.port, sensor17.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor18.port, sensor18.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor19.port, sensor19.pin, tx, rx, 1, 8);
            sendtx(&spi, sensor20.port, sensor20.pin, tx, rx, 1, 8);


            sendtx(&spi, sensor1.port, sensor1.pin, tx+5, rx, 0, 3);
            _presdata[0] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor2.port, sensor2.pin, tx+5, rx, 0, 3);
            _presdata[1] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor3.port, sensor3.pin, tx+5, rx, 0, 3);
            _presdata[2] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor4.port, sensor4.pin, tx+5, rx, 0, 3);
            _presdata[3] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor5.port, sensor5.pin, tx+5, rx, 0, 3);
            _presdata[4] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor6.port, sensor6.pin, tx+5, rx, 0, 3);
            _presdata[5] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor7.port, sensor7.pin, tx+5, rx, 0, 3);
            _presdata[6] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor8.port, sensor8.pin, tx+5, rx, 0, 3);
            _presdata[7] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor9.port, sensor9.pin, tx+5, rx, 0, 3);
            _presdata[8] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor10.port, sensor10.pin, tx+5, rx, 0, 3);
            _presdata[9] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor11.port, sensor11.pin, tx+5, rx, 1, 3);
            _presdata[10] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor12.port, sensor12.pin, tx+5, rx, 1, 3);
            _presdata[11] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor13.port, sensor13.pin, tx+5, rx, 1, 3);
            _presdata[12] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor14.port, sensor14.pin, tx+5, rx, 1, 3);
            _presdata[13] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor15.port, sensor15.pin, tx+5, rx, 1, 3);
            _presdata[14] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor16.port, sensor16.pin, tx+5, rx, 1, 3);
            _presdata[15] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor17.port, sensor17.pin, tx+5, rx, 1, 3);
            _presdata[16] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor18.port, sensor18.pin, tx+5, rx, 1, 3);
            _presdata[17] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor19.port, sensor19.pin, tx+5, rx, 1, 3);
            _presdata[18] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor20.port, sensor20.pin, tx+5, rx, 1, 3);
            _presdata[19] = rx[2] + (rx[1] << 8) + (rx[0] <<16);


            for(int i=0; i<20; i++) {
              sendcan(i, _presdata[i]);
            }

            sendtx(&spi, sensor1.port, sensor1.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor2.port, sensor2.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor3.port, sensor3.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor4.port, sensor4.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor5.port, sensor5.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor6.port, sensor6.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor7.port, sensor7.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor8.port, sensor8.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor9.port, sensor9.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor10.port, sensor10.pin, tx_t, rx, 0, 8);
            sendtx(&spi, sensor11.port, sensor11.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor12.port, sensor12.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor13.port, sensor13.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor14.port, sensor14.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor15.port, sensor15.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor16.port, sensor16.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor17.port, sensor17.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor18.port, sensor18.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor19.port, sensor19.pin, tx_t, rx, 1, 8);
            sendtx(&spi, sensor20.port, sensor20.pin, tx_t, rx, 1, 8);

            sendtx(&spi, sensor1.port, sensor1.pin, tx_t+5, rx, 0, 3);
            _tempdata[0] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor2.port, sensor2.pin, tx_t+5, rx, 0, 3);
            _tempdata[1] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor3.port, sensor3.pin, tx_t+5, rx, 0, 3);
            _tempdata[2] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor4.port, sensor4.pin, tx_t+5, rx, 0, 3);
            _tempdata[3] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor5.port, sensor5.pin, tx_t+5, rx, 0, 3);
            _tempdata[4] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor6.port, sensor6.pin, tx_t+5, rx, 0, 3);
            _tempdata[5] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor7.port, sensor7.pin, tx_t+5, rx, 0, 3);
            _tempdata[6] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor8.port, sensor8.pin, tx_t+5, rx, 0, 3);
            _tempdata[7] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor9.port, sensor9.pin, tx_t+5, rx, 0, 3);
            _tempdata[8] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor10.port, sensor10.pin, tx_t+5, rx, 0, 3);
            _tempdata[9] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor11.port, sensor11.pin, tx_t+5, rx, 1, 3);
            _tempdata[10] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor12.port, sensor12.pin, tx_t+5, rx, 1, 3);
            _tempdata[11] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor13.port, sensor13.pin, tx_t+5, rx, 1, 3);
            _tempdata[12] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor14.port, sensor14.pin, tx_t+5, rx, 1, 3);
            _tempdata[13] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor15.port, sensor15.pin, tx_t+5, rx, 1, 3);
            _tempdata[14] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor16.port, sensor16.pin, tx_t+5, rx, 1, 3);
            _tempdata[15] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor17.port, sensor17.pin, tx_t+5, rx, 1, 3);
            _tempdata[16] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor18.port, sensor18.pin, tx_t+5, rx, 1, 3);
            _tempdata[17] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor19.port, sensor19.pin, tx_t+5, rx, 1, 3);
            _tempdata[18] = rx[2] + (rx[1] << 8) + (rx[0] <<16);
            sendtx(&spi, sensor20.port, sensor20.pin, tx_t+5, rx, 1, 3);
            _tempdata[19] = rx[2] + (rx[1] << 8) + (rx[0] <<16);


            for(int i=0; i<20; i++) {
              sendcan(i, _tempdata[i]);
            }

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

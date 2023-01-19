/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: MKE18F512xxx16
package_id: MKE18F512VLH16
mcu_data: ksdk2_0
processor_version: 5.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '47', peripheral: GPIOA, signal: 'GPIO, 3', pin_signal: ADC1_SE1/PTA3/FTM3_CH1/LPI2C0_SCL/EWM_IN/LPUART0_TX}
  - {pin_num: '48', peripheral: GPIOA, signal: 'GPIO, 2', pin_signal: ADC1_SE0/PTA2/FTM3_CH0/LPI2C0_SDA/EWM_OUT_b/LPUART0_RX}
  - {pin_num: '32', peripheral: LPSPI0, signal: SCK, pin_signal: ADC0_SE6/PTB2/FTM1_CH0/LPSPI0_SCK/FTM1_QD_PHB/TRGMUX_IN3}
  - {pin_num: '31', peripheral: LPSPI0, signal: SIN, pin_signal: ADC0_SE7/PTB3/FTM1_CH1/LPSPI0_SIN/FTM1_QD_PHA/TRGMUX_IN2}
  - {pin_num: '50', peripheral: ADC0, signal: 'SE, 0', pin_signal: ADC0_SE0/ACMP0_IN0/PTA0/FTM2_CH1/LPI2C0_SCLS/FXIO_D2/FTM2_QD_PHA/LPUART0_CTS/TRGMUX_OUT3}
  - {pin_num: '16', peripheral: GPIOE, signal: 'GPIO, 9', pin_signal: ACMP2_IN2/DAC0_OUT/PTE9/FTM0_CH7/LPUART2_CTS}
  - {pin_num: '2', peripheral: LPSPI1, signal: SCK, pin_signal: ADC2_SE0/PTD0/FTM0_CH2/LPSPI1_SCK/FTM2_CH0/FXIO_D0/TRGMUX_OUT1}
  - {pin_num: '1', peripheral: LPSPI1, signal: SIN, pin_signal: ADC2_SE1/PTD1/FTM0_CH3/LPSPI1_SIN/FTM2_CH1/FXIO_D1/TRGMUX_OUT2}
  - {pin_num: '46', peripheral: LPSPI1, signal: SOUT, pin_signal: ADC1_SE2/PTD2/FTM3_CH4/LPSPI1_SOUT/FXIO_D4/TRGMUX_IN5}
  - {pin_num: '15', peripheral: GPIOD, signal: 'GPIO, 15', pin_signal: ACMP2_IN1/PTD15/FTM0_CH0}
  - {pin_num: '14', peripheral: GPIOD, signal: 'GPIO, 16', pin_signal: ACMP2_IN0/PTD16/FTM0_CH1}
  - {pin_num: '49', peripheral: GPIOA, signal: 'GPIO, 1', pin_signal: ADC0_SE1/ACMP0_IN1/PTA1/FTM1_CH1/LPI2C0_SDAS/FXIO_D3/FTM1_QD_PHA/LPUART0_RTS/TRGMUX_OUT0}
  - {pin_num: '37', peripheral: GPIOA, signal: 'GPIO, 7', pin_signal: ADC0_SE3/ACMP1_IN1/PTA7/FTM0_FLT2/RTC_CLKIN/LPUART1_RTS}
  - {pin_num: '35', peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: ADC2_SE15/PTC9/LPUART1_TX/FTM1_FLT1/LPUART0_RTS}
  - {pin_num: '30', peripheral: GPIOC, signal: 'GPIO, 14', pin_signal: ADC0_SE12/ACMP2_IN5/PTC14/FTM1_CH2}
  - {pin_num: '28', peripheral: GPIOC, signal: 'GPIO, 16', pin_signal: ADC0_SE14/PTC16/FTM1_FLT2/LPI2C1_SDAS}
  - {pin_num: '26', peripheral: GPIOC, signal: 'GPIO, 0', pin_signal: ADC0_SE8/ACMP1_IN4/PTC0/FTM0_CH0/FTM1_CH6}
  - {pin_num: '51', peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: ADC1_SE5/PTC7/LPUART1_TX/CAN1_TX/FTM3_CH3}
  - {pin_num: '53', peripheral: GPIOE, signal: 'GPIO, 6', pin_signal: ADC1_SE11/ACMP0_IN6/PTE6/LPSPI0_PCS2/FTM3_CH7/LPUART1_RTS}
  - {pin_num: '12', peripheral: GPIOB, signal: 'GPIO, 6', pin_signal: XTAL/PTB6/LPI2C0_SDA}
  - {pin_num: '4', peripheral: GPIOE, signal: 'GPIO, 10', pin_signal: ADC2_SE12/PTE10/CLKOUT/FTM2_CH4/FXIO_D4/TRGMUX_OUT4}
  - {pin_num: '59', peripheral: GPIOE, signal: 'GPIO, 1', pin_signal: ADC2_SE6/PTE1/LPSPI0_SIN/LPI2C0_HREQ/LPI2C1_SCL/FTM1_FLT1}
  - {pin_num: '55', peripheral: GPIOA, signal: 'GPIO, 13', pin_signal: ADC2_SE4/PTA13/FTM1_CH7/CAN1_TX/LPI2C1_SCLS}
  - {pin_num: '45', peripheral: GPIOD, signal: 'GPIO, 3', pin_signal: ADC1_SE3/PTD3/FTM3_CH5/LPSPI1_PCS0/FXIO_D5/TRGMUX_IN4/NMI_b}
  - {pin_num: '39', peripheral: GPIOE, signal: 'GPIO, 7', pin_signal: ADC2_SE2/ACMP2_IN6/PTE7/FTM0_CH7/FTM3_FLT0}
  - {pin_num: '24', peripheral: GPIOD, signal: 'GPIO, 5', pin_signal: PTD5/FTM2_CH3/LPTMR0_ALT2/FTM2_FLT1/PWT_IN2/TRGMUX_IN7}
  - {pin_num: '22', peripheral: GPIOD, signal: 'GPIO, 7', pin_signal: PTD7/LPUART2_TX/FTM2_FLT3}
  - {pin_num: '20', peripheral: GPIOC, signal: 'GPIO, 3', pin_signal: ADC0_SE11/ACMP0_IN4/EXTAL32/PTC3/FTM0_CH3/CAN0_TX}
  - {pin_num: '18', peripheral: GPIOB, signal: 'GPIO, 5', pin_signal: PTB5/FTM0_CH5/LPSPI0_PCS1/TRGMUX_IN0/ACMP1_OUT}
  - {pin_num: '3', peripheral: GPIOE, signal: 'GPIO, 11', pin_signal: ADC2_SE13/PTE11/PWT_IN1/LPTMR0_ALT1/FTM2_CH5/FXIO_D5/TRGMUX_OUT5}
  - {pin_num: '11', peripheral: GPIOB, signal: 'GPIO, 7', pin_signal: EXTAL/PTB7/LPI2C0_SCL}
  - {pin_num: '13', peripheral: GPIOE, signal: 'GPIO, 3', pin_signal: PTE3/FTM0_FLT0/LPUART2_RTS/FTM2_FLT0/TRGMUX_IN6/ACMP2_OUT}
  - {pin_num: '17', peripheral: GPIOE, signal: 'GPIO, 8', pin_signal: ACMP0_IN3/PTE8/FTM0_CH6}
  - {pin_num: '21', peripheral: GPIOC, signal: 'GPIO, 2', pin_signal: ADC0_SE10/ACMP0_IN5/XTAL32/PTC2/FTM0_CH2/CAN0_RX}
  - {pin_num: '23', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: PTD6/LPUART2_RX/FTM2_FLT2}
  - {pin_num: '25', peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: ADC0_SE9/ACMP1_IN3/PTC1/FTM0_CH1/FTM1_CH7}
  - {pin_num: '27', peripheral: GPIOC, signal: 'GPIO, 17', pin_signal: ADC0_SE15/PTC17/FTM1_FLT3/LPI2C1_SCLS}
  - {pin_num: '29', peripheral: GPIOC, signal: 'GPIO, 15', pin_signal: ADC0_SE13/ACMP2_IN4/PTC15/FTM1_CH3}
  - {pin_num: '34', peripheral: GPIOB, signal: 'GPIO, 0', pin_signal: ADC0_SE4/ADC1_SE14/PTB0/LPUART0_RX/LPSPI0_PCS0/LPTMR0_ALT3/PWT_IN3}
  - {pin_num: '36', peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: ADC2_SE14/PTC8/LPUART1_RX/FTM1_FLT0/LPUART0_CTS}
  - {pin_num: '38', peripheral: GPIOA, signal: 'GPIO, 6', pin_signal: ADC0_SE2/ACMP1_IN0/PTA6/FTM0_FLT1/LPSPI1_PCS1/LPUART1_CTS}
  - {pin_num: '42', peripheral: GPIOB, signal: 'GPIO, 13', pin_signal: ADC1_SE8/ADC2_SE8/PTB13/FTM0_CH1/FTM3_FLT1}
  - {pin_num: '43', peripheral: GPIOB, signal: 'GPIO, 12', pin_signal: ADC1_SE7/PTB12/FTM0_CH0/FTM3_FLT2}
  - {pin_num: '44', peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: ADC1_SE6/ACMP1_IN6/PTD4/FTM0_FLT3/FTM3_FLT3}
  - {pin_num: '52', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: ADC1_SE4/PTC6/LPUART1_RX/CAN1_RX/FTM3_CH2}
  - {pin_num: '56', peripheral: GPIOA, signal: 'GPIO, 12', pin_signal: ADC2_SE5/PTA12/FTM1_CH6/CAN1_RX/LPI2C1_SDAS}
  - {pin_num: '54', peripheral: GPIOE, signal: 'GPIO, 2', pin_signal: ADC1_SE10/PTE2/LPSPI0_SOUT/LPTMR0_ALT3/FTM3_CH6/PWT_IN3/LPUART1_CTS}
  - {pin_num: '60', peripheral: GPIOE, signal: 'GPIO, 0', pin_signal: ADC2_SE7/PTE0/LPSPI0_SCK/TCLK1/LPI2C1_SDA/FTM1_FLT2}
  - {pin_num: '57', peripheral: GPIOA, signal: 'GPIO, 11', pin_signal: PTA11/FTM1_CH5/LPUART0_RX/FXIO_D1}
  - {pin_num: '33', peripheral: GPIOB, signal: 'GPIO, 1', pin_signal: ADC0_SE5/ADC1_SE15/PTB1/LPUART0_TX/LPSPI0_SOUT/TCLK0}
  - {pin_num: '19', peripheral: LPSPI0, signal: SOUT, pin_signal: ACMP1_IN2/PTB4/FTM0_CH4/LPSPI0_SOUT/TRGMUX_IN1}
  - {pin_num: '5', peripheral: CAN0, signal: TX, pin_signal: PTE5/TCLK2/FTM2_QD_PHA/FTM2_CH3/CAN0_TX/FXIO_D7/EWM_IN}
  - {pin_num: '6', peripheral: CAN0, signal: RX, pin_signal: PTE4/BUSOUT/FTM2_QD_PHB/FTM2_CH2/CAN0_RX/FXIO_D6/EWM_OUT_b}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Clock Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Clock Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Clock Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* Clock Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* Clock Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    /* PORTA0 (pin 50) is configured as ADC0_SE0 */
    PORT_SetPinMux(PORTA, 0U, kPORT_PinDisabledOrAnalog);

    /* PORTA1 (pin 49) is configured as PTA1 */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAsGpio);

    /* PORTA11 (pin 57) is configured as PTA11 */
    PORT_SetPinMux(PORTA, 11U, kPORT_MuxAsGpio);

    /* PORTA12 (pin 56) is configured as PTA12 */
    PORT_SetPinMux(PORTA, 12U, kPORT_MuxAsGpio);

    /* PORTA13 (pin 55) is configured as PTA13 */
    PORT_SetPinMux(PORTA, 13U, kPORT_MuxAsGpio);

    /* PORTA2 (pin 48) is configured as PTA2 */
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAsGpio);

    /* PORTA3 (pin 47) is configured as PTA3 */
    PORT_SetPinMux(PORTA, 3U, kPORT_MuxAsGpio);

    /* PORTA6 (pin 38) is configured as PTA6 */
    PORT_SetPinMux(PORTA, 6U, kPORT_MuxAsGpio);

    /* PORTA7 (pin 37) is configured as PTA7 */
    PORT_SetPinMux(PORTA, 7U, kPORT_MuxAsGpio);

    /* PORTB0 (pin 34) is configured as PTB0 */
    PORT_SetPinMux(PORTB, 0U, kPORT_MuxAsGpio);

    /* PORTB1 (pin 33) is configured as PTB1 */
    PORT_SetPinMux(PORTB, 1U, kPORT_MuxAsGpio);

    /* PORTB12 (pin 43) is configured as PTB12 */
    PORT_SetPinMux(PORTB, 12U, kPORT_MuxAsGpio);

    /* PORTB13 (pin 42) is configured as PTB13 */
    PORT_SetPinMux(PORTB, 13U, kPORT_MuxAsGpio);

    /* PORTB2 (pin 32) is configured as LPSPI0_SCK */
    PORT_SetPinMux(PORTB, 2U, kPORT_MuxAlt3);

    /* PORTB3 (pin 31) is configured as LPSPI0_SIN */
    PORT_SetPinMux(PORTB, 3U, kPORT_MuxAlt3);

    /* PORTB4 (pin 19) is configured as LPSPI0_SOUT */
    PORT_SetPinMux(PORTB, 4U, kPORT_MuxAlt3);

    /* PORTB5 (pin 18) is configured as PTB5 */
    PORT_SetPinMux(PORTB, 5U, kPORT_MuxAsGpio);

    /* PORTB6 (pin 12) is configured as PTB6 */
    PORT_SetPinMux(PORTB, 6U, kPORT_MuxAsGpio);

    /* PORTB7 (pin 11) is configured as PTB7 */
    PORT_SetPinMux(PORTB, 7U, kPORT_MuxAsGpio);

    /* PORTC0 (pin 26) is configured as PTC0 */
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);

    /* PORTC1 (pin 25) is configured as PTC1 */
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);

    /* PORTC14 (pin 30) is configured as PTC14 */
    PORT_SetPinMux(PORTC, 14U, kPORT_MuxAsGpio);

    /* PORTC15 (pin 29) is configured as PTC15 */
    PORT_SetPinMux(PORTC, 15U, kPORT_MuxAsGpio);

    /* PORTC16 (pin 28) is configured as PTC16 */
    PORT_SetPinMux(PORTC, 16U, kPORT_MuxAsGpio);

    /* PORTC17 (pin 27) is configured as PTC17 */
    PORT_SetPinMux(PORTC, 17U, kPORT_MuxAsGpio);

    /* PORTC2 (pin 21) is configured as PTC2 */
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAsGpio);

    /* PORTC3 (pin 20) is configured as PTC3 */
    PORT_SetPinMux(PORTC, 3U, kPORT_MuxAsGpio);

    /* PORTC6 (pin 52) is configured as PTC6 */
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAsGpio);

    /* PORTC7 (pin 51) is configured as PTC7 */
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio);

    /* PORTC8 (pin 36) is configured as PTC8 */
    PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio);

    /* PORTC9 (pin 35) is configured as PTC9 */
    PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);

    /* PORTD0 (pin 2) is configured as LPSPI1_SCK */
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt3);

    /* PORTD1 (pin 1) is configured as LPSPI1_SIN */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt3);

    /* PORTD15 (pin 15) is configured as PTD15 */
    PORT_SetPinMux(PORTD, 15U, kPORT_MuxAsGpio);

    /* PORTD16 (pin 14) is configured as PTD16 */
    PORT_SetPinMux(PORTD, 16U, kPORT_MuxAsGpio);

    /* PORTD2 (pin 46) is configured as LPSPI1_SOUT */
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt3);

    /* PORTD3 (pin 45) is configured as PTD3 */
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAsGpio);

    /* PORTD4 (pin 44) is configured as PTD4 */
    PORT_SetPinMux(PORTD, 4U, kPORT_MuxAsGpio);

    /* PORTD5 (pin 24) is configured as PTD5 */
    PORT_SetPinMux(PORTD, 5U, kPORT_MuxAsGpio);

    /* PORTD6 (pin 23) is configured as PTD6 */
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAsGpio);

    /* PORTD7 (pin 22) is configured as PTD7 */
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);

    /* PORTE0 (pin 60) is configured as PTE0 */
    PORT_SetPinMux(PORTE, 0U, kPORT_MuxAsGpio);

    /* PORTE1 (pin 59) is configured as PTE1 */
    PORT_SetPinMux(PORTE, 1U, kPORT_MuxAsGpio);

    /* PORTE10 (pin 4) is configured as PTE10 */
    PORT_SetPinMux(PORTE, 10U, kPORT_MuxAsGpio);

    /* PORTE11 (pin 3) is configured as PTE11 */
    PORT_SetPinMux(PORTE, 11U, kPORT_MuxAsGpio);

    /* PORTE2 (pin 54) is configured as PTE2 */
    PORT_SetPinMux(PORTE, 2U, kPORT_MuxAsGpio);

    /* PORTE3 (pin 13) is configured as PTE3 */
    PORT_SetPinMux(PORTE, 3U, kPORT_MuxAsGpio);

    /* PORTE4 (pin 6) is configured as CAN0_RX */
    PORT_SetPinMux(PORTE, 4U, kPORT_MuxAlt5);

    /* PORTE5 (pin 5) is configured as CAN0_TX */
    PORT_SetPinMux(PORTE, 5U, kPORT_MuxAlt5);

    /* PORTE6 (pin 53) is configured as PTE6 */
    PORT_SetPinMux(PORTE, 6U, kPORT_MuxAsGpio);

    /* PORTE7 (pin 39) is configured as PTE7 */
    PORT_SetPinMux(PORTE, 7U, kPORT_MuxAsGpio);

    /* PORTE8 (pin 17) is configured as PTE8 */
    PORT_SetPinMux(PORTE, 8U, kPORT_MuxAsGpio);

    /* PORTE9 (pin 16) is configured as PTE9 */
    PORT_SetPinMux(PORTE, 9U, kPORT_MuxAsGpio);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

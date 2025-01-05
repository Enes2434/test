//#############################################################################
//
// FILE:     rec_hal.c
//
// TITLE:    solution hardware abstraction layer
//           This file consists of board related initialization
//           this file is used to make the
//           main file more readable
//
//#############################################################################

#include "rec_hal.h"

//
// Device setup
//

//
// This routine sets up the basic device ocnfiguration such as initializing PLL
// copying code from FLASH to RAM
//

//
// device_setup()
//
void REC_HAL_setupDevice(void){
    //
    // Initialize device clock and peripherals
    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    Device_init();
//    Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
}



void Rec_init(){
  EALLOW;
  myPinMux_init();
  UART0_DIR_init();
  myUART_init();
  I2C_EEPROM_init();
  myINTERRUPT_init();
  EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void myPinMux_init(){

    // GPIO64 -> UART0_DIR Pinmux
    GPIO_setPinConfig(UPS_REC_UART_DIR_PIN_CONFIG);

    //
    // SCI -> mySCI Pinmux
    //
    GPIO_setPinConfig(UPS_REC_SCIRX_PIN_CONFIG);
    GPIO_setPadConfig(UPS_REC_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(UPS_REC_SCIRX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(UPS_REC_SCITX_PIN_CONFIG);
    GPIO_setPadConfig(UPS_REC_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(UPS_REC_SCITX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(UPS_REC_UART0_ISO_SCIRX_PIN_CONFIG);
    GPIO_setPadConfig(UPS_REC_UART0_ISO_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(UPS_REC_UART0_ISO_SCIRX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(UPS_REC_UART0_ISOSCITX_PIN_CONFIG);
    GPIO_setPadConfig(UPS_REC_UART0_ISO_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(UPS_REC_UART0_ISO_SCITX_GPIO, GPIO_QUAL_ASYNC);

    // I2CA -> I2C_EEPROM Pinmux
    //
    GPIO_setPinConfig(I2C_EEPROM_I2CSDA_PIN_CONFIG);
    GPIO_setPadConfig(I2C_EEPROM_I2CSDA_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);//GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(I2C_EEPROM_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(I2C_EEPROM_I2CSCL_PIN_CONFIG);
    GPIO_setPadConfig(I2C_EEPROM_I2CSCL_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);//GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(I2C_EEPROM_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

}

void UART0_DIR_init(){
    GPIO_writePin(UART0_DIR, 0);
    GPIO_setPadConfig(UART0_DIR, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(UART0_DIR, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(UART0_DIR, GPIO_DIR_MODE_OUT);
    GPIO_setControllerCore(UART0_DIR, GPIO_CORE_CPU1);

    GPIO_writePin(UPS_REC_UART0_ISO_DIR, 1);
    GPIO_setPadConfig(UPS_REC_UART0_ISO_DIR, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(UPS_REC_UART0_ISO_DIR, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(UPS_REC_UART0_ISO_DIR, GPIO_DIR_MODE_OUT);
    GPIO_setControllerCore(UPS_REC_UART0_ISO_DIR, GPIO_CORE_CPU1);

}

void myUART_init(){

    SCI_clearInterruptStatus(UPS_REC_SCI_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    SCI_clearOverflowStatus(UPS_REC_SCI_BASE);
    SCI_disableFIFO(UPS_REC_SCI_BASE);
    SCI_resetChannels(UPS_REC_SCI_BASE);
    SCI_setConfig(UPS_REC_SCI_BASE, DEVICE_LSPCLK_FREQ, UPS_REC_SCI_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(UPS_REC_SCI_BASE);
    SCI_performSoftwareReset(UPS_REC_SCI_BASE);
    SCI_enableInterrupt(UPS_REC_SCI_BASE,SCI_INT_RXRDY_BRKDT);
    SCI_enableModule(UPS_REC_SCI_BASE);

    SCI_clearInterruptStatus(UPS_REC_UART0_ISO_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    SCI_clearOverflowStatus(UPS_REC_UART0_ISO_BASE);
    SCI_disableFIFO(UPS_REC_UART0_ISO_BASE);
    SCI_resetChannels(UPS_REC_UART0_ISO_BASE);
    SCI_setConfig(UPS_REC_UART0_ISO_BASE, DEVICE_LSPCLK_FREQ, UPS_REC_UART0_ISO_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(UPS_REC_UART0_ISO_BASE);
    SCI_performSoftwareReset(UPS_REC_UART0_ISO_BASE);
    SCI_enableModule(UPS_REC_UART0_ISO_BASE);

}

//void I2C_EEPROM_init(){
//    I2C_disableModule(I2C_EEPROM_BASE);
//    I2C_initController(I2C_EEPROM_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
//    I2C_setConfig(I2C_EEPROM_BASE, I2C_CONTROLLER_SEND_MODE);
//    I2C_setTargetAddress(I2C_EEPROM_BASE, 80);
//    I2C_disableLoopback(I2C_EEPROM_BASE);
//    I2C_setBitCount(I2C_EEPROM_BASE, I2C_BITCOUNT_8);
//    I2C_setDataCount(I2C_EEPROM_BASE, 3);
//    I2C_setAddressMode(I2C_EEPROM_BASE, I2C_ADDR_MODE_7BITS);
//    I2C_enableFIFO(I2C_EEPROM_BASE);
//    I2C_setEmulationMode(I2C_EEPROM_BASE, I2C_EMULATION_FREE_RUN);
//    I2C_enableModule(I2C_EEPROM_BASE);
//}
void I2C_EEPROM_init(){
    I2C_disableModule(I2C_EEPROM_BASE);
    I2C_initMaster(I2C_EEPROM_BASE, DEVICE_SYSCLK_FREQ, 100000, I2C_DUTYCYCLE_50);
    I2C_setConfig(I2C_EEPROM_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_setTargetAddress(I2C_EEPROM_BASE, 80);
    I2C_setOwnSlaveAddress(I2CA_BASE, 96); //I2CA address A0
    I2C_disableLoopback(I2C_EEPROM_BASE);
    I2C_setBitCount(I2C_EEPROM_BASE, I2C_BITCOUNT_8);
    I2C_setDataCount(I2C_EEPROM_BASE, 2);
    I2C_setAddressMode(I2C_EEPROM_BASE, I2C_ADDR_MODE_7BITS);
    I2C_enableFIFO(I2C_EEPROM_BASE);
    I2C_clearInterruptStatus(I2C_EEPROM_BASE, I2C_INT_ARB_LOST | I2C_INT_NO_ACK);
    I2C_setFIFOInterruptLevel(I2C_EEPROM_BASE, I2C_FIFO_TXEMPTY, I2C_FIFO_RX2); //!< Receive FIFO 2/16 full //Transmit FIFO empty
    I2C_enableInterrupt(I2C_EEPROM_BASE, I2C_INT_ADDR_SLAVE | I2C_INT_ARB_LOST | I2C_INT_NO_ACK | I2C_INT_STOP_CONDITION);
    I2C_setEmulationMode(I2C_EEPROM_BASE, I2C_EMULATION_FREE_RUN);
    I2C_enableModule(I2C_EEPROM_BASE);
}

//void myUART_init(){
//    SCI_clearInterruptStatus(UPS_REC_SCI_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
//    SCI_clearOverflowStatus(UPS_REC_SCI_BASE);
//    SCI_resetTxFIFO(UPS_REC_SCI_BASE);
//    SCI_resetRxFIFO(UPS_REC_SCI_BASE);
//    SCI_resetChannels(UPS_REC_SCI_BASE);
//    SCI_setConfig(UPS_REC_SCI_BASE, DEVICE_LSPCLK_FREQ, UPS_REC_SCI_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
//    SCI_enableLoopback(UPS_REC_SCI_BASE);
//    SCI_performSoftwareReset(UPS_REC_SCI_BASE);
//    SCI_setFIFOInterruptLevel(UPS_REC_SCI_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
//    SCI_enableFIFO(UPS_REC_SCI_BASE);
//    SCI_enableModule(UPS_REC_SCI_BASE);
//}

//void myUART_init(){
//    SCI_clearInterruptStatus(UPS_REC_SCI_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
//    SCI_clearOverflowStatus(UPS_REC_SCI_BASE);
//    SCI_resetTxFIFO(UPS_REC_SCI_BASE);
//    SCI_resetRxFIFO(UPS_REC_SCI_BASE);
//    SCI_resetChannels(UPS_REC_SCI_BASE);
//    SCI_setConfig(UPS_REC_SCI_BASE, DEVICE_LSPCLK_FREQ, UPS_REC_SCI_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
//    SCI_disableLoopback(UPS_REC_SCI_BASE);
//    SCI_performSoftwareReset(UPS_REC_SCI_BASE);
//    SCI_enableInterrupt(UPS_REC_SCI_BASE, SCI_INT_RXFF);
//    SCI_setFIFOInterruptLevel(UPS_REC_SCI_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
//    SCI_enableFIFO(UPS_REC_SCI_BASE);
//    SCI_enableModule(UPS_REC_SCI_BASE);
//}

void myINTERRUPT_init(){

//  Interrupt Setings for UPS_REC_INT_mySCI_RX
    Interrupt_register(UPS_REC_INT_mySCI_RX, &UPS_REC_COM_ISR);
    Interrupt_enable(UPS_REC_INT_mySCI_RX);

//    // Interrupt Setings for INT_mySCIB_TX
//    Interrupt_register(INT_mySCIB_TX, &INT_mySCIB_TX_ISR);
//    Interrupt_disable(INT_mySCIB_TX);
}


//
// clearPWM Interrupt Flag
//
void REC_HAL_clearPWMInterruptFlag(uint32_t base){
    EPWM_clearEventTriggerInterruptFlag(base);
}

//
// getPWM Interrupt Flag
//
bool REC_HAL_getPWMInterruptFlag(uint32_t base){
    return(EPWM_getEventTriggerInterruptStatus(base));
}

// SPI veri gönderme ve alma fonksiyonları
void SendSPI(uint16_t data)
{
    SPI_transmit16Bits(SPIA_BASE, data);
//    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) {} // Veri gönderiminin tamamlanmasını bekle
}

uint16_t ReadSPI(void)
{
//    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) {} // Veri alımının tamamlanmasını bekle
    return SPI_readDataNonBlocking(SPIA_BASE); // Alınan veriyi oku
}

////*****************************************************************************
////
//// Modbus read array
////
////*****************************************************************************
//uint16_t Modbus_read(uint32_t base, uint16_t * const array, uint16_t length,uint16_t com_start_byte){
//    ASSERT(SCI_isBaseValid(base));
//    uint16_t i;
//    volatile uint16_t recive_rx_mesag;
//
//   recive_rx_mesag = (uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
//   if(recive_rx_mesag == com_start_byte){
//       recive_rx_mesag=0;
//       for(i = 1U; i < length; i++){
//           // Wait until a character is available in the receive buffer.
//           uart_timeout_counter=0;
//           while(!SCI_isDataAvailableNonFIFO(base) ){
//               // Zaman aşımını kontrol et
//               if (uart_timeout_counter  > 10000) {
//                   return ; // bir hata döndürebilirsiniz
//               }
//           }
//           // Return the character from the receive buffer.
//           array[i] = (uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
//       }
//   }
//}



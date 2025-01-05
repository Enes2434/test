/*
 * I2C_EEPROM.c
 *
 *  Created on: 23 A?u 2024
 *      Author: PWR04
 */
#include "I2C_EEPROM.h"


//---------------------------------------------------------------------------/
// Function Name : writeDataI2C                                             /
// Description   : Delay between write and read...not less 4ms ,                                                     /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/

void writeDataI2C(uint16_t slaveAddr, uint16_t Addr, uint16_t data){

//    while(I2C_getStopConditionStatus(I2C_EEPROM_BASE));
    I2C_disableFIFO(I2C_EEPROM_BASE);
    I2C_enableFIFO(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(60);

    I2C_setDataCount(I2C_EEPROM_BASE, 3); //number of address + number of data

    I2C_setSlaveAddress(I2C_EEPROM_BASE, slaveAddr);

    I2C_putData(I2C_EEPROM_BASE, (uint8_t)((Addr & 0xFF00) >> 8)); // Send the EEPROM's internal address to write to : MSB of the address first ,
    I2C_putData(I2C_EEPROM_BASE, (uint8_t)(Addr & 0x00FF));  // Send the EEPROM's internal address to write to : LSB of the address ,
    I2C_putData(I2C_EEPROM_BASE, data);

    I2C_setConfig(I2C_EEPROM_BASE, I2C_MASTER_SEND_MODE);
    I2C_sendStartCondition(I2C_EEPROM_BASE);
    I2C_sendStopCondition(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(125); //250us gerekli yoksa verilerin hepsi gonderilemiyor
}


void EEPROM_Write_Data(uint16_t slaveAddr, uint16_t Addr, uint16_t num_of_data, uint16_t* msgBuffer, uint8_t is16Bit) {
    uint16_t i;
    I2C_disableFIFO(I2C_EEPROM_BASE);
    I2C_enableFIFO(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(60);

    I2C_setSlaveAddress(I2C_EEPROM_BASE, slaveAddr);

    // Verilen veri sayısını ve adresi ayarlayın
    if (is16Bit) {
        I2C_setDataCount(I2C_EEPROM_BASE, ((num_of_data * 2) + 2)); // Her 16 bit veri iki 8 bit veriye bölünecek
    } else {
        I2C_setDataCount(I2C_EEPROM_BASE, (num_of_data + 2));       // Doğrudan 8 bit veri gönderilecek
    }

    I2C_putData(I2C_EEPROM_BASE, (uint8_t)((Addr & 0xFF00) >> 8)); // MSB of the address
    I2C_putData(I2C_EEPROM_BASE, (uint8_t)(Addr & 0x00FF));        // LSB of the address

    if (is16Bit) {
        for (i = 0; i < num_of_data; i++) {
            // 16 bit veriyi iki 8 bit veriye böl
            uint16_t data = msgBuffer[i];
            I2C_putData(I2C_EEPROM_BASE, (uint8_t)((data & 0xFF00) >> 8)); // MSB of the data
            I2C_putData(I2C_EEPROM_BASE, (uint8_t)(data & 0x00FF));        // LSB of the data
        }
    } else {
        for (i = 0; i < num_of_data; i++) {
            I2C_putData(I2C_EEPROM_BASE, (uint8_t)msgBuffer[i]);
        }
    }

    I2C_setConfig(I2C_EEPROM_BASE, I2C_MASTER_SEND_MODE);
    I2C_sendStartCondition(I2C_EEPROM_BASE);
    I2C_sendStopCondition(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(250);
}

//---------------------------------------------------------------------------/
// Function Name : readDataI2C                                             /
// Description   : None                                                      /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
void readDataI2C(uint16_t slaveAddr, uint16_t Addr){

    uint16_t attemptCount;
//    while(I2C_getStatus(I2C_EEPROM_BASE) & I2C_STS_BUS_BUSY);
    I2C_disableFIFO(I2C_EEPROM_BASE);
    I2C_enableFIFO(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(60);

    I2C_setDataCount(I2C_EEPROM_BASE, 2); //number of address(es) (highAddr + Low Addr)
    I2C_setSlaveAddress(I2C_EEPROM_BASE, slaveAddr);

    I2C_putData(I2C_EEPROM_BASE, (uint8_t)((Addr & 0xFF00) >> 8)); // Send the EEPROM's internal address to write to : MSB of the address first ,
    I2C_putData(I2C_EEPROM_BASE, (uint8_t)(Addr & 0x00FF));  // Send the EEPROM's internal address to write to : LSB of the address ,

    I2C_setConfig(I2C_EEPROM_BASE, I2C_MASTER_SEND_MODE);
    I2C_sendStartCondition(I2C_EEPROM_BASE);
    I2C_sendStopCondition(I2C_EEPROM_BASE);
    DEVICE_DELAY_US(250);

    I2C_setSlaveAddress(I2C_EEPROM_BASE, slaveAddr);
    I2C_setConfig(I2C_EEPROM_BASE, I2C_MASTER_RECEIVE_MODE | I2C_REPEAT_MODE);
    I2C_sendStartCondition(I2C_EEPROM_BASE);
    I2C_sendStopCondition(I2C_EEPROM_BASE);

    attemptCount = 1;
    while(I2C_getStopConditionStatus(I2C_EEPROM_BASE) && attemptCount <= 25U);
    {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }
}



//---------------------------------------------------------------------------/
// Function Name : I2C_W_Word                                             /
// Description   : None                                                      /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
void I2C_W_Word(uint16_t slaveAddr,uint16_t Addr, uint16_t Data){

  uint8_t Buffer;
  Buffer = (uint8_t) ((Data & 0xFF00) >> 8); // high byte
  writeDataI2C(slaveAddr, Addr , Buffer);
  DEVICE_DELAY_US(4000);//4ms
  Buffer = (uint8_t) Data;
  writeDataI2C(slaveAddr, (Addr + 1), Buffer);
  DEVICE_DELAY_US(4000);//4ms
}


//---------------------------------------------------------------------------/
// Function Name : I2C_R_Word                                                   /
// Description   : None                                                      /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t I2C_R_Word(uint16_t slaveAddr,uint16_t Addr){
  uint8_t   Buffer;
  uint16_t  Res;
  Res = 0;
  readDataI2C(slaveAddr,Addr);
  Buffer=I2C_getData(I2C_EEPROM_BASE);
  DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
  Res = (Res | Buffer) << 8; // high byte

  readDataI2C(slaveAddr,(Addr + 1));
  Buffer=I2C_getData(I2C_EEPROM_BASE);
  DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
  Res = Res | Buffer;

  return  Res;
}

///**********************************************************************************/
///**********************************************************************************/
///**********************************************************************************/



uint16_t I2CBusScan(uint32_t base, uint16_t *pAvailableI2C_slaves){
    uint16_t probeSlaveAddress, i, status;
    //Disable interrupts on Stop condition, NACK and arbitration lost condition
    I2C_disableInterrupt(base, (I2C_INT_ADDR_SLAVE|I2C_INT_STOP_CONDITION | I2C_INT_ARB_LOST | I2C_INT_NO_ACK));
    i = 0;
    for(probeSlaveAddress=1;probeSlaveAddress<=MAX_10_BIT_ADDRESS;probeSlaveAddress++){
        //Check I2C bus status
        status = checkBusStatus(base);
        if(status){
           ESTOP0;
           return status;
        }

        I2C_setConfig(base, (I2C_MASTER_SEND_MODE | I2C_REPEAT_MODE));

        //Enable 10-bit addressing if probeSlaveAddress is greater than 127U
        if(probeSlaveAddress > MAX_7_BIT_ADDRESS){
            //10-bit addressing
            I2C_setAddressMode(base, I2C_ADDR_MODE_10BITS);
        }

        // Setup slave address
        I2C_setSlaveAddress(base, probeSlaveAddress);

        I2C_sendStartCondition(base);

        //Wait for the slave address to be transmitted
        while(!(I2C_getStatus(base) & I2C_STS_REG_ACCESS_RDY));

        //Generate STOP condition
        I2C_sendStopCondition(base);

        //Wait for the I2CMDR.STP to be cleared
        while(I2C_getStopConditionStatus(base));

        //Wait for the Bus busy bit to be cleared
        while(I2C_isBusBusy(base));

        uint16_t I2CStatus = I2C_getStatus(base);

        //If Slave address is acknowledged, store slave address
        //in pAvailableI2C_slaves
        if(!(I2CStatus & I2C_STS_NO_ACK))
        {
            pAvailableI2C_slaves[i++] = probeSlaveAddress;
        }
        //Clear NACK bit in I2CSTR
        I2C_clearStatus(base,I2C_STS_NO_ACK|I2C_STS_ARB_LOST|I2C_STS_REG_ACCESS_RDY|I2C_STS_STOP_CONDITION);
    }

    I2C_setConfig(base, (I2C_MASTER_SEND_MODE));
    I2C_setAddressMode(base, I2C_ADDR_MODE_7BITS); //7-bit addressing
    I2C_enableInterrupt(base, (I2C_INT_ADDR_SLAVE|I2C_INT_STOP_CONDITION | I2C_INT_ARB_LOST | I2C_INT_NO_ACK));
    return SUCCESS;
}

//---------------------------------------------------------------------------/
// Function Name : I2C_R_Word                                                   /
// Description   : I2C bus'ı üzerinde belirli bir slave adresi ve kontrol baytlarını güvenilir bir şekilde iletmeyi amaçlar                                                     /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t I2C_TransmitSlaveAddress_ControlBytes(uint16_t slaveAddr,uint16_t Addr,uint32_t base){
    uint16_t status, attemptCount=1;
    status = 1;
    //Bu döngü, I2C bus'ının durumunu kontrol eder ve belirli bir süre bekler. Eğer status doğruysa (bus boş değilse) ve deneme sayısı NumOfAttempts değerine ulaşmadıysa,
    //işlemi tekrarlar. DEVICE_DELAY_US fonksiyonu, belirtilen mikrosaniye kadar bekler
    while(status & (attemptCount <= 5)){
        status = checkBusStatus(base);
        attemptCount++;
        DEVICE_DELAY_US(10);
    }

    //Eğer status hala doğruysa, yani bus'ın durumu hala uygun değilse
    //bu durumda işlev durur ve mevcut durumu döndürür
    if(status){
        return status;
    }

  //Eğer bus uygun durumdaysa, I2C modülü master gönderme modunda ve tekrar modunda yapılandırılır.
    I2C_setConfig(base, (I2C_MASTER_SEND_MODE|I2C_REPEAT_MODE));

    // Setup slave address
    I2C_setSlaveAddress(base, slaveAddr);
    //Kontrol baytları, bayt bayt olarak gönderilir. Baytlar, büyük endian (en yüksek anlamlı bit önce) formatında gönderilir.

    I2C_putData(I2C_EEPROM_BASE, (uint8_t)((Addr & 0xFF00) >> 8)); // MSB of the address
    I2C_putData(I2C_EEPROM_BASE, (uint8_t)(Addr & 0x00FF));        // LSB of the address

    //START koşulu gönderilir ve ardından 150 mikro saniye beklenir.
    I2C_sendStartCondition(base);
    DEVICE_DELAY_US(150U);
    //NACK (No Acknowledgement) durumunu kontrol etmek için handleNACK fonksiyonu çağrılır.
    //Eğer NACK durumu varsa ve deneme sayısı belirli bir limitin altındaysa, yeniden başlatılır ve işlemi tekrarlar.
    status = handleNACK(base);
    if(status){
      if(attemptCount <= 5){
          attemptCount++;
          I2C_setConfig(base, (I2C_MASTER_SEND_MODE));
          I2C_sendStartCondition(base);
          DEVICE_DELAY_US(10);
      }
      else{
          return status;
      }
    }
    attemptCount = 1;
   // Veri FIFO durumunu kontrol eder ve FIFO'nun boşalmasını bekler. FIFO, veri gönderimi sırasında kullanılan bir tampon bellek alanıdır
   //FIFO boşalana kadar beklenir veya deneme sayısı belirli bir limitin altındaysa, işlemi tekrarlar. Her tekrar arasında belirli bir süre beklenir.
    while(I2C_getTxFIFOStatus(base) && attemptCount <= 36U){
       status = handleNACK(base);
       if(status){
          return status;
       }
       attemptCount++;
       DEVICE_DELAY_US(10);
    }
    return SUCCESS; //Eğer tüm işlemler başarılı bir şekilde tamamlanmışsa, SUCCESS döndürülür.
}

//---------------------------------------------------------------------------/
// Function Name : I2C_R_Word                                                   /
// Description   : Bu kod, I2C (Inter-Integrated Circuit) protokolü kullanarak bir master cihazdan slave cihaza veri gönderimini gerçekleştiren bir işlevdir.
//               : İşlev, FIFO (First In, First Out) tamponları kullanarak veri aktarımını optimize eder ve çeşitli durumları yönetir
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t I2C_MasterTransmitter(uint16_t slaveAddr,uint16_t Addr,uint16_t num_of_data,uint16_t* msgBuffer,uint32_t base){
    uint16_t status, attemptCount;

    //FIFO'lar devre dışı bırakılır ve ardından tekrar etkinleştirilir. Bu adım, FIFO'nun temizlenmesi ve başlangıçta uygun durumda olması için yapılır.
    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    //Slave adresi ve kontrol baytlarını gönderir. Bu işlev başarıyla tamamlanmışsa, işlem devam eder. Aksi halde, bir hata durumu döndürülür ve işlev sonlanır.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr,Addr,base);

    if(status){
        return status;
    }

    //FIFO'ya gönderilecek toplam veri sayısı ayarlanır. Bu, adres baytları ve veri baytlarının toplamını içerir.
    I2C_setDataCount(base, (2U + num_of_data));

    // I2C_FIFO_TXEMPTY: FIFO'nun boş olduğunda kesinti oluşturulur.
    // I2C_FIFO_RXFULL: FIFO'nun dolu olduğunda kesinti oluşturulur.
    // I2C_INT_TXFF: FIFO'nun dolması ile ilgili kesinti etkinleştirilir.
    I2C_setFIFOInterruptLevel(base, I2C_FIFO_TXEMPTY, I2C_FIFO_RXFULL);
    I2C_enableInterrupt(base, I2C_INT_TXFF);

     //numofSixteenByte: FIFO seviyesindeki tam veri bloklarının sayısı.
    //  remainingBytes: FIFO seviyesinin tam olarak doldurulamadığı kalan veri baytları.
    uint16_t numofSixteenByte  = num_of_data / I2C_FIFO_LEVEL;
    uint16_t remainingBytes    = num_of_data % I2C_FIFO_LEVEL;

    uint16_t i,count = 0,buff_pos=0;

    //FIFO'yu tam olarak doldurarak veriyi gönderir.
    //FIFO'nun boşalmasını bekler ve NACK (No Acknowledgement) durumunu kontrol eder.
    //Eğer NACK durumunda bir hata bulunursa, hata durumu döndürülür.
    while(count < numofSixteenByte){
        for(i=1;i<=I2C_FIFO_LEVEL;i++){
            I2C_putData(base, (uint8_t)msgBuffer[buff_pos++]);
        }

        attemptCount = 1;
        while(I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (I2C_FIFO_LEVEL + 2U)){
            status = handleNACK(base);
            if(status){
              return status;
            }
            attemptCount++;
            DEVICE_DELAY_US(10);
        }
        count++;
    }

    //FIFO'nun kalan baytları ile doldurulur ve gönderilir. FIFO'nun boşalmasını bekler ve NACK durumunu kontrol eder
    for (i=0; i < remainingBytes; i++){
        I2C_putData(base, (uint8_t)msgBuffer[buff_pos++]);
    }

    attemptCount = 1;
    while(I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (remainingBytes + 2U)){
        status = handleNACK(base);
        if(status){
          return status;
        }
        attemptCount++;
        DEVICE_DELAY_US(10);
    }
    //Verilerin gönderimi tamamlandığında STOP koşulu gönderilir ve STOP koşulunun tamamlanmasını bekler. Bu, veri transferinin sona erdiğini belirtir.
    I2C_sendStopCondition(base);
    attemptCount = 1;
    while(I2C_getStopConditionStatus(base) && attemptCount <= 3U){
        DEVICE_DELAY_US(10);
        attemptCount++;
    }

    return SUCCESS;
}


uint16_t I2C_MasterTransmitter_Word(uint16_t slaveAddr, uint16_t Addr, uint16_t num_of_data, uint16_t* msgBuffer, uint32_t base) {
    uint16_t status, attemptCount;

    // FIFO'lar devre dışı bırakılır ve ardından tekrar etkinleştirilir.
    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // Slave adresi ve kontrol baytlarını gönderir.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr, Addr, base);
    if(status) {
        return status;
    }

    // FIFO'ya gönderilecek toplam veri sayısı ayarlanır.
    // Her veri 16-bit olduğundan, toplam veri sayısını 2 ile çarparız.
    I2C_setDataCount(base, (2U + 2U * num_of_data));

    // FIFO kesinti seviyelerini ve kesintileri ayarlama.
    I2C_setFIFOInterruptLevel(base, I2C_FIFO_TXEMPTY, I2C_FIFO_RXFULL);
    I2C_enableInterrupt(base, I2C_INT_TXFF);

    // Tam blok verilerin sayısı ve kalan baytlar hesaplanır.
    uint16_t numofSixteenByte = (2U * num_of_data) / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = (2U * num_of_data) % I2C_FIFO_LEVEL;

    uint16_t i, count = 0, buff_pos = 0;

    // FIFO'yu tam olarak doldurarak veriyi gönderir.
    while(count < numofSixteenByte) {
        for(i = 1; i <= I2C_FIFO_LEVEL; i+=2) {
            // 16-bit veriyi 8-bit parçalarına ayırarak FIFO'ya yazma.
            uint16_t data = msgBuffer[buff_pos++];
            I2C_putData(base, (uint8_t)((data >> 8) & 0xFF)); // MSB
            I2C_putData(base, (uint8_t)(data & 0xFF));     // LSB
        }

        attemptCount = 1;
        while(I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (I2C_FIFO_LEVEL + 2U)) {
            status = handleNACK(base);
            if(status) {
                return status;
            }
            attemptCount++;
            DEVICE_DELAY_US(10);
        }
        count++;
    }

    // FIFO'nun kalan baytları ile doldurulur ve gönderilir.
    for (i = 0; i < remainingBytes; i+=2) {
        uint16_t data = msgBuffer[buff_pos++];
        I2C_putData(base, (uint8_t)((data >> 8) & 0xFF)); // MSB
        I2C_putData(base, (uint8_t)(data & 0xFF));     // LSB
    }

    attemptCount = 1;
    while(I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (remainingBytes + 2U)) {
        status = handleNACK(base);
        if(status) {
            return status;
        }
        attemptCount++;
        DEVICE_DELAY_US(10);
    }

    // Verilerin gönderimi tamamlandığında STOP koşulu gönderilir.
    I2C_sendStopCondition(base);
    attemptCount = 1;
    while(I2C_getStopConditionStatus(base) && attemptCount <= 3U) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }

    return SUCCESS;
}


uint16_t I2C_MasterTransmitter_DWord(uint16_t slaveAddr, uint16_t Addr, uint16_t num_of_data, uint32_t* msgBuffer, uint32_t base) {
    uint16_t status, attemptCount;

    // FIFO'lar devre dışı bırakılır ve ardından tekrar etkinleştirilir.
    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // Slave adresi ve kontrol baytlarını gönderir.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr, Addr, base);
    if (status) {
        return status;
    }

    // FIFO'ya gönderilecek toplam veri sayısı ayarlanır.
    // Her veri 32-bit olduğundan, toplam veri sayısını 4 ile çarparız.
    I2C_setDataCount(base, (2U + 4U * num_of_data));

    // FIFO kesinti seviyelerini ve kesintileri ayarlama.
    I2C_setFIFOInterruptLevel(base, I2C_FIFO_TXEMPTY, I2C_FIFO_RXFULL);
    I2C_enableInterrupt(base, I2C_INT_TXFF);

    // Tam blok verilerin sayısı ve kalan baytlar hesaplanır.
    uint16_t numofSixteenByte = (4U * num_of_data) / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = (4U * num_of_data) % I2C_FIFO_LEVEL;

    uint16_t i, count = 0, buff_pos = 0;

    // FIFO'yu tam olarak doldurarak veriyi gönderir.
    while (count < numofSixteenByte) {
        for (i = 1; i <= I2C_FIFO_LEVEL; i += 4) {
            // 32-bit veriyi 8-bit parçalarına ayırarak FIFO'ya yazma.
            uint32_t data = msgBuffer[buff_pos++];
            I2C_putData(base, (uint8_t)((data >> 24) & 0xFF)); // 1. byte (MSB)
            I2C_putData(base, (uint8_t)((data >> 16) & 0xFF)); // 2. byte
            I2C_putData(base, (uint8_t)((data >> 8) & 0xFF));  // 3. byte
            I2C_putData(base, (uint8_t)(data & 0xFF));         // 4. byte (LSB)
        }

        attemptCount = 1;
        while (I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (I2C_FIFO_LEVEL + 2U)) {
            status = handleNACK(base);
            if (status) {
                return status;
            }
            attemptCount++;
            DEVICE_DELAY_US(10);
        }
        count++;
    }

    // FIFO'nun kalan baytları ile doldurulur ve gönderilir.
    for (i = 0; i < remainingBytes; i += 4) {
        uint32_t data = msgBuffer[buff_pos++];
        I2C_putData(base, (uint8_t)((data >> 24) & 0xFF)); // 1. byte (MSB)
        I2C_putData(base, (uint8_t)((data >> 16) & 0xFF)); // 2. byte
        I2C_putData(base, (uint8_t)((data >> 8) & 0xFF));  // 3. byte
        I2C_putData(base, (uint8_t)(data & 0xFF));         // 4. byte (LSB)
    }

    attemptCount = 1;
    while (I2C_getTxFIFOStatus(base) && attemptCount <= 9 * (remainingBytes + 2U)) {
        status = handleNACK(base);
        if (status) {
            return status;
        }
        attemptCount++;
        DEVICE_DELAY_US(10);
    }

    // Verilerin gönderimi tamamlandığında STOP koşulu gönderilir.
    I2C_sendStopCondition(base);
    attemptCount = 1;
    while (I2C_getStopConditionStatus(base) && attemptCount <= 3U) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }

    return SUCCESS;
}


//---------------------------------------------------------------------------/
// Function Name : I2C_MasterReceiver                                                /
// Description   : Bu kod, bir master cihazın I2C bus'ı üzerinden bir slave cihazdan veri almasını gerçekleştiren bir işlevi tanımlar.
// İşlev, FIFO (First In, First Out) tamponlarını kullanarak veri alımını optimize eder ve çeşitli
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t I2C_MasterReceiver(uint16_t slaveAddr,uint16_t Addr,uint16_t num_of_data,uint16_t* msgBuffer,uint32_t base){
    uint16_t status;
    uint16_t attemptCount;


    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    //Slave adresi ve kontrol baytlarını gönderir. Bu işlev başarıyla tamamlanmışsa, işlem devam eder. Aksi halde, bir hata durumu döndürülür ve işlev sonlanır.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr,Addr,base);
    if(status){
        return status;
    }

    uint16_t numofSixteenByte  = num_of_data / I2C_FIFO_LEVEL;
    uint16_t remainingBytes    = num_of_data % I2C_FIFO_LEVEL;

    //I2C'yi master alım modunda ve tekrar modunda yapılandırır. START koşulu gönderilir.
    I2C_setConfig(base, (I2C_MASTER_RECEIVE_MODE|I2C_REPEAT_MODE));
    I2C_sendStartCondition(base);

    uint16_t i,count = 0,buff_pos=0;
    while(count < numofSixteenByte){
        status = handleNACK(base);
        if(status){
          return status;
        }
        count++;
        attemptCount = 1;
        while(!(I2C_getRxFIFOStatus(base) == I2C_FIFO_RXFULL) && attemptCount <= 162U){
            DEVICE_DELAY_US(10);
            attemptCount++;
        }

        for(i=0; i<I2C_FIFO_LEVEL; i++){
            msgBuffer[buff_pos++] = I2C_getData(base);
        }
    }

    //FIFO'nun kalan baytları ile doldurulmasını bekler. STOP koşulu gönderilir ve kalan baytlar alınarak msgBuffer'a aktarılır.
    attemptCount = 1;
    while(!(I2C_getRxFIFOStatus(base) == remainingBytes) && attemptCount <= 9 * (remainingBytes + 2U)){
       DEVICE_DELAY_US(10);
       attemptCount++;
    }
    I2C_sendStopCondition(base);
    for(i=0; i<remainingBytes; i++){
        msgBuffer[buff_pos++] = I2C_getData(base);
    }
    status = handleNACK(base);
    if(status) {
      return status;
    }
    I2C_disableFIFO(base);
    attemptCount = 1;
    while(I2C_getStopConditionStatus(base) && attemptCount <= 3U);{
        DEVICE_DELAY_US(10);
        attemptCount++;
    }
    return SUCCESS;
}

//******************//
//*****************//
//****************//
uint16_t I2C_MasterReceiver_Word(uint16_t slaveAddr, uint16_t Addr, uint16_t num_of_data, uint16_t* msgBuffer, uint32_t base) {
    uint16_t status, attemptCount;

    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // Slave adresi ve kontrol baytlarını gönderir.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr, Addr, base);
    if(status) {
        return status;
    }
    // I2C'yi master alım modunda ve tekrar modunda yapılandırır. START koşulu gönderilir.
    I2C_setConfig(base, (I2C_MASTER_RECEIVE_MODE | I2C_REPEAT_MODE));
    I2C_sendStartCondition(base);

    uint16_t numofSixteenByte = (2U * num_of_data) / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = (2U * num_of_data) % I2C_FIFO_LEVEL;

    uint16_t i, count = 0, buff_pos = 0;
    // FIFO'dan verileri okur ve `msgBuffer`'a 16-bit olarak yerleştirir.
    while(count < numofSixteenByte) {
        status = handleNACK(base);
        if(status) {
            return status;
        }
        count++;
        attemptCount = 1;
        while (!(I2C_getRxFIFOStatus(base) == I2C_FIFO_RXFULL) && attemptCount <= 162U) {
            DEVICE_DELAY_US(10);
            attemptCount++;
        }

        for(i = 0; i < I2C_FIFO_LEVEL; i+=2) {
            uint16_t msb = I2C_getData(base);
            uint16_t lsb = I2C_getData(base);
            msgBuffer[buff_pos++] = (msb << 8) | lsb;
        }
    }

    // FIFO'nun kalan baytlarını okur ve `msgBuffer`'a 16-bit olarak yerleştirir.
    attemptCount = 1;
    while (!(I2C_getRxFIFOStatus(base) == remainingBytes) && attemptCount <= 9 * (remainingBytes + 2U)) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }
    I2C_sendStopCondition(base);
    for(i = 0; i < remainingBytes; i += 2) {
        uint16_t msb = I2C_getData(base);
        uint16_t lsb = I2C_getData(base);
        msgBuffer[buff_pos++] = (msb << 8) | lsb;
    }

    status = handleNACK(base);
    if(status) {
        return status;
    }
    I2C_disableFIFO(base);
    attemptCount = 1;
    while(I2C_getStopConditionStatus(base) && attemptCount <= 3U) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }

    return SUCCESS;
}

uint16_t I2C_MasterReceiver_DWord(uint16_t slaveAddr, uint16_t Addr, uint16_t num_of_data, uint32_t* msgBuffer, uint32_t base) {
    uint16_t status, attemptCount;

    I2C_disableFIFO(base);
    I2C_enableFIFO(base);

    // Slave adresi ve kontrol baytlarını gönderir.
    status = I2C_TransmitSlaveAddress_ControlBytes(slaveAddr, Addr, base);
    if (status) {
        return status;
    }
    // I2C'yi master alım modunda ve tekrar modunda yapılandırır. START koşulu gönderilir.
    I2C_setConfig(base, (I2C_MASTER_RECEIVE_MODE | I2C_REPEAT_MODE));
    I2C_sendStartCondition(base);

    uint16_t numofSixteenByte = (4U * num_of_data) / I2C_FIFO_LEVEL;
    uint16_t remainingBytes = (4U * num_of_data) % I2C_FIFO_LEVEL;

    uint16_t i, count = 0, buff_pos = 0;
    // FIFO'dan verileri okur ve `msgBuffer`'a 32-bit olarak yerleştirir.
    while (count < numofSixteenByte) {
        status = handleNACK(base);
        if (status) {
            return status;
        }
        count++;
        attemptCount = 1;
        while (!(I2C_getRxFIFOStatus(base) == I2C_FIFO_RXFULL) && attemptCount <= 162U) {
            DEVICE_DELAY_US(10);
            attemptCount++;
        }

        for (i = 0; i < I2C_FIFO_LEVEL; i += 4) {
            uint8_t byte0 = I2C_getData(base);
            uint8_t byte1 = I2C_getData(base);
            uint8_t byte2 = I2C_getData(base);
            uint8_t byte3 = I2C_getData(base);
            msgBuffer[buff_pos++] = ((uint32_t)byte0 << 24) | ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | (uint32_t)byte3;
        }
    }

    // FIFO'nun kalan baytlarını okur ve `msgBuffer`'a 32-bit olarak yerleştirir.
    attemptCount = 1;
    while (!(I2C_getRxFIFOStatus(base) == remainingBytes) && attemptCount <= 9 * (remainingBytes + 2U)) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }
    I2C_sendStopCondition(base);
    for (i = 0; i < remainingBytes; i += 4) {
        uint8_t byte0 = I2C_getData(base);
        uint8_t byte1 = I2C_getData(base);
        uint8_t byte2 = I2C_getData(base);
        uint8_t byte3 = I2C_getData(base);
        msgBuffer[buff_pos++] = ((uint32_t)byte0 << 24) | ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | (uint32_t)byte3;
    }

    status = handleNACK(base);
    if (status) {
        return status;
    }
    I2C_disableFIFO(base);
    attemptCount = 1;
    while (I2C_getStopConditionStatus(base) && attemptCount <= 3U) {
        DEVICE_DELAY_US(10);
        attemptCount++;
    }

    return SUCCESS;
}



//---------------------------------------------------------------------------/
// Function Name : I2C_R_Word                                                /
// Description   : None                                                      /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t checkBusStatus(uint32_t base){
    if(I2C_isBusBusy(base)){
        return ERROR_BUS_BUSY;
    }

    if(I2C_getStopConditionStatus(base)){
        return ERROR_STOP_NOT_READY;
    }
    return SUCCESS;
}

//---------------------------------------------------------------------------/
// Function Name : I2C_R_Word                                                   /
// Description   : None                                                      /
// Argument      : None                                                      /
// Return        : None                                                      /
//---------------------------------------------------------------------------/
uint16_t handleNACK(uint32_t base){
    if(I2C_getStatus(base) & I2C_STS_NO_ACK){
        I2C_clearStatus(base, I2C_STS_NO_ACK);
        I2C_sendStopCondition(base);
        return ERROR_NACK_RECEIVED;
    }
    return SUCCESS;
}










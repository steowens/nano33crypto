extern "C" {
#include <string.h>
}
#include <Arduino.h>
#include <Wire.h>
#include "NanoCrypt.h"

const uint8_t cryptoChipSlaveAddress = 0x6A;

void NanoCrypt::hexString(uint8_t array[], unsigned int len, char buffer[], unsigned int bufferSize)
{
    memset(buffer, 0, bufferSize);
    int maxi = (len*3 <= bufferSize) ? len : (bufferSize / 3) - 1;
    unsigned int i;
    for (i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*3+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*3+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
        buffer[i*3+2] = ',';
    }
    buffer[i*3] = '\0';
}

NanoCrypt::NanoCrypt()
{
  baudRate = 100000;
  //Wire.setClock(baudRate);
}

NanoCryptCommand*  NanoCrypt::getCmd() {
     return &cmdBuffer; 
}
uint8_t NanoCrypt::getSlaveAddress(){
    return cryptoChipSlaveAddress;
}

void NanoCrypt::blink(int onDelay, int offDelay, int n){
    for(int x = 0; x < n; x++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(onDelay);
        digitalWrite(LED_BUILTIN, LOW);
        delay(offDelay);
    }
}
ATCA_STATUS sendPacket(NanoCryptCommand *cmdBuffer){
    ATCA_STATUS status = ATCA_FUNC_FAIL;
    Wire.beginTransmission(cryptoChipSlaveAddress);
    Wire.write(&(cmdBuffer->txPacket.opcode), cmdBuffer->txPacket.txsize);
    cmdBuffer->endTransmissionResult = Wire.endTransmission();
    switch(cmdBuffer->endTransmissionResult){
        case 0:
            status = ATCA_SUCCESS;
            break;
        case 1:
            status = ATCA_BAD_PARAM;
            break;
        case 2:
            status = ATCA_WAKE_FAILED;
            break;
        case 3:
            status = ATCA_TX_FAIL;
            break;
        default:
            status = ATCA_GEN_FAIL;
            break;
    }
    return status;
}

ATCA_STATUS recieveResponse(NanoCryptCommand *cmdBuffer, uint8_t responseSize, long timeout){
    ATCA_STATUS status = ATCA_SUCCESS;
    uint8_t* rcvPtr = &(cmdBuffer->rcvBuffer[0]);
    unsigned long time = millis();
    Wire.requestFrom(cryptoChipSlaveAddress, responseSize, true);
    while(!Wire.available()){
        unsigned long now = millis();
        if(now - time > timeout) {
            status = ATCA_RX_TIMEOUT;
            break;
        }
        delay(20);
    }
    while(cmdBuffer->recieved < responseSize){
        int nAvailable = Wire.available();
        for(int x = nAvailable; x > 0; x--){
            *rcvPtr = Wire.read();
            cmdBuffer->recieved++;
            rcvPtr++;
        }
        unsigned long now = millis();
        if(now - time > timeout){
            status = ATCA_RX_TIMEOUT;
            break;
        }
    }
    return status;
}

void NanoCrypt::prepareCommand(uint8_t param1, uint8_t param2){
    memset(&cmdBuffer, 0, sizeof(cmdBuffer));
    cmdBuffer.txPacket.param1 = param1;
    cmdBuffer.txPacket.param2 = param2;
}

ATCA_STATUS NanoCrypt::selfTest(uint8_t mode, uint8_t* result)
{
    prepareCommand(mode, 0);
    ATCA_STATUS status = atSelfTest(NULL, &cmdBuffer.txPacket);
    status = sendPacket(&cmdBuffer);
    if(status != ATCA_SUCCESS)
        return status;
    status = recieveResponse(&cmdBuffer, SELFTEST_RSP_SIZE, 5000);
    uint8_t response = cmdBuffer.rcvBuffer[ATCA_RSP_DATA_IDX];
    if (response & !mode){
        return status;
    }
    if (result)
    {
        *result = response;
    }
    return status;
}

ATCA_STATUS NanoCrypt::info(uint8_t mode, uint16_t param2, uint8_t* out_data)
{
    prepareCommand(mode, param2);
    ATCA_STATUS status;
    if ((status = atInfo(NULL, &cmdBuffer.txPacket)) != ATCA_SUCCESS)
       return status;
    status = sendPacket(&cmdBuffer);
    if(status != ATCA_SUCCESS)
        return status;
    status = recieveResponse(&cmdBuffer, sizeof(cmdBuffer.rcvBuffer), 1000);
    if (out_data != NULL && cmdBuffer.rcvBuffer[ATCA_COUNT_IDX] >= 7)
    {
        memcpy(out_data, &cmdBuffer.rcvBuffer[ATCA_RSP_DATA_IDX], 4);
    }
}

/** \brief Compute the address given the zone, slot, block, and offset
 *  \param[in] zone   Zone to get address from. Config(0), OTP(1), or
 *                    Data(2) which requires a slot.
 *  \param[in] slot   Slot Id number for data zone and zero for other zones.
 *  \param[in] block  Block number within the data or configuration or OTP zone .
 *  \param[in] offset Offset Number within the block of data or configuration or OTP zone.
 *  \param[out] addr  Pointer to the address of data or configuration or OTP zone.
 *  \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS atcab_get_addr(uint8_t zone, uint16_t slot, uint8_t block, uint8_t offset, uint16_t* addr)
{
    ATCA_STATUS status = ATCA_SUCCESS;
    uint8_t mem_zone = zone & 0x03;

    if (addr == NULL)
    {
        return ATCA_BAD_PARAM;
    }
    if ((mem_zone != ATCA_ZONE_CONFIG) && (mem_zone != ATCA_ZONE_DATA) && (mem_zone != ATCA_ZONE_OTP))
    {
        return ATCA_BAD_PARAM;
    }
    do
    {
        // Initialize the addr to 00
        *addr = 0;
        // Mask the offset
        offset = offset & (uint8_t)0x07;
        if ((mem_zone == ATCA_ZONE_CONFIG) || (mem_zone == ATCA_ZONE_OTP))
        {
            *addr = block << 3;
            *addr |= offset;
        }
        else     // ATCA_ZONE_DATA
        {
            *addr = slot << 3;
            *addr  |= offset;
            *addr |= block << 8;
        }
    }
    while (0);

    return status;
}

/** \brief Executes Read command, which reads either 4 or 32 bytes of data from
 *          a given slot, configuration zone, or the OTP zone.
 *
 *   When reading a slot or OTP, data zone must be locked and the slot
 *   configuration must not be secret for a slot to be successfully read.
 *
 *  \param[in]  block   32 byte block index within the zone.
 *  \param[in]  offset  4 byte work index within the block. Ignored for 32 byte
 *                      reads.
 *  \param[out] data    Read data is returned here must be array of at least 32 bytes.
 *  \param[in]  len     Length of the data to be read. Must be either 32.
 *
 *  returns ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS NanoCrypt::readConfigZone(uint8_t block, uint8_t *data, uint8_t len)
{
    ATCA_STATUS status;
    zone = ATCA_ZONE_CONFIG;
    uint16_t addr;
    // Check the input parameters
    if (data == NULL) {
        sprintf(cmdBuffer.errorMessage, "Bad <data> param.");
        return ATCA_BAD_PARAM;
    }
    if (len != 32) {
        sprintf(cmdBuffer.errorMessage, "Bad <len> param.");
        return ATCA_BAD_PARAM;
    }

    // The get address function checks the remaining variables
    if ((status = atcab_get_addr(zone, slot, block, offset, &addr)) != ATCA_SUCCESS) {
        sprintf(cmdBuffer.errorMessage, "Failure code 0x%X on atcab_get_addr.", status);
        return status;
    }
    
    // If there are 32 bytes to read, then OR the bit into the mode
    if (len == ATCA_BLOCK_SIZE)
        zone = zone | ATCA_ZONE_READWRITE_32;
        
    prepareCommand(zone, addr);
    if ((status = atRead(NULL, &cmdBuffer.txPacket)) != ATCA_SUCCESS) {
        sprintf(cmdBuffer.errorMessage, "Failure code 0x%X on atRead.", status);
        return status;
    }
    if((status = sendPacket(&cmdBuffer)) != ATCA_SUCCESS) {
        sprintf(cmdBuffer.errorMessage, "Failure code 0x%X on sendPacket.", status);
        return status;
    }
    if((status != recieveResponse(&cmdBuffer, len + 1, 1500)) != ATCA_SUCCESS) {
        sprintf(cmdBuffer.errorMessage, "Failure code 0x%X on recieveResponse.", status);
        return status;
    }
    if(data){
        memcpy(data, &cmdBuffer.rcvBuffer[1], len);
    }
}


NanoCrypt Crypt;



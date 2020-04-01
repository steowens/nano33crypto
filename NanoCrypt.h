#ifndef NanoCrypt_h
#define NanoCrypt_h

#include "atca_status.h"
#include "atca_command.h"
#include "atca_devtypes.h"

typedef struct {
  ATCAPacket txPacket;  // Primary packet buffer
  uint8_t endTransmissionResult;
  uint8_t recieved;
  uint8_t rcvBuffer[192];
  char errorMessage[128];
} NanoCryptCommand;

class NanoCrypt
{
    public:
        NanoCrypt();

        // Helper Functions
        void blink(int onDelay, int offDelay, int n);
        uint32_t getBaudRate();
        NanoCryptCommand*  getCmd();
        void hexString(uint8_t array[], unsigned int len, char buffer[], unsigned int bufferSize);
        uint8_t getSlaveAddress();

        // Chip interface funcions
        ATCA_STATUS info(uint8_t mode, uint16_t param2, uint8_t* out_data);
        ATCA_STATUS selfTest(uint8_t mode, uint8_t* result);
        ATCA_STATUS readZone(uint8_t zone, uint16_t slot, uint8_t block, uint8_t offset, uint8_t *data, uint8_t len);
    private:
        void prepareCommand(uint8_t param1, uint8_t param2);
        char debugBuffer[200];
        uint32_t baudRate;
        NanoCryptCommand cmdBuffer;
};

extern NanoCrypt Crypt;


#endif // NanoCrypt_h

#include "Crc.hpp"

uint8_t Crc::Get_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8) {
    unsigned char ucIndex;
    while(dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

bool Crc::VerifyCrc8CheckSum(uint8_t* pchMessage, uint32_t dwLength) {
    if((pchMessage == 0) || (dwLength <= 2)) {
        return false;
    }
    unsigned char ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

uint16_t Crc::Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if(pchMessage == nullptr) {
        return 0xFFFF;
    }
    while(dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ CRC16_TAB[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}
bool Crc::VerifyCrc16CheckSum(uint8_t* pchMessage, uint32_t dwLength) {
    if((pchMessage == nullptr) || (dwLength <= 2)) {
        return false;
    }
    uint16_t wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

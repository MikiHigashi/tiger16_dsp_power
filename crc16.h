#ifndef CRC16_H
#define CRC16_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

uint16_t crc16(unsigned short crc, uint8_t const *ptr, int len);

#endif	//CRC16
/**
 End of File
*/


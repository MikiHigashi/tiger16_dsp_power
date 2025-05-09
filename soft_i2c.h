#ifndef SOFT_I2C_H
#define SOFT_I2C_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

       
int I2C_start();
int I2C_send(unsigned char);
unsigned char I2C_ackchk();
int I2C_acksnd();
int I2C_nacksnd();
unsigned char I2C_rcv();
int I2C_stop();


#endif	//SOFT_I2C_H
/**
 End of File
*/
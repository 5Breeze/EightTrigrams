/*-------------------------------------------------*/
/*                                                */
/*            实现内部eeprom功能的头文件            */
/*                                                 */
/*-------------------------------------------------*/
 
#ifndef _DATA_STORAGE_H_
#define _DATA_STORAGE_H_
 
#include "main.h"
#include <string.h>  //包含需要的头文件
void Write_Float_to_EEPROM(float data, uint8_t floatIndex);

float Read_Float_from_EEPROM(uint8_t floatIndex);
 
#endif

/*-------------------------------------------------*/
/*                                                */
/*            ʵ���ڲ�eeprom���ܵ�ͷ�ļ�            */
/*                                                 */
/*-------------------------------------------------*/
 
#ifndef _DATA_STORAGE_H_
#define _DATA_STORAGE_H_
 
#include "main.h"
#include <string.h>  //������Ҫ��ͷ�ļ�
void Write_Float_to_EEPROM(float data, uint8_t floatIndex);

float Read_Float_from_EEPROM(uint8_t floatIndex);
 
#endif

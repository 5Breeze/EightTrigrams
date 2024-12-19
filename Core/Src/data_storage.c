#include "data_storage.h"     //包含需要的头文件

void Write_Float_to_EEPROM(float data, uint8_t floatIndex)
{
    uint32_t address = 0x08080000 + (floatIndex * sizeof(float)); // 计算目标地址
    uint32_t temp;
    
    HAL_FLASHEx_DATAEEPROM_Unlock(); // 解锁数据EEPROM区域
    
    // 将float转换为uint32_t以备写入
    memcpy(&temp, &data, sizeof(float));

    // 写入数据
    if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, temp) != HAL_OK)
    {
        // 错误处理代码
        while(1);
    }

    HAL_FLASHEx_DATAEEPROM_Lock(); // 锁定数据EEPROM区域
}
float Read_Float_from_EEPROM(uint8_t floatIndex)
{
    uint32_t address = 0x08080000 + (floatIndex * sizeof(float)); // 计算目标地址
    uint32_t temp;
    float data;
    
    // 将从Flash读取的数据存入临时变量temp
    memcpy(&temp, (void*)address, sizeof(uint32_t));

    // 将读取到的uint32_t转换回float
    memcpy(&data, &temp, sizeof(float));

    return data;
}
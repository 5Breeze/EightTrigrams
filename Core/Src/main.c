/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix.h"
#include "data_storage.h"
#include "yaw_calculation.h"
#include "low_power.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	#define LIS2MDL_I2C_ADDR  (0x1E << 1) // LIS2MDL的I2C地址

// LIS2DW12 I2C地址（根据SA0引脚选择为0x18或0x19）
#define LIS2DW12_I2C_ADDR  (0x19 << 1) 

extern I2C_HandleTypeDef hi2c1; // 确保I2C已初始化

void LIS2DW12_InitForSingleConversion(void) {
    uint8_t config;
	
		// 0. 软件复位，寄存器默认值
		config = 0x40; // SOFT_RESET = 1 (bit 6)
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x21, 1, &config, 1, HAL_MAX_DELAY);

	  // 2. 设置CTRL2：使能BDU和地址自增
    config = 0x0C; // BDU=1
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x21, 1, &config, 1, HAL_MAX_DELAY);
	
    // 1. 设置CTRL1：单次转换模式，低功耗模式1
    config = 0x50; // ODR=200Hz, MODE[1:0]=10, LP_MODE[1:0]=00
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x20, 1, &config, 1, HAL_MAX_DELAY);
	
    // 3. 设置CTRL3：配置单次转换触发器（SLP_MODE_SEL=1, SLP_MODE_1=0）
    config = 0x02; // SLP_MODE_SEL=1
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x22, 1, &config, 1, HAL_MAX_DELAY);
	
	
}

void LIS2DW12_ConfigActivityDetection(void) {
    uint8_t config;

		// 0. 软件复位，寄存器默认值
		config = 0x40; // SOFT_RESET = 1 (bit 6)
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x21, 1, &config, 1, HAL_MAX_DELAY);
	
    // 1. 设置为低功耗模式，ODR = 1.6Hz，低功耗模式 1（12位分辨率，低功耗）
    config = 0x10; // ODR = 1.6 Hz, MODE = 00 (Low Power Mode 1)
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x20, 1, &config, 1, HAL_MAX_DELAY);
	
    // 2. 设置唤醒阈值：阈值为 0.125g
    config = 0x04; // WK_THS = 0x04
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x34, 1, &config, 1, HAL_MAX_DELAY);
	
	  // 2. +-2g量程，低噪，中带宽
		config = 0x44; // 设置 BW_FILT = 01
		HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x25, 1, &config, 1, HAL_MAX_DELAY);


    // 3. 设置不活动持续时间：约为 1s/1LSB
    config = 0x20; // SLEEP_DUR = 0x02
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x35, 1, &config, 1, HAL_MAX_DELAY);

    // 4. 配置滤波器，使能中断
    config = 0x20; 
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x3F, 1, &config, 1, HAL_MAX_DELAY);

    // 5. 使能唤醒中断，并输出到 INT1 引脚
    config = 0x20; // INT1_WU: 唤醒中断
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x23, 1, &config, 1, HAL_MAX_DELAY);
}

// 读取加速度计数据
void LIS2DW12_ReadAcceleration(int16_t *accX, int16_t *accY, int16_t *accZ) {
    uint8_t data[6]; // 存储 X、Y、Z 数据寄存器的值
		uint8_t status;
    // 2. 等待数据准备完成 (DRDY=1)
    do {
        HAL_I2C_Mem_Read(&hi2c1, LIS2DW12_I2C_ADDR, 0x27, 1, &status, 1, HAL_MAX_DELAY);
    } while (!(status & 0x01)); // STATUS寄存器的DRDY位

    // 3. 按顺序读取加速度数据寄存器
    HAL_I2C_Mem_Read(&hi2c1, LIS2DW12_I2C_ADDR, 0x28 | 0x80, 1, data, 6, HAL_MAX_DELAY);
		
    // 4. 合并高低字节数据
    *accX = (int16_t)(data[1] << 8 | data[0]);
    *accY = (int16_t)(data[3] << 8 | data[2]);
    *accZ = (int16_t)(data[5] << 8 | data[4]);
}

void LIS2DW12_ReadSingleConversion(int16_t *accX, int16_t *accY, int16_t *accZ) {
    uint8_t status;
	uint8_t status2;
    uint8_t data[6];
    uint8_t config;

    // 1. 触发单次转换
    config = 0x03; // 设置SLP_MODE_SEL=1, SLP_MODE_1=0
    HAL_I2C_Mem_Write(&hi2c1, LIS2DW12_I2C_ADDR, 0x22, 1, &config, 1, HAL_MAX_DELAY);

    // 2. 等待数据准备完成 (DRDY=1)
    do {
        HAL_I2C_Mem_Read(&hi2c1, LIS2DW12_I2C_ADDR, 0x27, 1, &status, 1, HAL_MAX_DELAY);
    } while (!(status & 0x01)); // STATUS寄存器的DRDY位

    // 3. 按顺序读取加速度数据寄存器
    HAL_I2C_Mem_Read(&hi2c1, LIS2DW12_I2C_ADDR, 0x28 | 0x80, 1, data, 6, HAL_MAX_DELAY);
		
    // 4. 合并高低字节数据
    *accX = (int16_t)(data[1] << 8 | data[0]);
    *accY = (int16_t)(data[3] << 8 | data[2]);
    *accZ = (int16_t)(data[5] << 8 | data[4]);
}


void LIS2MDL_Init(void) {
    uint8_t cfg_reg_a = 0x81; // 单模低功耗模式（MD[1:0]=01, LP=1）
    uint8_t cfg_reg_c = 0x01; // 启用数据就绪信号（DRDY_on_PIN=1）
		cfg_reg_c |= (1 << 4); //启用BDU
    HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_I2C_ADDR, 0x60, 1, &cfg_reg_a, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_I2C_ADDR, 0x62, 1, &cfg_reg_c, 1, HAL_MAX_DELAY);
}

void LIS2MDL_ReadMagnetometer(int16_t *magX, int16_t *magY, int16_t *magZ) {
    uint8_t status;
    uint8_t data[6];
    uint8_t cfg_reg_a = 0x81; // 单模低功耗模式（MD[1:0]=01, LP=1）
    HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_I2C_ADDR, 0x60, 1, &cfg_reg_a, 1, HAL_MAX_DELAY);
    // 等待数据准备就绪
    do {
        HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_I2C_ADDR, 0x67, 1, &status, 1, HAL_MAX_DELAY);
    } while (!(status & 0x08)); // STATUS_REG Zyxda位

    // 读取数据
    HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_I2C_ADDR, 0x68, 1, data, 6, HAL_MAX_DELAY);

    // 解析数据（16位补码格式）
    *magX = (int16_t)(data[1] << 8 | data[0]);
    *magY = (int16_t)(data[3] << 8 | data[2]);
    *magZ = (int16_t)(data[5] << 8 | data[4]);
}

void I2C_Scan(uint8_t *found_devices, uint8_t *device_count) {
    uint8_t address;
    uint8_t i2c_address;
    HAL_StatusTypeDef result;

    *device_count = 0; // 初始化找到的设备数量

    for (address = 1; address < 128; address++) {
        /* I2C 地址是 7 位，需要左移 1 位 */
        i2c_address = address << 1;

        /* 检测设备是否应答 */
        result = HAL_I2C_IsDeviceReady(&hi2c1, i2c_address, 1, HAL_MAX_DELAY);

        if (result == HAL_OK) {
            /* 如果设备响应，将其地址保存到数组中 */
            found_devices[*device_count] = address;
            (*device_count)++;
        }
    }
}


// 统计参数
float calcAverages(const short* x, const short* y, const short* z, int num_points, float* params) {
    //1th
		long long sum_x = 0, sum_y = 0, sum_z = 0;
		//2th
    long long sum_xx = 0, sum_yy = 0, sum_zz = 0;
    long long sum_xy = 0, sum_xz = 0, sum_yz = 0;
	  //3th
		long long sum_xxx = 0, sum_yyy = 0, sum_zzz = 0;
		long long sum_xxy = 0, sum_xxz = 0, sum_xyy = 0;
		long long sum_xzz = 0, sum_yyz = 0, sum_yzz = 0;
		//4th
	  long long sum_yyyy = 0, sum_zzzz = 0, sum_xxyy = 0, sum_xxzz = 0, sum_yyzz = 0;
 
  	for (int i = 0; i < num_points; i++) {
				// 一次项
				sum_x += (long long) x[i];
			
				sum_y += (long long) y[i];
				sum_z += (long long) z[i];
		
				// 二次项
				sum_xx += (long long) x[i] * x[i];
				sum_yy += (long long) y[i] * y[i];
				sum_zz += (long long) z[i] * z[i];
				sum_xy += (long long) x[i] * y[i];
				sum_xz += (long long) x[i] * z[i];
				sum_yz += (long long) y[i] * z[i];
		
				// 三次项
				sum_xxx += (long long) x[i] * x[i] * x[i];
				sum_yyy += (long long) y[i] * y[i] * y[i];
				sum_zzz += (long long) z[i] * z[i] * z[i];
				sum_xxy += (long long) x[i] * x[i] * y[i];
				sum_xxz += (long long) x[i] * x[i] * z[i];
				sum_xyy += (long long) x[i] * y[i] * y[i];
				sum_xzz += (long long) x[i] * z[i] * z[i];
				sum_yyz += (long long) y[i] * y[i] * z[i];
				sum_yzz += (long long) y[i] * z[i] * z[i];
		
				// 四次项
				sum_yyyy += (long long) y[i] * y[i] * y[i] * y[i];
				sum_zzzz += (long long) z[i] * z[i] * z[i] * z[i];
				sum_xxyy += (long long) x[i] * x[i] * y[i] * y[i];
				sum_xxzz += (long long) x[i] * x[i] * z[i] * z[i];
				sum_yyzz += (long long) y[i] * y[i] * z[i] * z[i];
    }

    // 转换为浮点数平均值
    params[0] = sum_x / (float)num_points;    // x_avg
    params[1] = sum_y / (float)num_points;    // y_avg
    params[2] = sum_z / (float)num_points;    // z_avg
    params[3] = sum_xx / (float)num_points;   // xx_avg
    params[4] = sum_yy / (float)num_points;   // yy_avg
    params[5] = sum_zz / (float)num_points;   // zz_avg
    params[6] = sum_xy / (float)num_points;   // xy_avg
    params[7] = sum_xz / (float)num_points;   // xz_avg
    params[8] = sum_yz / (float)num_points;   // yz_avg
		params[9] = sum_xxx / (float)num_points;   // xxx_avg
		params[10] = sum_yyy / (float)num_points; // yyy_avg
		params[11] = sum_zzz / (float)num_points; // zzz_avg
		params[12] = sum_xxy / (float)num_points; // xxy_avg
		params[13] = sum_xxz / (float)num_points; // xxz_avg
		params[14] = sum_xyy / (float)num_points; // xyy_avg
		params[15] = sum_xzz / (float)num_points; // xzz_avg
		params[16] = sum_yyz / (float)num_points; // yyz_avg
		params[17] = sum_yzz / (float)num_points; // yzz_avg
		params[18] = sum_yyyy / (float)num_points; // yyyy_avg
		params[19] = sum_zzzz / (float)num_points; // zzzz_avg
		params[20] = sum_xxyy / (float)num_points; // xxyy_avg
		params[21] = sum_xxzz / (float)num_points; // xxzz_avg
		params[22] = sum_yyzz / (float)num_points; // yyzz_avg
		
    return 0;

}

void calculateEllipsoidParameters(float* result, float* center, float* radii) {
    float a1 = result[0], a2 = result[1];
    float x0 = -result[2] / 2;
    float y0 = -result[3] / (2 * a1);
    float z0 = -result[4] / (2 * a2);

    float radius_x = sqrt(x0 * x0 + a1 * y0 * y0 + a2 * z0 * z0 - result[5]);
    float radius_y = radius_x / sqrt(a1);
    float radius_z = radius_x / sqrt(a2);

    center[0] = x0;
    center[1] = y0;
    center[2] = z0;

    radii[0] = radius_x;
    radii[1] = radius_y;
    radii[2] = radius_z;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_Delay(2000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	
//uint8_t found_devices[128]; // 保存找到的设备地址
//uint8_t device_count;       // 记录设备数量
//I2C_Scan(found_devices, &device_count);
static int16_t magX, magY, magZ;
static float fmagX, fmagY, fmagZ;
static int16_t accX, accY, accZ;
static float_t yaw;



	
	#define NUM_POINTS 50  // 采样点数
short x[NUM_POINTS], y[NUM_POINTS], z[NUM_POINTS];  // 原始数据

LIS2MDL_Init();


//LIS2DW12_ConfigActivityDetection();
LIS2DW12_InitForSingleConversion();

	//for(int j=0;j<NUM_POINTS;j++)
	//{
	//	LIS2MDL_ReadMagnetometer(&magX, &magY, &magZ);
	//	x[j]=magX;
	//	y[j]=magY;
	//	z[j]=magZ;
	//	HAL_Delay(500);
	//	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
	//}
	//
	//
	//float params[23];
  //calcAverages(x,y,z,NUM_POINTS, params);
	//
	//// 初始化系数矩阵 A
	//float A[6][6] = {
	//		{params[18], params[22], params[14], params[10],  params[16], params[4]},  // yyyy_avg, yyzz_avg, xyy_avg, yyy_avg, yyz_avg, yy_avg
	//		{params[22], params[19], params[15], params[17], params[11], params[5]},  // yyzz_avg, zzzz_avg, xzz_avg, yzz_avg, zzz_avg, zz_avg
	//		{params[14], params[15], params[3],  params[6],  params[7],  params[0]},  // xyy_avg, xzz_avg, xx_avg,  xy_avg,  xz_avg,  x_avg
	//		{params[10],  params[17], params[6],  params[4],  params[8],  params[1]},  // yyy_avg, yzz_avg, xy_avg,  yy_avg,  yz_avg,  y_avg
	//		{params[16], params[11], params[7],  params[8],  params[5],  params[2]},  // yyz_avg, zzz_avg, xz_avg,  yz_avg,  zz_avg,  z_avg
	//		{params[4],  params[5],  params[0],  params[1],  params[2],  1.0f}        // yy_avg,  zz_avg,  x_avg,  y_avg,  z_avg,  1
	//};
	//
	//// 初始化非齐次向量 b
	//float b[6] = {
	//		-params[20],  // -xxyy_avg
	//		-params[21],  // -xxzz_avg
	//		-params[9],   // -xxx_avg
	//		-params[12],  // -xxy_avg
	//		-params[13],  // -xxz_avg
	//		-params[3]    // -xx_avg
	//};
	//
	//
	//
	//
  //float result[6];
  //float inverse[6][6];
  //matrix_inverse(A, inverse, 6);
	//matrix__mul(inverse, b, result);
	//
  //float center[3], radii[3];
  //calculateEllipsoidParameters(result, center, radii);
	//
	//Write_Float_to_EEPROM(center[0], 0);
	//Write_Float_to_EEPROM(center[1], 1);
	//Write_Float_to_EEPROM(center[2], 2);
	//
	//Write_Float_to_EEPROM(radii[0], 3);
	//Write_Float_to_EEPROM(radii[1], 4);
	//Write_Float_to_EEPROM(radii[2], 5);
	//
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	
	float center[3], radii[3];
	center[0]=Read_Float_from_EEPROM(0);
	center[1]=Read_Float_from_EEPROM(1);
	center[2]=Read_Float_from_EEPROM(2);
	radii[0]=Read_Float_from_EEPROM(3);
	radii[1]=Read_Float_from_EEPROM(4);
	radii[2]=Read_Float_from_EEPROM(5);
	int ii=0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		LIS2MDL_ReadMagnetometer(&magX, &magY, &magZ);
		fmagX=((float)magX-center[0])/radii[0];
		fmagY=((float)magY-center[1])/radii[1];
		fmagZ=((float)magZ-center[2])/radii[2];
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET);
		HAL_Delay(1000); // 延迟100ms
//        // 转换数据单位（1.5 mG/LSB）
//        float magX_mG = magX * 1.5f;
//        float magY_mG = magY * 1.5f;
//        float magZ_mG = magZ * 1.5f;

        // 用户代码处理
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET);    
		LIS2DW12_ReadSingleConversion(&accX, &accY, &accZ);
		
		yaw = calculate_yaw((float)accX, (float)accY, (float)accZ, fmagX, fmagY, fmagZ);
		HAL_Delay(1000); // 延迟100ms
		ii++;
//		if(ii==10)
//		{
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET);
//		LIS2DW12_ConfigActivityDetection();	
//		enter_standby_mode();
//		}
		
  }
	
	
	
	


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

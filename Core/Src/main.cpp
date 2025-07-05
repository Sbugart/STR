
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx_hal_conf.h"

#include "stdio.h"
#include <cstdint>

extern "C"{
  #include "config_micro.h"
}
/* USER CODE END Includes */
#define VL53L0X_ADDR (0x52) // 7-bit left-shifted


HAL_StatusTypeDef VL53L0X_InitSimple(void)
{
  uint8_t cmd = 0x01;
  HAL_StatusTypeDef ret;
  uint8_t ready;
  // 1) Iniciar medição única (escreve 0x01 em 0x00)
  ret = HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
  if (ret != HAL_OK)
	return ret;

  do
  {
	ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &ready, 1, 100);

  } while ((ready != 0xEE));
  return ret;
}

HAL_StatusTypeDef VL53L0X_ReadSingleSimple(uint16_t *distance)
{
  HAL_StatusTypeDef ret;
  uint8_t cmd = 0x01;
  uint8_t rangeData[2];
  uint8_t clearInterrupt = 0x01;
  uint8_t ready;

  ret = HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
  do
  {
	ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &ready, 1, 100);

  } while (ready != 0xEE);

  // 3) Ler distância: 0x1E + 0x1F,,
  ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0x1E, I2C_MEMADD_SIZE_8BIT, rangeData, 2, 100);

  // 4) Limpar interrupção/status
  ret = HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &clearInterrupt, 1, 100);
  if (ret != HAL_OK) return ret;

  *distance = (rangeData[0] << 8) | rangeData[1];
  return ret;
}



int main(void){
 /* USER CODE BEGIN 2 */

	// Initialise a message buffer.
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
    MX_TIM20_Init();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	MX_I2C1_Init();
	VL53L0X_InitSimple();

	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
	Motor_SetPower(60);

	uint16_t distance;

  /* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
	while (1) {

		// uint16_t distance is the distance in millimeters.
		// statInfo_t_VL53L0X distanceStr is the statistics read from the sensor.
		VL53L0X_ReadSingleSimple(&distance);

		printf("Distance: %d\r\n", distance);

	}
	/* USER CODE END WHILE */
}




/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx_hal_conf.h"

#include "stdio.h"
#include <cstdint>
/* USER CODE END Includes */
#define VL53L0X_ADDR (0x52) // 7-bit left-shifted
I2C_HandleTypeDef hi2c1;

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


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

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* --- Habilita clocks GPIOA e GPIOB --- */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB7 -> SDA (I2C1_SDA) */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* PA15 -> SCL (I2C1_SCL) */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	  /* Mode, Pull, Speed e Alternate permanecem iguais */
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

static void MX_I2C1_Init(void)
{

  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

int main(void){
 /* USER CODE BEGIN 2 */

	// Initialise a message buffer.
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	VL53L0X_InitSimple();

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



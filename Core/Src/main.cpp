
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx_hal_conf.h"
#include "Pid.h"
#include "stdio.h"
#include <cstdint>

extern "C"{
  #include "config_micro.h"
}
/* USER CODE END Includes */

// ======== DEFINIÇÕES ========
#define VL53L0X_ADDR (0x52) // 7-bit left-shifted

// ======== PARÂMETROS PID ========
float valorControlador = 0;
const float KP = -0.0001f;       // KP        -  proportional gain
const float KI = -0.000012f;     // KI        -  Integral gain
const float KD =  0.00001f;      // KD        -  derivative gain
const float DT = 0.050f;         // DT        -  loop interval time
const float SAIDA_MAX = 0.3f;    // SAIDA_MAX - maximum value of manipulated variable
const float SAIDA_MIN = -0.3f;   // SAIDA_MIN - minimum value of manipulated variable

PID pid = PID(DT, SAIDA_MAX, SAIDA_MIN, KP, KD, KI);
float saidaPID = 0.0f;
const uint16_t setPoint = 200;

// ======== STACKS DAS THREADS ========
uint32_t stackControlador[40];
uint32_t stackGerarPWM[40];

// ======== DEFINIÇÃO DAS THREADS ========
//OSThread threadControlador;
//OSThread threadGerarPWM;

// ======== FUNÇÕES DO SENSOR VL53L0X ========
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

// ======== FUNÇÕES DE CONTROLE ========
void AtualizarControle(uint16_t distancia) {
    // Se quiser proteger região crítica:
    // OS_NPP_enterCriticalRegion();
    saidaPID = pid.calculate(setPoint, distancia);
    saidaPID += 0.61;
    // OS_NPP_exitCriticalRegion();
}

void AplicarPWM(void) {
    // OS_NPP_enterCriticalRegion();
    Motor_SetPower(saidaPID); // Define % da potência do motor
    // OS_NPP_exitCriticalRegion();
}

// ======== MAIN ========
int main(void){
 /* USER CODE BEGIN 2 */

	// ======== Inicializações ========
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM20_Init();
    MX_I2C1_Init();

    // Configuração do GPIO
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Start do sensor e PWM
	VL53L0X_InitSimple();
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);


	uint16_t distanciaAtual = 0;

  /* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
	while (1) {

		// uint16_t distance is the distance in millimeters.
		// statInfo_t_VL53L0X distanceStr is the statistics read from the sensor.
		VL53L0X_ReadSingleSimple(&distanciaAtual);
		AtualizarControle(distanciaAtual);
		AplicarPWM();

	}
	/* USER CODE END WHILE */
}



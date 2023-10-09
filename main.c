/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include "MS5611.h"

//#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU6050_ADDR 0xD0
//#define MPU6050_ADDR 0x68

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define MPU6050_CONFIG 0x1A
#define PI 3.1415926535897932384626433832795


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
float gyroXoffset, gyroYoffset, gyroZoffset;

float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY, angleAccZ;

float angleX, angleY, angleZ;

float x = 0, y = 0, z = 0;

float interval;
long timePrev, time;

//float  accCoef = 0.02f;
//float  gyroCoef = 0.98f;

float  accCoef = 0.98f;
float  gyroCoef = 0.02f;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t check;
uint8_t Data;
int i;
uint16_t adc_value[4];
bool flag_PWM = false;
int flag_count = 0;
#define Calibrate 1
MS5611_t MS5611;
float altitude;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Init and config parameter MPU6050
void MPU6050_InitVer1 (void)
{
	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 0x72)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL = 2 -> ? 8g
		Data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 500 ?/s
		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		
//		Data = 0x00;
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void calcGyroOffsets(bool console)
{

	int16_t rx, ry, rz;
	uint8_t Rec_Data[6];

  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    }
		// read gyro 
		HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
		
		rx = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		ry = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		rz = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
		
    x += rx / 65.5;
    y += ry / 65.5;
    z += rz / 65.5;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;
	HAL_Delay(250);

}


uint8_t Rec_Data1[6];
void MPU6050_Read_Accel (void)
{
//	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data1, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data1[0] << 8 | Rec_Data1 [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data1[2] << 8 | Rec_Data1 [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data1[4] << 8 | Rec_Data1 [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 2. So I am dividing by 4096.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/4096.0;
	Ay = Accel_Y_RAW/4096.0;
	Az = Accel_Z_RAW/4096.0;
}

uint8_t Rec_Data2[6];
void MPU6050_Read_Gyro (void)
{
	//uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data2, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data2[0] << 8 | Rec_Data2 [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data2[2] << 8 | Rec_Data2 [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data2[4] << 8 | Rec_Data2 [5]);

	/*** convert the RAW values into dps (?/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 1. So I am dividing by 65.5
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/65.5;
	Gy = Gyro_Y_RAW/65.5;
	Gz = Gyro_Z_RAW/65.5;
	
	Gx -= gyroXoffset;
  Gy -= gyroYoffset;
  Gz -= gyroZoffset;
}

void MPU6050_Read_Angle (void)
{
//	timePrev = time;
//	time = HAL_GetTick();
	angleAccX = atan(Ax / (sqrt(Az * Az + Ay * Ay))) * 360 / 2.0f / PI;
  angleAccY = atan(Ay / (sqrt(Az * Az + Ax * Ax))) * 360 / 2.0f / PI;
  angleAccZ = atan((sqrt((Ay*Ay) + Ax*Ax)) / Az) * 360 / 2.0f / PI;

//	interval = (time - timePrev) * 0.001;
	
	angleGyroX += Gx * interval;
  angleGyroY += Gy * interval;
  angleGyroZ += Gz * interval;

//  angleX = (gyroCoef * (angleX + Gx * interval)) + (accCoef * angleAccX);
//  angleY = (gyroCoef * (angleY + Gy * interval)) + (accCoef * angleAccY);
//  angleZ = (gyroCoef * (angleZ + Gz * interval)) + (accCoef * angleAccZ);
	
	angleX = (gyroCoef * angleGyroX) + (accCoef * angleAccX);
  angleY = (gyroCoef * angleGyroY) + (accCoef * angleAccY);
  angleZ = (gyroCoef * angleGyroZ) + (accCoef * angleAccZ);
	
	//prevTime = HAL_GetTick();
}
uint16_t ADC_Data=0;
int ADC_Converted[4];

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	ADC_Converted[0] = map(adc_value[0], 0, 4095, 50, 100);
//	ADC_Converted[1] = map(adc_value[1], 0, 4095, 50, 100);
//	ADC_Converted[2] = map(adc_value[2], 0, 4095, 50, 100);
//	ADC_Converted[3] = map(adc_value[3], 0, 4095, 50, 100);
//	HAL_Delay(100);

//	TIM1->CCR1 = ADC_Converted[0];  // chuyen qua thang do 50 - 100 tuong ung 1ms - 2ms
//	TIM1->CCR2 = ADC_Converted[1];
//	TIM1->CCR3 = ADC_Converted[2];
//	TIM1->CCR4 = ADC_Converted[3];
//}
void Calibrate_ESC()
{
  TIM1->CCR1 = 100;  // Set the maximum pulse (2ms)
	TIM1->CCR2 = 100;
	TIM1->CCR3 = 100;
	TIM1->CCR4 = 100;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
  HAL_Delay (2000);  // wait for 1 beep
	
  TIM1->CCR1 = 50;   // Set the minimum Pulse (1ms)
	TIM1->CCR2 = 50;
	TIM1->CCR3 = 50;
	TIM1->CCR4 = 50;
  HAL_Delay (1000);  // wait for 2 beeps
	
  TIM1->CCR1 = 0;    // reset to 0, so it can be controlled via ADC
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
}
float PID, pwm1, pwm2, pwmRear, pwmFront, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////
double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

void PIDcontrol(float angle, int direct)
{
		/*First calculate the error between the desired angle and  the real measured angle*/
		error = angle - desired_angle;
				
		/*Next the proportional value of the PID is just a proportional constant *multiplied by the error*/
		pid_p = kp*error;

		/*The integral part should only act if we are close to the
		desired position but we want to fine tune the error. That's
		why I've made a if operation for an error between -2 and 2 degree.
		To integrate we just sum the previous integral value with the
		error multiplied by  the integral constant. This will integrate (increase)
		the value each loop till we reach the 0 point*/
		if(-3 < error && error < 3)  pid_i = pid_i+(ki*error);  


		/*The last part is the derivate. The derivate acts upon the speed of the error.
		As we know the speed is the amount of error that produced in a certain amount of
		time divided by that time. For taht we will use a variable called previous_error.
		We substract that value from the actual error and divide all by the elapsed time. 
		Finnaly we multiply the result by the derivate constant*/

		pid_d = kd*((error - previous_error)/interval);

		/*The final PID values is the sum of each of this 3 parts*/
		PID = pid_p + pid_i + pid_d;

		/*We know that the min value of PWM signal is 1000us and the max is 2000. So that
		tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
		have a value of 2000us the maximum value that we could sybstract is 1000 and when
		we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
		to reach the maximum 2000us*/
		if(PID < -1000) PID = -1000;

		if(PID > 1000)	PID = 1000;


		/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
		pwm1 = throttle - PID;
		pwm2 = throttle + PID;


		/*Once again we map the PWM values to be sure that we won't pass the min
		and max values. Yes, we've already maped the PID values. But for example, for 
		throttle value of 1300, if we sum the max PID value we would have 2300us and
		that will mess up the ESC.*/
		
		//Left
		if(pwm1 < 1000)	pwm1 = 1000;
		if(pwm1 > 2000) pwm1 = 2000;
		
		//Right
		if(pwm2 < 1000) pwm2 = 1000;
		if(pwm2 > 2000) pwm2 = 2000;


		/*Finnaly using the servo function we create the PWM pulses with the calculated
		width for each pulse*/
		if (direct == 2){
			pwmLeft  = map(pwm1, 1000, 2000, 50, 100);
			pwmRight = map(pwm2, 1000, 2000, 50, 100);
			TIM1->CCR1 = pwmLeft;
			TIM1->CCR2 = pwmRight;
			TIM1->CCR3 = pwmRight;
			TIM1->CCR4 = pwmLeft;
		}
		if (direct == 1){
			pwmFront = map(pwm1, 1000, 2000, 50, 100);
			pwmRear  = map(pwm2, 1000, 2000, 50, 100);
			TIM1->CCR1 = pwmFront;
			TIM1->CCR2 = pwmFront;
			TIM1->CCR3 = pwmRear;
			TIM1->CCR4 = pwmRear;
		}
		previous_error = error; //Remember to store the previous error.
}
void controlLed()
{
			if (angleX > 30 ) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
			
				if (angleY > 30) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
				}
				else if (angleY < -30) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
				}
				else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		}
		else if (angleX < -30 ){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
			
				if (angleY > 30) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
				}
				else if (angleY < -30) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
				}
				else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		}
		else {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
				}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		flag_PWM = true;
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_InitVer1();
	HAL_Delay (250);  // wait for a while
  calcGyroOffsets(true);
	HAL_Delay (1000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	
	//	PWM--------------------------------------------
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//#if Calibrate
//  TIM1->CCR1 = 100;  // Set the maximum pulse (2ms)
//  HAL_Delay (2000);  // wait for 1 beep
//	HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_13, 1);
//  TIM1->CCR1 = 50;   // Set the minimum Pulse (1ms)
//  HAL_Delay (1000);  // wait for 2 beeps
//  TIM1->CCR1 = 0;    // reset to 0, so it can be controlled via ADC
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
//#endif

	// ADC----------------------------------------------
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,4);
	
	// MS5611
	MS5611_Reset(&hi2c1, &MS5611);
	MS5611_ReadProm(&hi2c1, &MS5611);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)	
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
		
		timePrev = time;
		time = HAL_GetTick();
		
		MPU6050_Read_Angle();
		
		interval = (time - timePrev) * 0.001;
		
		PIDcontrol(angleX, 1);
		PIDcontrol(angleY, 2);
		HAL_Delay(250);  // wait for a while

		if (flag_count == 0 && flag_PWM ) {
				TIM1->CCR1 = 108;  // Set the maximum pulse (2ms)
				HAL_Delay (2000);  // wait for 1 beep
				HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_13, 1);
				TIM1->CCR1 = 50;   // Set the minimum Pulse (1ms)
				HAL_Delay (1000);  // wait for 2 beeps
				TIM1->CCR1 = 0;    // reset to 0, so it can be controlled via ADC
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
				flag_count++;
		}
//		if (flag_PWM) {
//			TIM1->CCR1 = 108;
//			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 100);
//			float temp = map(80, 0, 100, 50, 100);
//			TIM1->CCR1 = temp;
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
//			HAL_Delay(15000);
//		}
		// Read Altitude
				MS5611_ReadPressure(&hi2c1,&MS5611);
				MS5611_CalculatePressure(&MS5611);
				MS5611_RequestTemperature(&hi2c1,OSR_4096);
				MS5611.Alt = (MS5611_getAltitude1((float)MS5611.P/100.f))*100;

				#define X 0.90f
				MS5611.Alt_Filt = MS5611.Alt_Filt * X + MS5611.Alt * (1.0f-X);
				altitude = MS5611.Alt_Filt;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

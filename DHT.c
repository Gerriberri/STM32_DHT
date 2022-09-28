/*
 * DHT.c
 *
 *  Created on: Nov 25, 2021
 *      Author: Lucas
 */

#include "DHT.h"

void delay_us(uint16_t us, TIM_HandleTypeDef htim)
{
	__HAL_TIM_SET_COUNTER(&htim,0);  // set the counter value a 0
  	while (__HAL_TIM_GET_COUNTER(&htim) < us);  // wait for the counter to reach the us input in the parameter
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  	GPIO_InitStruct.Pin = GPIO_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  	GPIO_InitStruct.Pin = GPIO_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef htim)
{
	Set_Pin_Output(GPIOx, GPIO_Pin); // set the pin as output
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);   // pull the pin low
	delay_us(1200, htim);   // wait for > 1ms

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);   // pull the pin high
	delay_us(20, htim);   // wait for 30us

	Set_Pin_Input(GPIOx, GPIO_Pin);   // set as input
}

uint8_t DHT22_Check_Response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef htim)
{
	Set_Pin_Input(GPIOx, GPIO_Pin);   // set as input
	uint8_t Response = 0;
	delay_us(40, htim);  // wait for 40us
	if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) // if the pin is low
	{
		delay_us(80, htim);   // wait for 80us

		if ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));   // wait for the pin to go low
	return Response;
}

uint8_t DHT22_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef htim)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));   // wait for the pin to go high
		delay_us(40, htim);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));  // wait for the pin to go low
	}

	return i;
}

void DHT_GetData (DHT_DataTypedef *DHT_Data, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef htim)
{
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint8_t Presence = 0;
	uint16_t SUM;

	DHT22_Start(GPIOx, GPIO_Pin, htim);
	Presence = DHT22_Check_Response(GPIOx, GPIO_Pin, htim);
	Rh_byte1 = DHT22_Read(GPIOx, GPIO_Pin, htim);
	Rh_byte2 = DHT22_Read(GPIOx, GPIO_Pin, htim);
	Temp_byte1 = DHT22_Read(GPIOx, GPIO_Pin, htim);
	Temp_byte2 = DHT22_Read(GPIOx, GPIO_Pin, htim);
	SUM = DHT22_Read(GPIOx, GPIO_Pin, htim);

	if (SUM == ( (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2) & 0x00FF) )
	{
		DHT_Data->Temperature = ( ((Temp_byte1 << 8) | Temp_byte2) * 0.1f);
		DHT_Data->Humidity = ( ((Rh_byte1 << 8) | Rh_byte2) * 0.1f);
	}
}



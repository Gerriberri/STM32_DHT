/*
 * DHT.h
 *
 *  Created on: Nov 25, 2021
 *      Author: Lucas
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_


#include "main.h"

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;

void DHT_GetData(DHT_DataTypedef *DHT_Data, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef htim);

#endif /* INC_DHT_H_ */

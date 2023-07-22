#include "keyscan.h"


void row(int r){
	switch (r){
		case 0:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
			break;
	}
}

int colume(int c){
	int ret;
	switch (c){
		case 0:
			ret = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
			break;
		case 1:
			ret = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
			break;
		case 2:
			ret = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14);
			break;
		case 3:
			ret = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15);
			break;
	}
	return ret;
}

int keyscan(void){
	int key = 0;
	int r = 0;
	int c = 0;
	for(r = 0; r < 4; r++){
		row(r);
		for(c = 0; c < 4; c++){
			if(colume(c)){
				key = r*4+c+1;
			}
		}
	}

	
	return key;
}


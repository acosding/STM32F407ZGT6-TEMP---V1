#include "motor.h"

extern TIM_HandleTypeDef htim5;

void motor(int index,int speed){
	//limit speed 
	if(speed>1000){
		speed = 1000;
	}
	if(speed < -1000){
		speed = -1000;
	}
	//switch the motor
	if(index == 1){
		if(speed > 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,speed);
		}else if(speed < 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000+speed);
		}else if(speed == 0){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000);
		}
	}else if(index == 2){
		if(speed > 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,speed);
		}else if(speed < 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,1000+speed);
		}else if(speed == 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,1000);
		}
	}else if(index == 3){
		if(speed > 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,1000-speed);
		}else if(speed < 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,-speed);
		}else if(speed == 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,1000);
		}
	}else if(index == 4){
		if(speed > 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,speed);
		}else if(speed < 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,1000+speed);
		}else if(speed == 0){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,1000);
		}
	}
}

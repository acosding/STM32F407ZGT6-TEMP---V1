#include "encoder.h"

motor_encoder motor1;
motor_encoder motor2;
motor_encoder motor3;
motor_encoder motor4;

float filter[4][3];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			motor1.pluse++;
			if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_5) == GPIO_PIN_SET){
				motor1.direction = 1;
				motor1.position++;
			}else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_5) == GPIO_PIN_RESET){
				motor1.direction = -1;
				motor1.position--;
			}
			if(motor1.position>M1ENCODER_POS_MAX){
				motor1.position = -M1ENCODER_POS_MAX;
			}else if(motor1.position<-M1ENCODER_POS_MAX){
				motor1.position = M1ENCODER_POS_MAX;
			}
			//
		}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			motor2.pluse++;
			if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6) == GPIO_PIN_SET){
				motor2.direction = 1;
				motor2.position++;
			}else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6) == GPIO_PIN_RESET){
				motor2.direction = -1;
				motor2.position--;
			}
			if(motor2.position>M2ENCODER_POS_MAX){
				motor2.position = -M2ENCODER_POS_MAX;
			}else if(motor2.position<-M2ENCODER_POS_MAX){
				motor2.position = M2ENCODER_POS_MAX;
			}
			//
		}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			motor3.pluse++;
			if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7) == GPIO_PIN_SET){
				motor3.direction = -1;
				motor3.position--;
			}else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7) == GPIO_PIN_RESET){
				motor3.direction = 1;
				motor3.position++;
			}
			if(motor3.position>M3ENCODER_POS_MAX){
				motor3.position = -M3ENCODER_POS_MAX;
			}else if(motor3.position<-M3ENCODER_POS_MAX){
				motor3.position = M3ENCODER_POS_MAX;
			}
			//
		}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			motor4.pluse++;
			if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8) == GPIO_PIN_SET){
				motor4.direction = 1;
				motor4.position++;
			}else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8) == GPIO_PIN_RESET){
				motor4.direction = -1;
				motor4.position--;
			}
			if(motor4.position>M4ENCODER_POS_MAX){
				motor4.position = -M4ENCODER_POS_MAX;
			}else if(motor4.position<-M4ENCODER_POS_MAX){
				motor4.position = M4ENCODER_POS_MAX;
			}
		}
	}
}

float readencoder(int motor_index){
	float output = 0;
	if(motor_index == 1){
		output = motor1.speed;
	}else if(motor_index == 2){
		output = motor2.speed;
	}else if(motor_index == 3){
		output = motor3.speed;
	}else if(motor_index == 4){
		output = motor4.speed;
	}
		filter[motor_index-1][0] = output;
		output = 0.25f*filter[motor_index-1][0] + 0.5f*filter[motor_index-1][1] + 0.25f*filter[motor_index-1][2];
		filter[motor_index-1][2] = filter[motor_index-1][1];
		filter[motor_index-1][1] = filter[motor_index-1][0];
	return output;
}

int readencoderlength(int motor_index){
	float output = 0;
	if(motor_index == 1){
		output = motor1.position;
	}else if(motor_index == 2){
		output = motor2.position;
	}else if(motor_index == 3){
		output = motor3.position;
	}else if(motor_index == 4){
		output = motor4.position;
	}
	return output;
}

void encoder_init(int motor_index){
	if(motor_index == 1){
		motor1.direction = 0;
		motor1.pluse = 0;
		motor1.speed = 0;
		motor1.position = 0;
	}else if(motor_index == 2){
		motor2.direction = 0;
		motor2.pluse = 0;
		motor2.speed = 0;
		motor2.position = 0;
	}else if(motor_index == 3){
		motor3.direction = 0;
		motor3.pluse = 0;
		motor3.speed = 0;
		motor3.position = 0;
	}else if(motor_index == 4){
		motor4.direction = 0;
		motor4.pluse = 0;
		motor4.speed = 0;
		motor4.position = 0;
	}
}
void encoder_reset(int motor_index){
	if(motor_index == 1){
		motor1.position = 0;
	}else if(motor_index == 2){
		motor2.position = 0;
	}else if(motor_index == 3){
		motor3.position = 0;
	}else if(motor_index == 4){
		motor4.position = 0;
	}
}

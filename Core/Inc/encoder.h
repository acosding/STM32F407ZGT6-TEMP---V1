#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "main.h"
#include "tim.h"

#define M1ENCODER_POS_MAX 200000000
#define M2ENCODER_POS_MAX 200000000
#define M3ENCODER_POS_MAX 200000000
#define M4ENCODER_POS_MAX 200000000

typedef struct{
	int pluse;
	int direction;
	int position;
	float speed;
}motor_encoder;



float readencoder(int motor_index);
int readencoderlength(int motor_index);
void encoder_init(int motor_index);
void encoder_reset(int motor_index);


#endif

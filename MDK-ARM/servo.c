#include <main.h>
#include <math.h>
#include "servo.h"

float servo1Write(float goc)
{
	float kq = ((goc * 95.0 + 500.0 * 9.0) / (9.0)); //goc*(95/9) + 500.0;
	return kq;
}

//===================================================
float servo2Write(float goc)
{
	float kq = ((goc * 191.0 + 500.0 * 18.0) / (18.0)); //goc*(191/18) + 500.0;
	return kq;
}

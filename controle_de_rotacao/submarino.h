#ifndef Propulsores_h
#define Propulsores_h

#include <Arduino.h>

class Propulsores
{
	private:
	unsigned long prevTime,currentTime, dt;
	int pinOut_1, pinOut_2,direction; //horario 1 anti-horario 2;
	float minValue = 111, maxValue = 222,velocidade;
	float kp, ki, kd;
	float p, i, d, pid;
	

	public:
	propulsores(int pinOut_1, int pinOut_2,float kp, float ki, float kd);
	void acionarPropulsor(int direction, float velocidade);
	float pid(float setPoint, float value, unsigned long prevTime, unsigned long currentTimefloat, minValue, maxValue);

}	

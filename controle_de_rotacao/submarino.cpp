#include <Propulsores.h>


Submarino::Submarino(int pinOut_1, int pinOut_2,float kp, float ki, float kd)
{
	pinOut_1 = pinOut_1;
	pinOut_2 = pinOut_2;
	kp = kp;
	ki = ki;
	kd = kd;
}

Submarino::acionarPropulsor(int direction, float velocidade){
	if(direction == 1)
		analogWrite(pinOut_1,velocidade);	
	else
		analogWrite(pinOut_2,velocidade);



}
Submarino::pid(float setPoint, float value, unsigned long prevTime, unsigned long currentTime, float minValue, float maxValue){
	float error = setPoint - value;
	dt = currentTime - prevTime;
	p = kp * error;
	i = ki * error * dt;
	d = kd * error / dt;
	pid = p + i + d;
	return (map(velocidade,-180,180,-62,62)+160);
	//return pid;
}
 
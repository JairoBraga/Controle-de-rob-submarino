#ifndef SUBMARINO_H
#define SUBMARINO_H

#define propulsor_direito_horario 3
#define	propulsor_direito_anti_h 5
#define propulsor_esquerdo_horario 6
#define propulsor_esquerdo_anti_h 9
#define	submersor_direito_horario 10
#define submersor_direito_anti_h 7
#define submersor_esquerdo_horario 11
#define submersor_esquerdo_anti_h 8


#include <Arduino.h>

//Imersão não tera pwm somente atuara os dois motores em carga máxima com pinos digitais
//Devido a falta de pinos de pwm disponiveis no arduino uno

//Nota de controle: 
//Quem vai determinar qual motor será acionado será quem chamou			
class Submarino
{
	private:
	float kp, ki, kd;
	float p, i, d, pid;
	float yawPoint, rollPoint, pitchPoint;
  float minVelocidade = 111;
  float curveValue = 155;
  unsigned long prevTime, currentTime, dt;

	public:
	Submarino(float kp, float ki, float kd);
	int controle(int comando, float yaw,float pitch, float roll);
	int acionarPropulsor(int pinMotor, float velocidade);
	int PID(float setPoint, float valueGyro);
};	
#endif

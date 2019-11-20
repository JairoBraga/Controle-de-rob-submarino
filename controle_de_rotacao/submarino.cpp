#include "Submarino.h"

Submarino::Submarino(float kp, float ki, float kd)
{
	kp = kp;
	ki = ki;
	kd = kd;
	rollPoint = 0;
	yawPoint = 0;
	pitchPoint = 0;
	
}

Submarino::controle(int comando, float yawGyro,float pitchGyro, float rollGyro){
	//falta adicionar a submersão
	switch(comando){
    case 's':
        acionarPropulsor(minVelocidade,submersor_direito_anti_h);
        acionarPropulsor(minVelocidade,submersor_esquerdo_horario);
        break;
        
    case 'i':
        acionarPropulsor(0,submersor_direito_anti_h);
        acionarPropulsor(0,submersor_esquerdo_horario);
        break;
        
		case 'f'://frente
				acionarPropulsor(PID(yawPoint,yawGyro), propulsor_direito_horario);
				acionarPropulsor(minVelocidade, propulsor_esquerdo_horario);
				acionarPropulsor(PID(rollPoint,rollGyro), submersor_direito_horario);
				acionarPropulsor(minVelocidade, submersor_esquerdo_horario); // setpoint vai ser 0	
		break;
    
		case 'r':
        acionarPropulsor(PID(yawPoint,yawGyro), submersor_direito_anti_h);				
				acionarPropulsor(minVelocidade, propulsor_esquerdo_anti_h);
				acionarPropulsor(PID(rollPoint,rollGyro), submersor_direito_horario);
				acionarPropulsor(minVelocidade, submersor_esquerdo_horario); // setpoint vai ser 0	
		break;
    
		case 'd'://direita
				acionarPropulsor(minVelocidade, propulsor_direito_anti_h);
				acionarPropulsor(minVelocidade, propulsor_esquerdo_horario);
				acionarPropulsor(PID(rollPoint,rollGyro), submersor_esquerdo_horario); // setpoint vai ser 0
				acionarPropulsor(minVelocidade, submersor_direito_horario);
				rollPoint = rollGyro;
		break;
    
		case 'e'://esquerda
				acionarPropulsor(minVelocidade, propulsor_direito_horario);
				acionarPropulsor(minVelocidade, propulsor_esquerdo_anti_h);
				acionarPropulsor(PID(rollPoint,rollGyro), submersor_esquerdo_horario); // setpoint vai ser 0
				acionarPropulsor(minVelocidade, submersor_direito_horario);
				rollPoint = rollGyro;
	  break;
    case 'o':
        //desliga todo os propulsores para evitar acionamento dos 4 transistores
        acionarPropulsor(0,propulsor_direito_horario);
        acionarPropulsor(0,propulsor_direito_anti_h);
        acionarPropulsor(0,propulsor_esquerdo_horario);
        acionarPropulsor(0,propulsor_esquerdo_anti_h);
		break;
//		case 'f'://frentedireita
//       	acionarPropulsor(minVelocidade, propulsor_direito_anti_h);
//				acionarPropulsor(curveValue, propulsor_esquerdo_horario);
//				acionarPropulsor(PID(rollPoint,rollGyro), submersor_esquerdo_horario); // setpoint vai ser 0
//				acionarPropulsor(minVelocidade, submersor_direito_horario);
//				rollPoint = rollGyro;
//		break;
//		case 'fe'://frenteesquerda
//				acionarPropulsor(curveValue, propulsor_direito_anti_h);
//				acionarPropulsor(minVelocidade, propulsor_esquerdo_horario);
//				acionarPropulsor(PID(rollPoint,rollGyro), submersor_esquerdo_horario); // setpoint vai ser 0
//				acionarPropulsor(minVelocidade, submersor_direito_horario);
//				rollPoint = rollGyro;
//		break;

    
    
    break;
	}	
}	
	
Submarino::acionarPropulsor(int pinMotor, float velocidade){
		analogWrite(pinMotor,velocidade);

}


Submarino::PID(float setPoint, float valueGyro){
  currentTime = millis();
  if(currentTime - prevTime > 1){
  prevTime = currentTime;
	float error = setPoint - valueGyro;
	dt = currentTime - prevTime;
	p = kp * error;
	i = ki * error * dt;
	d = kd * error / dt;
	pid = p + i + d;
	pid = (map(pid, -180, 180,-100, 100) + minVelocidade);	
}
}

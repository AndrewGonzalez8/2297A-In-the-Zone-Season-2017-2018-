#pragma config(Sensor, in3,    lineleftup,     sensorLineFollower)
#pragma config(Sensor, in4,    reversepot,     sensorPotentiometer)
#pragma config(Sensor, in5,    fourbarpot,     sensorPotentiometer)
#pragma config(Sensor, in6,    linerightup,    sensorLineFollower)
#pragma config(Sensor, in7,    potmob,         sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  Encoder_R,      sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  Encoder_L,      sensorQuadEncoder)
#pragma config(Motor,  port1,           downleft,      tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           yr,            tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           yl,            tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           mob,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           rol,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port6,           arml,          tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           fourbl,        tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           fourbr,        tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           armr,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          downright,     tmotorVex393HighSpeed_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#define MotorMaximum       127
#define MotorMinimum     (-127)
#define MobileMaximum     2600
#define MobileMinimum     300
static float MaximumorMinimumMobile;
static float MaximumorMinimum;
static float MaximumorMinimumArm;

/*          Copyright (C) <2018>  Andrew Gonzalez Perez              *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    Special thanks to 2223z's programmer, Marcos Pesante for his           *
*    collaborations on this project.                                        *
*    Part of this code was retreived from                                   *
*    https://github.com/M4rqu1705/2223Z-In-the-Zone/tree/master/RobotC      *
*    Also, I'd like to thank Pedro Alvarado from                            *
*    the Puerto Rico Institute of Robotics (PRIOR),                         *
*    for his contributions as well.                                         */


float Kp=1;
float Ki=0;
float Kd=0.5;
float Gp=0.5;
float Gi=0;
float Gd=0.2;
float Cp=0.2;
float Ci=0;
float Cd=0;
float Ap=0.36;
float Ai=0.01;
float Ad=0;
float Mp=0.1;
float Mi=0;
float Md=0;
////////////////////////////////////////*Functions for autonomous and driver control*////////////////////////////////////////////////////

void Base_Forward(float Distance_I, float Seconds)
{
	Distance_I *= 24.66; //This value is obtained from the formula: 360/(4.0*3.14) and then adding or subtracting from it.
	Distance_I = round(Distance_I);//This line makes sure that the value returned from Distance_I is an integer.
  Seconds*= (1000/15);//This converts how many seconds do you want the program to run.
	float Power, Error, Integral=0, Derivative, Prev_Error=0, Error_E;//Variables
	/*
	This part resets the encoders.
	*/
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	/*
	//This makes sure that the program only runs for the time you assign it to.
	*/
	for(int counter =0; counter<Seconds;	counter++)
	{
		Error=(Distance_I - -1*(SensorValue[Encoder_R]));//Calculates error to adjust the values of the PID.

		Integral=Integral+Error;//Calculates integral part of the PID.


		if(Error==0)//Adjust integral error
		{
			Integral=0;
		}
		Derivative=Error-Prev_Error;//Calculates derivative part of the PID.
		Prev_Error=Error;//Adjusts the D term based on the distance.
		Power=(Error*Kp)+(Integral*Ki)+(Derivative*Kd);//Set power correction for motors based on the PID

		/*
		This part limits the power assigned to the motors.
		*/
		if(Power>MotorMaximum)
		{
			Power=MotorMaximum;
		}
		if(Power< MotorMinimum)
		{
			Power=MotorMinimum;
		}

		if(-1*(SensorValue[Encoder_R]) > -1*(SensorValue[Encoder_L]))//Adjust motor power for encoder difference / Right > Left
		{
			Error_E=(-1*(SensorValue[Encoder_R])- -1*(SensorValue[Encoder_L]));
			motor[downleft] = Power;
			motor[yl] = Power;
			motor[downright] = Power-Error_E;
			motor[yr] = Power-Error_E;
		}

		else if(-1*(SensorValue[Encoder_R]) < -1*(SensorValue[Encoder_L]))//Adjust motor power for encoder difference / Left > Right
		{
			Error_E= (-1*(SensorValue[Encoder_L]) - -1*(SensorValue[Encoder_R]));
			motor[downleft] = Power-Error_E;
			motor[yl] = Power-Error_E;
			motor[downright] = Power;
			motor[yr] = Power;
		}
		else//Set power for motors w/o difference
		{
			motor[downleft] = Power;
			motor[yl] = Power;
			motor[downright] = Power;
			motor[yr] = Power;
		}
		wait1Msec(25);//Update power every 25ms.
	}
	motor[downleft] = 0;
	motor[yl] = 0;
	motor[downright] = 0;
	motor[yr] = 0;
}


void Base_Backward(float Distance_I, float Seconds)
{
	Distance_I *= 24.66; //This value is obtained from the formula: 360/(4.0*3.14) and then adding or subtracting from it.
	Distance_I = round(Distance_I);//This line makes sure that the value returned from Distance_I is an integer.
  Seconds*= (1000/15);//This converts how many seconds do you want the program to run.
	float Power, Error, Integral=0, Derivative, Prev_Error=0, Error_E;//Variables
	/*
	This part resets the encoders.
	*/
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	/*
	//This makes sure that the program only runs for the time you assign it to.
	*/
	for(int counter =0; counter<Seconds;	counter++)
	{
		Error=(Distance_I - SensorValue[Encoder_R]);//Calculates error to adjust the values of the PID.

		Integral=Integral+Error;//Calculates integral part of the PID.


		if(Error==0)//Adjust integral error
		{
			Integral=0;
		}
		Derivative=Error-Prev_Error;//Calculates derivative part of the PID.
		Prev_Error=Error;//Adjusts the D term based on the distance.
		Power=(Error*Kp)+(Integral*Ki)+(Derivative*Kd);//Set power correction for motors based on the PID

		/*
		This part limits the power assigned to the motors.
		*/
		if(Power>MotorMaximum)
		{
			Power=MotorMaximum;
		}
		if(Power< MotorMinimum)
		{
			Power=MotorMinimum;
		}

		if(SensorValue[Encoder_R] > SensorValue[Encoder_L])//Adjust motor power for encoder difference / Right > Left
		{
			Error_E=SensorValue[Encoder_R]-SensorValue[Encoder_L];
			motor[downleft] = -Power;
			motor[yl] = -Power;
			motor[downright] = -Power+Error_E;
			motor[yr] = -Power+Error_E;
		}

		else if(SensorValue[Encoder_R] < SensorValue[Encoder_L])//Adjust motor power for encoder difference / Left > Right
		{
			Error_E= SensorValue[Encoder_L] - SensorValue[Encoder_R];
			motor[downleft] = -Power+Error_E;
			motor[yl] = -Power+Error_E;
			motor[downright] = -Power;
			motor[yr] = -Power;
		}
		else//Set power for motors w/o difference
		{
			motor[downleft] = -Power;
			motor[yl] = -Power;
			motor[downright] = -Power;
			motor[yr] = -Power;
		}
		wait1Msec(25);//Update power every 25ms.
	}
	motor[downleft] = 0;
	motor[yl] = 0;
	motor[downright] = 0;
	motor[yr] = 0;
}


void Correct(float Degree, float Seconds)//**********************************************************************************************************************
{
 SensorValue[in8]=0;
 Degree= round(Degree*(-10));
 float Power, Integral, Derivative, Prev_Error;
 float Error;
  Seconds*= (1000/15);


 for(int counter =0; counter<Seconds;	counter++)															 //Turn Right
	{
		Error=(Degree - SensorValue[in8]);						           //Calculate error to adjust
		Integral=Integral+Error;	//Calculate integral part of correction
		datalogClear();


			if(Error==0)																						 //Adjust integral error
			{Integral=0;}

		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=-1*(Error*Gp)+Integral*Gi+-1*(Derivative*Gd);									 //Set power correction for motors

			if(Power>100)
			{
				Power=100;
				}

			motor[downleft] = Power;
			motor[yl] = Power;
			motor[downright] = -Power;
			motor[yr] = -Power;

			wait1Msec(15);
		}
	    motor[downleft] = 0;
			motor[yl] = 0;
			motor[downright] = 0;
			motor[yr] = 0;
}



void Turn_Right(float Degree, float Seconds)//**********************************************************************************************************************
{
 SensorValue[in8]=0;
 Degree= round(Degree*(-10));
 float Power, Integral, Derivative, Prev_Error;
 float Error;
  Seconds*= (1000/15);


 for(int counter =0; counter<Seconds;	counter++)															 //Turn Right
	{
		Error=(Degree - SensorValue[in8]);						           //Calculate error to adjust
		Integral=Integral+Error;	//Calculate integral part of correction
		datalogClear();


			if(Error==0)																						 //Adjust integral error
			{Integral=0;}

		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=-1*(Error*Gp)+Integral*Gi+-1*(Derivative*Gd);									 //Set power correction for motors

			if(Power>100)
			{
				Power=100;
				}

			motor[downleft] = Power;
			motor[yl] = Power;
			motor[downright] = -Power;
			motor[yr] = -Power;

			wait1Msec(15);
		}
	    motor[downleft] = 0;
			motor[yl] = 0;
			motor[downright] = 0;
			motor[yr] = 0;
}

void Turn_Left(float Degree, float Seconds, bool Option)
{
 /*
This part allows the user to decide whether to reset the gyroscope to 0 or not.
 */
 if(Option == 1)
 {
    SensorValue[in8]=0;
  }

  else
  {
  	SensorValue[in8]*=1;
  }
/*
This converts the grades that you want the robot turn into values that RobotC can assign
*/
 Degree= round(Degree*(10));
 float Power, Integral = 0, Derivative, Prev_Error = 0, Error;//Calculates error to adjust the values of the PID.
 Seconds*= (1000/15);//This converts how many seconds do you want the program to run.

 for(int counter =0; counter<Seconds;	counter++)
	{
		Error=(Degree - SensorValue[in8]);//Calculate error to PID.
		Integral=Integral+Error;//Calculate integral part of correction.

			if(Error==0)//Adjust integral error.
			{
			 Integral=0;
			}

		Derivative=Error-Prev_Error;//Calculate derivative part of correction.
		Prev_Error=Error;

		Power=(Error*Gp)+(Integral*Gi)+(Derivative*Gd);//Set power correction for motors based on the PID.

			if(Power>MotorMaximum)
			{
			 Power=MotorMaximum;
			}
			if(Power<MotorMinimum)
	  	{
			 Power= MotorMinimum;
		  }


			motor[downleft] = -Power;
			motor[yl] = -Power;
			motor[downright] = Power;
			motor[yr] = Power;

			wait1Msec(15);
		}
			    motor[downleft] = 0;
			motor[yl] = 0;
			motor[downright] = 0;
			motor[yr] = 0;
}
void Chain_Up(int Height, float Seconds)
{
	int Power, Error, Integral=0, Derivative, Prev_Error=0;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=  SensorValue[fourbarpot] - Height;//Calculate error to adjust
		Integral=Integral+Error;//Calculate integral part of correction
		Derivative=Error-Prev_Error;//Calculate derivative part of correction
		Prev_Error=Error;

		Power=(Error*Ap)+(Integral*Ai)+(Derivative*Ad);//Set power correction for motors based on the PID.

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
			}
				if(Power < MotorMinimum)
				{
					Power= MotorMinimum;
				}
		motor[fourbl]=Power;
		motor[fourbr]=Power;
		wait1Msec(15);
	}
		motor[fourbl]=0;
		motor[fourbr]=0;
}

task Chain_Position
{
int Power, Error, Integral=0, Derivative, Prev_Error=0;//, Integral, Derivative, Prev_Error;
  Error=  SensorValue[fourbarpot] - MaximumorMinimum;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;
		Power=(Error*Cp)+(Integral*Ci)+(Derivative*Cd);									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
			}
				if(Power < MotorMinimum)
				{
					Power= MotorMinimum;
				}


		motor[fourbl]=-Power;
		motor[fourbr]=-Power;

		wait1Msec(15);
	}

	task Chain_Auto
{
while(1)
{
	int Power, Error, Integral=0, Derivative, Prev_Error=0;//, Integral, Derivative, Prev_Error;
  Error=  SensorValue[fourbarpot] - MaximumorMinimum;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;
		Power=(Error*Cp)+(Integral*Ci)+(Derivative*Cd);									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
			}
				if(Power < MotorMinimum)
				{
					Power= MotorMinimum;
				}


		motor[fourbl]=-Power;
		motor[fourbr]=-Power;

		wait1Msec(15);
	}
	}


task Arm_Position
	{
		int Power, Error, Integral=0, Derivative, Prev_Error=0;
	Error=MaximumorMinimumArm - SensorValue[reversepot];          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{Power=MotorMinimum;
				}

		motor[arml]=Power;
		motor[armr]=Power;

		wait1Msec(15);

}

task Arm_Auto
{
	datalogClear();
while(1)
	{

		int Power, Error, Integral=0, Derivative, Prev_Error=0;
	Error=MaximumorMinimumArm - SensorValue[reversepot];          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{Power=MotorMinimum;
				}

		motor[arml]=Power;
		motor[armr]=Power;

		wait1Msec(15);
		datalogAddValue(1,Error);

}
}



void Arm_Up(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=Height - SensorValue[reversepot];          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
			}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
			}



		motor[arml]=Power;
		motor[armr]=Power;

		wait1Msec(25);
	}
		motor[arml]=0;
		motor[armr]=0;
}


void Mobile_Down(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=SensorValue[potmob] - Height;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Mp+Integral*Mi+Derivative*Md;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
				}

		motor[mob]=Power;


		wait1Msec(15);
	}
	motor[mob]=0;
}

task Mobile_Auto
{
	while(1)
	{

	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
  Error=SensorValue[potmob] - MaximumorMinimumMobile;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Mp+Integral*Mi+Derivative*Md;									 //Set power correction for motors


			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
				}

		motor[mob]=Power;


		wait1Msec(15);
	}
}
void Mobile_Up(int Height, float Seconds)//*******************************************************************************************************************
{
	Seconds*=(1000/15);
	int Power, Error, Integral, Derivative, Prev_Error;//, Integral, Derivative, Prev_Error;
	for(int counter =0; counter<Seconds;	counter++)
	{ Error=SensorValue[potmob] - Height;          															 //Calculate error to adjust
		Integral=Integral+Error;																	 //Calculate integral part of correction
		Derivative=Error-Prev_Error;															 //Calculate derivative part of correction
		Prev_Error=Error;

		Power=Error*Mp+Integral*Mi+Derivative*Md;									 //Set power correction for motors

			if(Power > MotorMaximum)
			{
				Power=MotorMaximum;
				}
			if(Power < MotorMinimum)
			{
				Power=MotorMinimum;
				}

		motor[mob]=Power;


		wait1Msec(15);
	}
	motor[mob]=0;
}

void left2cones20zone()
{
	MaximumorMinimumArm = 1800;
  startTask(Arm_Auto);
  MaximumorMinimumMobile = MobileMinimum;
  startTask(Mobile_Auto);
  wait1Msec(400);
  Base_Forward(52,1);
  stopTask(Mobile_Auto);
  MaximumorMinimum = 1970;
	startTask(Chain_Auto);
	wait1Msec(350);
	motor[rol] = -127;
	wait1Msec(200);
	motor[rol] = 0;
	MaximumorMinimumArm =1500;
	startTask(Arm_Auto);
	wait1Msec(200);
  Mobile_Up(4095,1);
  Correct(0,0.3);
	stopTask(Arm_Auto);
	stopTask(Chain_Auto);
	MaximumorMinimum = 2650;
	startTask(Chain_Auto);
	motor[rol] = 60;
	Base_Forward(10,0.3);
	wait1Msec(150);
	stopTask(Arm_Auto);
	MaximumorMinimumArm= 2700;
	startTask(Arm_Auto);
	wait1Msec(550);
	MaximumorMinimumArm = 1500;
	startTask(Arm_Auto);
	wait1Msec(400);
	stopTask(Chain_Auto);
	MaximumorMinimum = 1280;
	startTask(Chain_Auto);
  wait1Msec(700);
  stopTask(Arm_Auto);
  MaximumorMinimumArm = 2200;
  startTask(Arm_Auto);
  wait1Msec(300);
  motor[rol] = -100;
  wait1Msec(200);
  motor[rol] = 0;
  stopTask(Arm_Auto);
  stopTask(Chain_Auto);
  Turn_Right(1.5,0.2);
  Base_Backward(55,1.3);
  Turn_Left(40,0.5,1);
  Base_Backward(26,0.5);
  Turn_Left(105,1,1);
  wait1Msec(200);
   motor[rol] = -100;
  MaximumorMinimumArm = 1600;
  startTask(Arm_Auto);
  MaximumorMinimumMobile = MobileMinimum + 850;
  startTask(Mobile_Auto);
  Base_Forward(24,1);
  Base_Forward(8.5,0.3);
  stopTask(Mobile_Auto);
  MaximumorMinimumMobile = MobileMinimum;
  startTask(Mobile_Auto);
  Base_Backward(15,1);
}

void conestationarymobile1cone5ptzoneparking()
{
	MaximumorMinimumArm = 800;
	startTask(Arm_Auto);
	Base_Forward(19.5,0.7);
	MaximumorMinimum =2300;
	startTask(Chain_Auto);
	wait1Msec(300);
	motor[rol] = -100;
	wait1Msec(200);
	MaximumorMinimum = 1500;
	startTask(Chain_Auto);
	MaximumorMinimumArm= 300;
	startTask(Arm_Auto);
	Base_Backward(20,1);
	Turn_Right(98,0.8);
	MaximumorMinimumMobile = MobileMinimum;
  startTask(Mobile_Auto);
  wait1Msec(400);
  Base_Forward(55,0.8);
  motor[rol] = 0;


  MaximumorMinimumArm = 300;
 startTask(Arm_Auto);
   stopTask(Mobile_Auto);
	wait1Msec(200);
  Mobile_Up(4095,1);
  Turn_Left(5,0.3,1);
	stopTask(Arm_Auto);
	stopTask(Chain_Auto);
	MaximumorMinimum = 2750;
	startTask(Chain_Auto);
	motor[rol] = 60;
	Base_Forward(8,0.3);
	wait1Msec(150);
	stopTask(Arm_Auto);
	MaximumorMinimumArm= 0;
	startTask(Arm_Auto);
	wait1Msec(400);
	MaximumorMinimumArm = 300;
	startTask(Arm_Auto);
	wait1Msec(400);
	stopTask(Chain_Auto);
	MaximumorMinimum = 1280;
	startTask(Chain_Auto);
  wait1Msec(700);
  stopTask(Arm_Auto);
  Base_Backward(60,1);
  Turn_Right(180,0.8*(2));
  motor[rol] = -120;
MaximumorMinimumMobile = MobileMinimum;
startTask(Mobile_Auto);
wait1Msec(400);
Base_Backward(60,1.5);
}

void mobilepreloadplus3loads20ptzone()
{
		MaximumorMinimumArm = 200;
  startTask(Arm_Auto);
  MaximumorMinimumMobile = MobileMinimum;
  startTask(Mobile_Auto);
  wait1Msec(400);
  Base_Forward(52,1);
  stopTask(Mobile_Auto);
  MaximumorMinimum = 1970;
	startTask(Chain_Auto);
	wait1Msec(350);
	motor[rol] = -127;
	wait1Msec(200);
	motor[rol] = 0;
	MaximumorMinimumArm =300;
	startTask(Arm_Auto);
	wait1Msec(200);
  MaximumorMinimumMobile = MobileMaximum;
  startTask(Mobile_Auto);
  wait1Msec(200);
  Base_Backward(19,0.9);
  MaximumorMinimumArm = 500;
  Turn_Right(91,1);
  motor[rol] = 60;
  MaximumorMinimumArm = 450;
  MaximumorMinimum = 2450;
  wait1Msec(400);
  MaximumorMinimum = 1100;
  wait1Msec(500);
  MaximumorMinimumArm = 100;
  wait1Msec(200);
  motor[rol] = -100;
  wait1Msec(300);
  motor[rol] = 60;
   MaximumorMinimumArm = 450;
   wait1Msec(250);
   MaximumorMinimum = 2450;
  wait1Msec(400);
   MaximumorMinimum = 1100;
  wait1Msec(950);
  motor[rol] = -100;
  wait1Msec(200);
  motor[rol] = 60;
   MaximumorMinimum = 2450;
  wait1Msec(500);
   MaximumorMinimum = 1100;
  Turn_Right(110,1);
  MaximumorMinimumArm = 100;
  Base_Forward(55,1.3);
  Turn_Left(70,0.8,1);
  Base_Forward(30,0.6);
  MaximumorMinimumMobile = MobileMinimum;
  MaximumorMinimumArm = 400;
  motor[rol] = -60;
  startTask(Mobile_Auto);
  startTask(Arm_Auto);
  wait1Msec(300);
  Base_Forward(30,0.6);
  stopTask(Mobile_Auto);
  MaximumorMinimumMobile = MobileMaximum;
  startTask(Mobile_Auto);
  Base_Backward(15,1);
}

void pre_auton()
{
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	 SensorType[in8] = sensorNone;
	 wait1Msec(2000);
  //Reconfigure Analog Port 4 as a Gyro sensor and allow time for ROBOTC to calibrate it
  SensorType[in8] = sensorGyro;
 wait1Msec(1000);
}
task autonomous()
{
}
task usercontrol()
{
  while (true)
  {
  		motor[downleft] = vexRT[Ch3] + vexRT[Ch4];
		motor[downright] = vexRT[Ch3] - vexRT[Ch4];
		motor[yr] = vexRT[Ch3] - vexRT[Ch4];
		motor[yl] = vexRT[Ch3] + vexRT[Ch4];


		if (vexRT[Btn8U] == 1)
		{
			motor[mob] = 127;
		}

		else if (vexRT[Btn8D] == 1)
		{
			motor[mob] = -127;
		}

		else if(vexRT[Btn8U] == 0 && vexRT[Btn8D] == 0)
		{
			motor[mob] = 0;
		}



		if (vexRT[Btn6U] == 1)
		{
			motor[rol] = 70;
		}

		else if (vexRT[Btn6D] == 1)
		{
			motor[rol] = -127;
		}

		else if(vexRT[Btn6U] == 0 && vexRT[Btn6D] == 0)
		{
			motor[rol] = 0;
		}

		motor[arml] = vexRT[Ch2];
		motor[armr] = vexRT[Ch2];
		if (vexRT[Btn5U] == 1)
		{
			MaximumorMinimum = 1120;
			startTask(Chain_Position);
		}

		else if (vexRT[Btn5D] == 1)
		{
			MaximumorMinimum = 2650;
			startTask(Chain_Position);
		}

		else if(vexRT[Btn5U] == 0 && vexRT[Btn5D] == 0)
		{
			motor[fourbl] = 0;
			motor[fourbr] = 0;
		}

  }
}
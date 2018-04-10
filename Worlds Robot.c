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
#pragma config(Motor,  port7,           fourbl,        tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           fourbr,        tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           armr,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          downright,     tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#define MotorMaximum       127
#define MotorMinimum     (-127)

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


float Kp=0.4;
float Ki=0;
float Kd=7;
float Gp=1.2;
float Gi=0;
float Gd=2.2;
float Ap=0.5;
float Ai=0;
float Ad=2;

////////////////////////////////////////*Functions for autonomous and driver control*////////////////////////////////////////////////////

void Base_Forward(float Distance_I, float Seconds)
{
	Distance_I *= 28.66; //This value is obtained from the formula: 360/(4.0*3.14)
	Distance_I = round(Distance_I);//This line makes sure that the value returned from Distance_I is an integer.
  Seconds*= (1000/15);//This converts how many seconds do you want the program to run.
	float Power, Error, Integral=0, Derivative, Prev_Error=0, Error_E;//Variables
	/*
	This part resets the encoders and makes them run in positive numbers
	*/
	SensorValue[Encoder_R]=0;
	SensorValue[Encoder_L]=0;
	SensorValue[Encoder_L]*= -1;
	SensorValue[Encoder_R]*= -1;
	/*
	//This makes sure that the program only run for the time you assign it to.
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
		Power=(Error*Kp)+(Integral*Ki)+(Derivative*Kd);//Set power correction for based on the PID

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
			Error_E=(SensorValue[Encoder_R]- SensorValue[Encoder_L]);
			motor[downleft] = Power;
			motor[yl] = Power;
			motor[downright] = Power-Error_E;
			motor[yr] = Power-Error_E;
		}

		else if(SensorValue[Encoder_R] < SensorValue[Encoder_L])//Adjust motor power for encoder difference / Left > Right
		{ Error_E= (SensorValue[Encoder_L] - SensorValue[Encoder_R]);
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
		wait1Msec(25);
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

}

void Turn_Left(float Degree, float Seconds, bool Option)
{
 datalogClear();
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

		Power=(Error*Gp)+Integral*Gi+Derivative*Gd;//Set power correction for motors.

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

			if(Power > 127)
			{Power=127;}

		motor[arml]=Power;
		motor[armr]=Power;

		wait1Msec(15);
	}
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

		Power=Error*Ap+Integral*Ai+Derivative*Ad;									 //Set power correction for motors

			if(Power > 127)
			{Power=127;}

		motor[mob]=Power;


		wait1Msec(15);
	}
}

void pre_auton()
{
}
task autonomous()
{
	Base_Forward(12,2);
}
task usercontrol()
{
  while (true)
  {
  }
}

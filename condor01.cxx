#include </lib/drone/MPU9250/MPU9250.h>
//#include </lib/drone/MPU9250/MPU9250_2.h>
#include "/lib/drone/MPU9250/BMP280.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
//#include </lib/drone/PWM/SoftPWM.h>
//#include </lib/drone/PWM/>
#include <softPwm.h>
#include <math.h>
#include <iostream>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <signal.h>
#include <pigpio.h>

using namespace std;

//installieren
int PWM_pin1 = 17;  //11
int PWM_pin2 = 27; //13 
int PWM_pin3 = 22; //15  
int PWM_pin4 = 23; //16

//Ersteinrichtung
float max_value=19,inital_value=10,duty_cycle=20;

//schweben
float CW1,CW2,CCW3,CCW4,CW=11,CCW=11;
float accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ;
float rad_to_deg = 180.0/3.1415;
float roll_accel,pitch_accel,yaw_gyro[2],totalmag;

//aktuelle Werte
float pv_PID,sp_PID,i_PID_buffer,hohe,pressure;
float error_accel,error_gyro,error_mag,error_PID;

//proportional, ganzzahlig, ableitend
float p_PID=0.07,i_PID=0.002,d_PID=0.0003;

//Zähler
int timer=0,timerprev,elapsedtime;

//Gyroskop Erstellen
int16_t ACCxyz[3];
int16_t GYRxyz[3];
int16_t MAGxyz[3];

float MagDes[3];
float CalDes1[3];
float CalDes2[3];
float SelDes[6];

void Gyroskop_read(void);
void Gyroskop_installieren(void);
void ErstellenPWM(void);
void PIDSteuerung(void);

int main(int argc, char **argv)
{
	//wiringPiSetupGpio();
	gpioInitialise();
	
	ErstellenPWM();
	Gyroskop_installieren();

	timer=0;
	
	while(1)
	{
		Gyroskop_read();
		PIDSteuerung();
		
		timer=timer+1; 
		elapsedtime=elapsedtime+timer;
		
		cout << "CW " << CW << "\n";
		cout << "CCW " << CCW << "\n";
		//cout << "CW1 " << CW1 << "\n";
		//cout << "CW2 " << CW2 << "\n";
		//cout << "CCW3 " << CCW3 << "\n";
		//cout << "CCW4 " << CCW4 << "\n \n";
		
		cout << "timer " << timer<< "\n \n";
		
		//softPwmWrite(PWM_pin1,CW);
		//softPwmWrite(PWM_pin2,CW);
		//softPwmWrite(PWM_pin3,CCW);
		//softPwmWrite(PWM_pin4,CCW);
		gpioWrite(PWM_pin1,1088);
		
		cout << "duty_cycle " << duty_cycle<< "\n \n";

		delay(duty_cycle);
	}
	
	return 0;
}
void PIDSteuerung()
{
	CW=CW;
	CCW=CCW;
	CW1=CW1+0.07,
	CW2=CW2+0.07;
	CCW3=CCW3+0.07,
	CCW4=CCW4+0.07;

}
void Gyroskop_read()
{
		readAccelData(ACCxyz);
	    readGyroData(GYRxyz);
		readMagData(MAGxyz);
		bmp280_read();
		
		accelX=ACCxyz[0]*getAres();
		accelY=ACCxyz[1]*getAres();
		accelZ=ACCxyz[2]*getAres();
		
		gyroX=GYRxyz[0]*getGres();
		gyroY=GYRxyz[1]*getGres();
		gyroZ=GYRxyz[2]*getGres();
		
		magX=MAGxyz[0]*getMres();
		magY=MAGxyz[1]*getMres();
		magZ=MAGxyz[2]*getMres();
		
		hohe=bmp.altitude;
		pressure=bmp.pressure;
		
		printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n",accelX,accelY,accelZ);
		pitch_accel = atan((accelY/16384.0)/sqrt(pow((accelX/16384.0),2) + pow((accelZ/16384.0),2)))*rad_to_deg;
        roll_accel = atan(-1*(accelX/16384.0)/sqrt(pow((accelY/16384.0),2) + pow((accelZ/16384.0),2)))*rad_to_deg;
        printf("ACC:  \tPitch %5.4f  \tRoll: %5.4f  \r\n\n",pitch_accel,roll_accel);
	
        printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n",gyroX,gyroY,gyroZ);
        yaw_gyro[0]=0;
        yaw_gyro[1]=1;
        yaw_gyro[2]=(250-gyroZ)/131;
		printf("GYRO: \tZ: %7.4f  \tZ: %7.4f  \tZ: %7.4f\r\n\n",yaw_gyro[0],yaw_gyro[1],yaw_gyro[2]);
	
        printf("MAG:  \tX: %8.3f  \tY: %8.3f  \tZ: %8.3f\r\n",magX,magY,magZ);
        totalmag = (atan2(magY, magX)) * rad_to_deg;
        printf("MAG:  \tmag: %8.3f \r\n",totalmag);
        
        printf("Temp: \t%3.1f°C\r\n\r\n",readTempInC());

        printf("Temp:\t\t%2.2f °C\r\n", bmp.temperature);
		printf("Pressure:\t%5.4f mbar\r\n", pressure);
		printf("Altitude:\t%5.3f m\r\n\r\n", hohe);
   
}
void Gyroskop_installieren()
{
	BMP280();
	BMP280_read_id();
    BMP280_reg_check();
	
    MPU9250();
    initMPU9250();
    initAK8963(MagDes);
    
    //calibrateMPU9250(CalDes1,CalDes2);
    
    MPU9250SelfTest(SelDes);
}
void ErstellenPWM(void)
{
	//gpioSetPWMrange(PWM_pin1,500);
	
	gpioSetPWMfrequency(PWM_pin1,2000);
	//pinMode(PWM_pin1,OUTPUT);
	//pinMode(PWM_pin2,OUTPUT);
	//pinMode(PWM_pin3,OUTPUT);
	//pinMode(PWM_pin4,OUTPUT);
	
	//softPwmCreate(PWM_pin1,inital_value,max_value);
	//softPwmCreate(PWM_pin2,inital_value,max_value);
	//softPwmCreate(PWM_pin3,inital_value,max_value);
	//softPwmCreate(PWM_pin4,inital_value,max_value);
	gpioWrite(PWM_pin1,1064);
	//while(1)
	//{
		
	//softPwmWrite(PWM_pin1,inital_value);
	//softPwmWrite(PWM_pin2,inital_value);
	//softPwmWrite(PWM_pin3,inital_value);
	//softPwmWrite(PWM_pin4,inital_value);
	
	//timer =timer+1;
	//delay(duty_cycle);
	
	//cout << "timer " << timer<< "\n \n";
	
	//if(timer==10)
	//return;
	//}
	
	
	
}

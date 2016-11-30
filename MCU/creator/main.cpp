/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator firmware for MCU
 * Author: Andrés Calderón [andres.calderon@admobilize.com]
 *
 * MATRIX Creator firmware for MCU is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"
#include "board.h"

#include <math.h>
#include <string.h>
#include <mcuconf.h>

#include "./i2c.h"
#include "./sensors_data.h"
#include "./mpl3115a2.h"
#include "./lsm9ds1.h"
#include "./hts221.h"
#include "./veml6070.h"

#include "chprintf.h"

extern "C" {
#include "atmel_psram.h"
}

const uint32_t kFirmwareCreatorID = 0x10;
const uint32_t kFirmwareVersion = 0xB9B9B9B9; /* 0xYYMMDD */

char *ptr = (char *) PSRAM_BASE_ADDRESS;

/* Global objects */
creator::I2C i2c;  

void psram_copy(uint32_t mem_offset, char *data, uint8_t len) {
  register char *psram = (char *)PSRAM_BASE_ADDRESS;

  for (int i = 0; i < len; i++) {
    psram[mem_offset + i] = data[i];
  }
}
//------------------------------------PID-------------------------------------
int32_t maxPwm=0x061A80; //Pwm Máx 2ms
int32_t minPwm=0x035B60;

//PID Variables
float pid_p_gain_roll=0.8;     //Gain setting for the roll P-controller
float pid_i_gain_roll=0.04;   //Gain setting for the roll I-controller
float pid_d_gain_roll=28;     //Gain setting for the roll D-controller

int pid_max_roll = 400;   //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_roll_setpoint  =-1;
float pid_pitch_setpoint =3;

IMUData dataimu;

float pid_output_roll=0;
float pid_output_pitch=0;
float pid_i_mem_roll=0;
float pid_i_mem_pitch=0;
float pid_last_roll_d_error=0;
float pid_last_pitch_d_error =0;


void calculate_pid(){
  //Roll calculations
  float pid_error_temp;
  pid_error_temp = dataimu.roll - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = dataimu.pitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;
}

char getChar(uint32_t data, char select)
{
	char dataToReturn=0;
	
	switch (select)
	{
	case 1:
	 dataToReturn=(data & 0xFF0000)>>16;
	 
	break;
	case 2:
	 dataToReturn=(data & 0x00FF00)>>8;
	break;
	case 3:
	 dataToReturn=(data & 0x0000FF);
	break;
	default:
	break;
   }

return dataToReturn; 
}

//--------------------------------PWM-------------------------------------------------------
void set_period (PWMData *data,char d1, char d2, char d3){ 
	data->period_1=d1;	
	data->period_2=d2;
	data->period_3=d3;   
	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
}
void set_duty (PWMData *data,char motor,char d1, char d2, char d3){
	switch(motor){
	case 1:
	data->duty1_1   = d1;
   	data->duty1_2   = d2;
   	data->duty1_3   = d3;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	case 2:
	data->duty2_1   = d1;
        data->duty2_2   = d2;
	data->duty2_3   = d3;
	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
	break;
	case 3:
	data->duty3_1   = d1;
   	data->duty3_2   = d2;
   	data->duty3_3   = d3;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	case 4:
   	data->duty4_1   = d1;
        data->duty4_2   = d2;
        data->duty4_3   = d3;
        psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
   	break;
   	default:
   	data->duty1_1   = 0;
   	data->duty1_2   = 0;
   	data->duty1_3   = 0;
   	psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
	}
  }


void initMotors(PWMData *data)
{
    set_period(data,0x3D,0X09,0X00);
    set_duty (data,1,0x03,0X0D,0X40);
    set_duty (data,2,0x03,0X0D,0X40);
    set_duty (data,3,0x03,0X0D,0X40);
    set_duty (data,4,0x03,0X0D,0X40);
    chThdSleepMilliseconds(3000);    	
}
//Sequential increase of the cycle in 20 steps from 1ms to 2ms
//Manual mode
void plusDuty(PWMData *data,int motor){
int a =ptr[0xF1];
switch(a){
	case 0:  set_duty (data,motor,0x03,0X0D,0X40); break;
	case 1:  set_duty (data,motor,0x03,0X34,0X50); break;
	case 2:  set_duty (data,motor,0x03,0X5B,0X60); break;
	case 3:  set_duty (data,motor,0x03,0X82,0X70); break;
	case 4:  set_duty (data,motor,0x03,0XA9,0X80); break;
	case 5:  set_duty (data,motor,0x03,0XD0,0X90); break;
	case 6:  set_duty (data,motor,0x03,0XF7,0XA0); break;
	case 7:  set_duty (data,motor,0x04,0x1E,0xB0); break;
	case 8:  set_duty (data,motor,0x04,0X45,0XC0); break;
	case 9:  set_duty (data,motor,0x04,0X6C,0XD0); break;
	case 10: set_duty (data,motor,0x04,0X93,0XE0); break;
	case 11: set_duty (data,motor,0x04,0XBA,0XF0); break;
	case 12: set_duty (data,motor,0x04,0XE2,0X00); break;
	case 13: set_duty (data,motor,0x05,0X09,0X10); break;
	case 14: set_duty (data,motor,0x05,0X30,0X20); break;
	case 15: set_duty (data,motor,0x05,0X57,0X30); break;
	case 16: set_duty (data,motor,0x05,0X7E,0X40); break;
	case 17: set_duty (data,motor,0x05,0XA5,0X50); break;
	case 18: set_duty (data,motor,0x05,0XCC,0X60); break;
	case 19: set_duty (data,motor,0x05,0XF3,0X70); break;
	case 20: set_duty (data,motor,0x06,0X1A,0X80); break;
	default: set_duty (data,motor,0x03,0X0D,0X40);
	}
}
//PID mode
int32_t plusDuty2(){
int a =ptr[0xF1];
switch(a){
	case 0:  return 0x030D40; break;
	case 1:  return 0x033450; break;
	case 2:  return 0x035B60; break;
	case 3:  return 0x038270; break;
	case 4:  return 0x03A980; break;
	case 5:  return 0x03D090; break;
	case 6:  return 0x03F7A0; break;
	case 7:  return 0x041EB0; break;
	case 8:  return 0x0445C0; break;
	case 9:  return 0x046CD0; break;
	case 10: return 0x0493E0; break;
	case 11: return 0x04BAF0; break;
	case 12: return 0x04E200; break;
	case 13: return 0x050910; break;
	case 14: return 0x053020; break;
	case 15: return 0x055730; break;
	case 16: return 0x057E40; break;
	case 17: return 0x05A550; break;
	case 18: return 0x05CC60; break;
	case 19: return 0x05F370; break;
	case 20: return 0x061A80; break;
	default: return 0x030D40; break;
	}
}
//----THREAD THAT INTEGRATES IMU, PWM AND PID---------------------
static WORKING_AREA(waIMUThread, 512);
static msg_t IMUThread(void *arg) {
  (void)arg;
  LSM9DS1 imu(&i2c, IMU_MODE_I2C, 0x6A, 0x1C);
  
  IMUData dataimu;
  PWMData data;
  //Init Imu
  imu.begin();
  //Init Motors  
  initMotors(&data);  
 
  while (true) {
    imu.readGyro();
    dataimu.gyro_x = imu.calcGyro(imu.gx);
    dataimu.gyro_y = imu.calcGyro(imu.gy);
    dataimu.gyro_z = imu.calcGyro(imu.gz);

    imu.readMag();
    dataimu.mag_x = imu.calcMag(imu.mx);
    dataimu.mag_y = imu.calcMag(imu.my);
    dataimu.mag_z = imu.calcMag(imu.mz);

    imu.readAccel();
    dataimu.accel_x = imu.calcAccel(imu.ax);
    dataimu.accel_y = imu.calcAccel(imu.ay);
    dataimu.accel_z = imu.calcAccel(imu.az);

    dataimu.yaw = atan2(dataimu.mag_y, -dataimu.mag_x) * 180.0 / M_PI;
    dataimu.roll = atan2(dataimu.accel_y, dataimu.accel_z) * 180.0 / M_PI;
    dataimu.pitch = atan2(-dataimu.accel_x, sqrt(dataimu.accel_y * dataimu.accel_y +
                                           dataimu.accel_z * dataimu.accel_z)) *
                 180.0 / M_PI;

    psram_copy(mem_offset_imu, (char *)&dataimu, sizeof(dataimu));
    
   //Automatic Mode
   if(ptr[0xF0]==0x01){
   		//PID calculations
		calculate_pid();
		palSetPad(IOPORT3, 17);
		pid_output_roll *=200;
		pid_output_pitch*=200;

		int32_t writeRoll  = (int32_t) pid_output_roll;
		int32_t writePitch = (int32_t) pid_output_pitch;
				
		int32_t PWM_base=plusDuty2();

		int32_t esc_1 = PWM_base - writePitch + writeRoll; //Calculate the pulse for esc 1 (front-right - CCW)
		int32_t esc_2 = PWM_base + writePitch - writeRoll; //Calculate the pulse for esc 3 (rear-left - CCW)
		int32_t esc_3 = PWM_base + writePitch + writeRoll; //Calculate the pulse for esc 2 (rear-right - CW)
		int32_t esc_4 = PWM_base - writePitch - writeRoll; //Calculate the pulse for esc 4 (front-left - CW)

		if     (esc_1 < minPwm) esc_1 = minPwm;
		if     (esc_1 > maxPwm) esc_1 = maxPwm;
		if     (esc_2 < minPwm) esc_2 = minPwm;
		if     (esc_2 > maxPwm) esc_2 = maxPwm;
		if     (esc_3 < minPwm) esc_3 = minPwm;
		if     (esc_3 > maxPwm) esc_3 = maxPwm;
		if     (esc_4 < minPwm) esc_4 = minPwm;
		if     (esc_4 > maxPwm) esc_4 = maxPwm; 


		set_duty(&data,1,getChar(esc_1,1),getChar(esc_1,2),getChar(esc_1,3));
		set_duty(&data,2,getChar(esc_2,1),getChar(esc_2,2),getChar(esc_2,3));
		set_duty(&data,3,getChar(esc_3,1),getChar(esc_3,2),getChar(esc_3,3)); 
		set_duty(&data,4,getChar(esc_4,1),getChar(esc_4,2),getChar(esc_4,3));    
    }
    //Manual Mode
    else if(ptr[0XF0]==0X02){ 
      palClearPad(IOPORT3, 17);
      plusDuty(&data,1);
      plusDuty(&data,2);
      plusDuty(&data,3);
      plusDuty(&data,4);
    }
    chThdSleepMilliseconds(20);
  }
  return (0);
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();

  chSysInit();

  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  i2c.Init();
  
  /* Creates the imu thread. */
  chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread,
                    NULL);

   sdStart(&SD1,NULL);

}

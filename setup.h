/* 
 * File:   setup.h
 * Author: Duy Ngo
 *
 * Created on November 12, 2019, 10:51 AM
 */

#ifndef SETUP_H
#define	SETUP_H

void setup_ports(void);
void setup_motor_pwms(void);
void setup_adc(void);
void setup_qei(void);
void setup_timers(void);
unsigned int ReadConfig(int);
void WriteConfig(int,int);

#endif	/* SETUP_H */


/*
 * DPU_3200_48.h
 *
 *  Created on: Jun 2, 2022
 *      Author: Lenovo
 */

#ifndef INC_DPU_3200_48_H_
#define INC_DPU_3200_48_H_



#include "stdint.h"
//#define DPU_Address 0x8E
//extern uint8_t DPU_REC[2];
extern uint16_t DPU_Vol_Count;
extern uint16_t SetVolt; // 48mV
extern uint16_t DpuVol;
extern uint16_t FactorDpu;
extern uint16_t duty;


extern void delay(unsigned int cycles);
extern void long_delay(unsigned int cycle);
extern void setdpuvoltage(void);
extern void ControlVoltage(void);
extern void DPU_Average(void);




//#define ON_OFF_CONFIG                      0x01
//#define VOUT_MODE                          0x20
//#define VOUT_COMMAND_R                     0x21
//#define VOUT_TRIM                          0x22
//#define IOUT_OC_FAULT_LIMIT                0x46
//#define IOUT_OC_FAULT_RESPONSE             0x47
//#define STATUS_WORD                        0x79
//#define STATUS_VOUT                        0x7A
//#define STATUS_IOUT                        0x7B
//#define STATUS_TEMPERATURE                 0x7D
//#define READ_VIN                           0x88
//#define READ_VOUT                          0x8B
//#define READ_IOUT                          0x8C
//#define CURVE_CONFIG                       0xB4
//#define CHG_STATUS                         0xB8
//
//
//
//typedef enum  {
//
//	Default=0x00,
//	Gel_Battery=0x01,
//	Flooded_Battery=0x02,
//	AGM_Battery=0x03,
//
//
//
//}Choose_Curve;
//
//
//typedef enum{
//
//	Disable=0x00,
//	Neg3mV=0x01,
//	Neg4mV=0x02,
//	Neg5mV=0x03
//
//
//}Temp_comp_sett;
//
//typedef enum{
//	Not_Fully_Charged,
//	Fully_Charged
//
//}Charging_Staus;






#endif /* INC_DPU_3200_48_H_ */

/*
 * DPU_3200_48.c
 *
 *  Created on: Jun 2, 2022
 *      Author: Lenovo
 */

#include"DPU_3200_48.h"
#include"main.h"

 uint16_t SetVolt=24500; //
uint16_t DpuVol=0;
uint16_t FactorDpu=0;
uint16_t PreVoltage=24500;
uint16_t DPU_Vol_Count=0;
uint16_t duty=2;
_Bool NextFlag=0;



void DPU_Average(void)
{
 	uint32_t AverageBufferVol=0;
 	AverageBufferVol=DPU_Vol_Count;
	for(int i=0; i<50; i++)
	{
		AverageBufferVol+=ADC_Count_DPU[0];
		delay(300);
	}
	DPU_Vol_Count=AverageBufferVol/50;
}



void setdpuvoltage(void)
{

	 DPU_Average();
	 DpuVol=(FactorDpu*ADC_Count_DPU[0])/1000;
	 if(SetVolt < 23000) SetVolt=23000;
	 if(SetVolt > 56000) SetVolt=56000;
	 if(SetVolt > PreVoltage)
		 {
			 PreVoltage=SetVolt;
			 for(int i=0; i<480; i++)
			 {
 				 duty=duty+1;
				 FactorDpu=FactorDpu-1;
				 TIM3->CCR4=duty;
				 DPU_Average();
				 long_delay(15);
				 DpuVol=(FactorDpu*ADC_Count_DPU[0]/1000);
	 			 if(SetVolt <= DpuVol)
	 				 {
	 				 break;
	 				 }

			 }
		 }
		 else if(SetVolt < PreVoltage)
		 {
			 PreVoltage=SetVolt;
			 for(int i=0; i<480; i++)
			 {
 				 duty=duty-1;
				 TIM3->CCR4=duty;
				 FactorDpu=FactorDpu-1;
				 DPU_Average();
				 long_delay(15);
				 DpuVol=(FactorDpu*ADC_Count_DPU[0]/1000);
	 			 if(SetVolt >= (DpuVol-200))
	 				 {
	 				 break;
	 				 }

			 }
		 }

}




void ControlVoltage(void)
{
	 DPU_Average();
	 if(DpuVol < 18000)
	 {
		  FactorDpu=17920;
		  duty=2;
		  NextFlag=1;

	 }
	 else if(DpuVol > 18000 && NextFlag==1)
	 {
		 NextFlag=0;
		 long_delay(100);

	 }
	 DpuVol=(FactorDpu*DPU_Vol_Count)/1000;
	 if(SetVolt < 23000) SetVolt=23000;
	 if(SetVolt > 56000) SetVolt=56000;




	if((SetVolt-200) < DpuVol && DpuVol < SetVolt-200)
	{

	}
	else if(SetVolt+200 < DpuVol)
	{
		 //for(int i=0; i<480; i++)
		 {
			 //if(duty==1) break;
//			 if(SetVolt >= (DpuVol-200))
//				 {
//				 break;
//				 }
			 duty=duty-1;
			 if(DpuVol < 26000) FactorDpu=17930;
 			 else if(DpuVol < 42000) FactorDpu=FactorDpu+14;
			 else FactorDpu=17216;
			 if(duty == 0)
			 {
				 duty=1;

			 }
			 TIM3->CCR4=duty;
			 DPU_Average();
			 long_delay(20);
			 DpuVol=(FactorDpu*DPU_Vol_Count/1000);


		 }
	}
	else if(DpuVol < SetVolt-200)
	{
 		 {

			 duty=duty+1;
			 if(DpuVol < 26000) FactorDpu=17930;
 			 if(DpuVol < 42000) FactorDpu=FactorDpu-14;
			 else FactorDpu=17216;

 			 if(duty > 480)
			 {
				 duty=1;

			 }
			 TIM3->CCR4=duty;
			 DPU_Average();
			 long_delay(20);
			 DpuVol=(FactorDpu*DPU_Vol_Count/1000);


		 }
	}

}
































void delay(unsigned int cycles)
{
   while(cycles!=0)
   cycles--;
}

void long_delay(unsigned int cycle)
{

	while(cycle>0)
	{
		delay(65500);
		cycle--;
	}
}

// HAL_StatusTypeDef return_value;
// uint8_t data = CHG_STATUS;
// if (HAL_SMBUS_IsDeviceReady(&hsmbus2, (uint16_t) DPU_Address, 2, 2)!= HAL_OK)
// {
//         Error_Handler();//passed
// }
//return_value = HAL_SMBUS_Master_Transmit_IT(&hsmbus1,(uint16_t)DPU_Address,&data,1,SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);
//while(HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY);
//if (return_value != HAL_OK)
//{
//	return return_value;
//}
//
//return_value = HAL_SMBUS_Master_Receive_IT(&hsmbus1,(uint16_t)DPU_Address,&DPU_REC[0],2,SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);
//if (return_value != HAL_OK)
//{
//	return return_value;
//}

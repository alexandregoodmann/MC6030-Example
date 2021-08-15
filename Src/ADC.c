
#include "main.h"
#include "stm32f4xx_hal_adc.h"


uint32_t ADC_Value[40] = {0};
uint8_t i;
uint32_t ad1,ad2,ad3,ad4;
int16_t Bus_current=0;
extern uint32_t Speed;
extern uint32_t Angle;
extern uint8_t Control_Mode;

void Adc_Convert()
{
 
  for(i = 0, ad1 =0, ad2=0, ad3=0, ad4=0; i < 40;)
  {
    ad1 += ADC_Value[i++];
    ad2 += ADC_Value[i++];
    ad3 += ADC_Value[i++];
    ad4 += ADC_Value[i++];
  }
    ad1 /= 10;
    ad2 /= 10;
    ad3 /= 10;
    ad4 /= 10;
    //printf("\r\n******** ADC DMA Example ********\r\n\r\n");
    //printf(" AD1 value = %1.3fV \r\n", ad1*3.3f/4096);
    //printf(" ad1 value = %d \r\n", ad1);
    
    printf(" Bus voltage = %1.3fV \r\n", ad2*3.3f/4096*21);
	printf(" AD0 value = %1.3fV \r\n", ad4*3.3f/4096);
    
    //printf(" ad3 value = %d \r\n", ad3);
    
    Bus_current = (ad3-2035)*30000/2035;// 0-1.65 <0 ,1.65-3.3 >0  offset 2035
    printf(" Bus current = %dmA \r\n", Bus_current);
    //printf(" ad3 value = %1.3fV \r\n", ad3*3.3f/4096);
    
}

void potentiometer_Control()
{   
  uint8_t id;
  
  Mode_Select();
  Adc_Convert();
   
  if((ad4>=10) && (ad4<=4095))     //deadband: 10
  {
  
    switch(Control_Mode)
    {
       case 0:
        
            printf(" Speed Control Mode:");
            Speed = ((ad4 - 10 )*50 + 1000);   //speed = (voltage -100 )/(4096-100) * (200000 -1500) + 1500 dps*100 , min:15dps, max: 200dps 
            printf(" Speed = %d \r\n", Speed);             
            for (id=1;id<4;id++)
            {
             RS_speedControl(id, Speed);
             speedControl(id, Speed);
             HAL_Delay(3);            
            }
            
            break;
            
      case 1:     
    
            printf(" Angle Control Mode:");
            Angle = ((ad4 - 10 )*36000/(4096-10));
            //printf(" Angle = %#X \r\n", Angle);
           
            for (id=1;id<4;id++)
            {           
             RS_angleControl_2(id,Angle,50000);
             Multi_angleControl_2(id,Angle,50000);
             HAL_Delay(3);
            }

           break;
    }
      
  }
  
  else
  {
    Speed = 0 ;
    for(id=1;id<4;id++)
    {
      RS_Motor_Off(id);
      Motor_Off(id);
      HAL_Delay(2);
    }   
  }  
}
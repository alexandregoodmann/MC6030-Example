

#include "main.h"
#include "RC_pwm.h"


#define a 0.01
#define b 0.01
#define d_wheel 80      // speed = 3.14 * d_wheel * v_m
#define speed_cmd 23

#define FILTER_DEEP 4                                  /*!<  PWM Low pass filter variable */

static int32_t Vs_m[5]={0};
uint32_t v_x, v_y, v_w;
uint8_t data_len = 1 ;
uint8_t cmd_send_buf[10]={0x3E,0xA2,0,0,0,0,0,0,0,0};
uint8_t motor_stop_cmd[5]={0x3E,0x81,0x01,0x00,0xC0};
uint8_t buffer_completed=0;

uint32_t PWM_tmp_buffer[FILTER_DEEP];                 /*!<  PWM filter variable */
uint32_t index_filter = 1;                            /*!<  PWM filter variable */
uint32_t PWM_filtered = 0;                            /*!<  PWM filter variable */
uint32_t PWM_sum_filt = 0;                            /*!<  PWM filter variable */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

uint8_t CaptureNumber_CH1 = 0;
uint8_t CaptureNumber_CH2 = 0;
uint8_t CaptureNumber_CH3 = 0;

uint32_t capture_Buf_CH1[2]={0};
uint32_t capture_Buf_CH2[2]={0};
uint32_t capture_Buf_CH3[2]={0};

uint32_t Ton_value_CH1;
uint32_t Ton_value_CH2;
uint32_t Ton_value_CH3;

extern uint8_t Uart3_receive_buf[26];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Velocity_calculation( int32_t vt_x, int32_t vt_y, int32_t vt_w );
void printf_pwm(void);
void motor_stop_send_out(uint8_t motor_id);
int32_t pwm_to_speed (uint32_t Ton_value );




void motor_speed_send_out(void)
{
  int32_t v_x, v_y, v_w ;
  uint8_t id;
  
 
  v_x = pwm_to_speed ( Ton_value_CH1 );
  v_y = pwm_to_speed ( Ton_value_CH2 );
  v_w = pwm_to_speed ( 0 );
  
  if (v_x==0 && v_y==0 )
  {
    
    for(id=1;id<5;id++)
   {
      motor_stop_send_out( id ) ;
      
   }
  
  }
  else 
  {
  
    Velocity_calculation( v_x, v_y, v_w );
    
  }      
  
   for(id=1;id<5;id++)
   {
      speed_send_out( Vs_m[id] ,id) ;
      printf("Vs_m[id] : %d \r\n",Vs_m[id]);
   }
  
}

int32_t pwm_to_speed (uint32_t Ton_value )
{

  int32_t speed ;
  uint32_t pwm_on_time ;
  
  pwm_on_time = Ton_value;

  if((pwm_on_time>1520) && (pwm_on_time<2100))
  {
    speed = (pwm_on_time -1500 ) * (200-15) + 15000 ;
    
  }
  else if((pwm_on_time>900) && (pwm_on_time<1480))
  {
    speed = (pwm_on_time -1500 ) * (200-15) - 15000 ;
    
  }
  else 
  {
    speed = 0 ;   
  }
  return speed;
}



/*
            m2 ----- m1
             |        \
             \        /
            m3 ----- m4
*/

void Velocity_calculation( int32_t vt_x, int32_t vt_y, int32_t vt_w )
{
  
  //Vs_m[1] = vt_x - vt_y + vt_w * ( a + b );
  //Vs_m[2] = -(vt_x + vt_y - vt_w * ( a + b ));
  //Vs_m[3] = -(vt_x - vt_y - vt_w * ( a + b ));
  //Vs_m[4] = vt_x + vt_y + vt_w * ( a + b );
  
  Vs_m[1] = vt_y - vt_x ;
  Vs_m[2] = -(vt_y + vt_x );
  Vs_m[3] = vt_y - vt_x ;
  Vs_m[4] = vt_y + vt_x ;
  

}



void speed_send_out( uint32_t motor_speed , uint8_t motor_id )
{
  //uint32_t motor_speed = 20000 ;
  
  cmd_send_buf[0] = 0x3E ;        // header
  cmd_send_buf[1] = 0xA2 ;        // cmd_id
  cmd_send_buf[2] = motor_id ;           // motor_id
  cmd_send_buf[3] = 0x04 ;        // data_length
  cmd_send_buf[4] = cmd_send_buf[0] + cmd_send_buf[1] + cmd_send_buf[2] + cmd_send_buf[3];        // check_sum
  
  cmd_send_buf[5] =  motor_speed&0XFF ;        // data
  cmd_send_buf[6] = (motor_speed>>8)&0XFFFFFF;        // data
  cmd_send_buf[7] = (motor_speed>>16)&0XFFFF ;        // data
  cmd_send_buf[8] = (motor_speed>>24)&0XFF ;        // data
  cmd_send_buf[9] = cmd_send_buf[5] + cmd_send_buf[6] + cmd_send_buf[7] + cmd_send_buf[8];        // check_sum  
  
  HAL_UART_Transmit(&huart5, cmd_send_buf, 10, 10);
  HAL_Delay(3);
}



void motor_stop_send_out(uint8_t motor_id)
{
  
   //uint32_t motor_speed = 20000 ;
  
  cmd_send_buf[0] = 0x3E ;        // header
  cmd_send_buf[1] = 0x81 ;        // cmd_id
  cmd_send_buf[2] = motor_id ;           // motor_id
  cmd_send_buf[3] = 0x00 ;        // data_length
  cmd_send_buf[4] = cmd_send_buf[0] + cmd_send_buf[1] + cmd_send_buf[2] + cmd_send_buf[3];        // check_sum
   
  HAL_UART_Transmit(&huart5, cmd_send_buf, 5, 10);
  HAL_Delay(3);  
}


void printf_pwm(void)
{
        printf("capture_Buf_CH1[0] : %d \r\n",capture_Buf_CH1[0]);
        printf("capture_Buf_CH1[1] : %d \r\n",capture_Buf_CH1[1]);
        printf("Ton_value_CH1 : %d \r\n",Ton_value_CH1);
        printf("\r\n");
        printf("\r\n");
        
        printf("capture_Buf_CH2[0] : %d \r\n",capture_Buf_CH2[0]);
        printf("capture_Buf_CH2[1] : %d \r\n",capture_Buf_CH2[1]);
        printf("Ton_value_CH2 : %d \r\n",Ton_value_CH2);
        printf("\r\n");
        printf("\r\n");
        
        printf("capture_Buf_CH3[0] : %d \r\n",capture_Buf_CH3[0]);
        printf("capture_Buf_CH3[1] : %d \r\n",capture_Buf_CH3[1]);
        printf("Ton_value_CH3 : %d \r\n",Ton_value_CH3);
        printf("\r\n");
        printf("\r\n");
}


uint16_t CH[18];  // 
uint8_t  rc_flag = 0;

void Sbus_Data_Decode(uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
	CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
	CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
	CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
	CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
	CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
	CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
	CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
	CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}


void Sbus_test(void)
{
  if(Uart3_receive_buf[0] == 0x0F)
  {
  Sbus_Data_Decode(Uart3_receive_buf);
  printf("CH[0]=%d\r\n",CH[0]);
  printf("CH[1]=%d\n\r",CH[1]);
  printf("\r\n"); 
  }
  else
  {
  printf("not find head\r\n");
  }
 
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{


  /* TIM3_CH1 Capture */
if(htim->Instance == TIM3 )
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 )
  {
    if (CaptureNumber_CH1 == 0)
    {
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
      capture_Buf_CH1[0] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
      CaptureNumber_CH1 = 1;
      //printf("capture_Buf_CH1[0] : %d \r\n",capture_Buf_CH1[0]);
    }
    else
    {
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);  //设置为RISING捕获
      capture_Buf_CH1[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
      CaptureNumber_CH1 = 0;
      //printf("capture_Buf_CH1[1] : %d \r\n",capture_Buf_CH1[1]);
      
    }
  if (CaptureNumber_CH1 == 0)
  {
    if(capture_Buf_CH1[1] > capture_Buf_CH1[0])
    {
      Ton_value_CH1 = capture_Buf_CH1[1]- capture_Buf_CH1[0];
      //printf("Ton_value_CH1 : %d \r\n",Ton_value_CH1);
    }

  }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
  {
    if (CaptureNumber_CH2 == 0)
    {
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
      capture_Buf_CH2[0] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//获取当前的捕获值.
      CaptureNumber_CH2 = 1;
      //printf("capture_Buf_CH2[0] : %d \r\n",capture_Buf_CH2[0]);
    }
    else
    {
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);  //设置为RISING捕获
      capture_Buf_CH2[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//获取当前的捕获值.
      CaptureNumber_CH2 = 0;
      //printf("capture_Buf_CH2[1] : %d \r\n",capture_Buf_CH2[1]);
      
    }
  if (CaptureNumber_CH2 == 0)
  {
    if(capture_Buf_CH2[1] > capture_Buf_CH2[0])
    {
      Ton_value_CH2 = capture_Buf_CH2[1]- capture_Buf_CH2[0];
    //printf("Ton_value_CH2 : %d \r\n",Ton_value_CH2);
    }

  }
}
}
}

}
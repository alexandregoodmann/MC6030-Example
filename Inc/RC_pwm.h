

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void printf_pwm(void);
void speed_send_out( uint32_t motor_speed , uint8_t motor_id );
void motor_speed_send_out(void);
void Sbus_Data_Decode(uint8_t *buf);
void Sbus_test(void);

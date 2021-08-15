

//#define 

void speedControl(uint8_t Motor_ID, int32_t speedControl);
void Multi_angleControl_1(uint8_t Motor_ID, int32_t angleControl);
void Multi_angleControl_2(uint8_t Motor_ID, uint16_t maxSpeed, int32_t angleControl);
void Single_loop_angleControl_1(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl);
void Single_loop_angleControl_2(uint8_t Motor_ID, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl);
void Multi_Wr_Accel_Ram(uint8_t Motor_ID, int32_t Accel);
void Motor_Off(uint8_t Motor_ID);

void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1);
void RS_angleControl_2(uint8_t Motor_ID, int64_t angleControl_1, uint32_t maxSpeed);
void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1);
void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed);
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl);
void RS_Motor_Off(uint8_t Motor_ID);

uint8_t Mode_Select();


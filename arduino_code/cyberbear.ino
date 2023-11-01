uint32_t CAN_ID = 1; // Xiaomi 모터의 CAN ID
double R_MIN_RAD_S = 3.1415926535 / 30; // 최소 라디안 속도

#define Master_CAN_ID 0x00

// 통신 유형 정의
#define Communication_Type_GetID 0x00     // 장치의 ID 및 64비트 MCU 고유 식별자 얻기
#define Communication_Type_MotionControl 0x01 	// 제어 명령 전송
#define Communication_Type_MotorRequest 0x02	// 모터 상태 피드백
#define Communication_Type_MotorEnable 0x03	// 모터 활성화
#define Communication_Type_MotorStop 0x04	// 모터 정지
#define Communication_Type_SetPosZero 0x06	// 모터 영점 설정
#define Communication_Type_CanID 0x07	// 현재 모터 CAN_ID 변경
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	
#define Communication_Type_SetSingleParameter 0x12	
#define Communication_Type_ErrorFeedback 0x15

#define Run_mode 0x7005	
#define Iq_Ref 0x7006
#define Spd_Ref 0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain 0.1

#define MAX_P 720
#define MIN_P -720
#define MAX_S 30
#define MIN_S -30
#define MAX_T 12
#define MIN_T -12

#define Motor_Error 0x00
#define Motor_OK 0X01

#define Current_mode 3
#define Position_mode 1
#define Motion_mode 0
#define Speed_mode 2


/*Functions------------------------------------------------------------------*/

void Float_to_Byte(float f, uint8_t byte[4]) {
  unsigned long longdata = *(unsigned long*)&f;       
  byte[0] = (longdata >> 24) & 0xFF;
  byte[1] = (longdata >> 16) & 0xFF;
  byte[2] = (longdata >> 8) & 0xFF;
  byte[3] = longdata & 0xFF;
}

void send_command(uint32_t id_num, uint8_t cmd_mode, uint8_t* tx_data)
{
    uint8_t cmd_data[2] = {0}; // Initialize cmd_data with zeros
    cmd_data[0] = 0 & 0xFF; // Set cmd_data[0] to a specific value (you might want to change this)
    uint8_t cdata[13] = {0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    cdata[1] = cmd_mode;
    cdata[2] = cmd_data[1];
    cdata[3] = cmd_data[0];
    cdata[4] = id_num;

    for (int i = 0; i < 8; i++) {
        cdata[5 + i] = tx_data[i];
    }

    uint8_t udata[16] = {0xAA, 1, 0, 0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    for (int i = 0; i < 12; i++) {
        udata[4 + i] = cdata[i + 1];
    }

    Serial.write("id_num ");
    String myString = String(id_num);
    Serial.println(myString);
 

    Serial.write("cmd_mode ");
    myString = String(cmd_mode);
    Serial.println(myString);
 
    for (int i = 0; i < 16; i++) {
    myString = String(udata[i]);
    Serial.print(myString);
    Serial.print(' ');
    }
    Serial.println();
    Serial1.write(udata, 16);
    delay(1);
}

void Init_Motor(uint32_t Can_Id,float mode){
	Enable_Motor(Can_Id);
  Set_Mode(Can_Id,mode);
}

void Enable_Motor(uint32_t CAN_ID){
	uint32_t Send_ID;
	uint8_t temp[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  send_command(CAN_ID, 3, temp);  

}

void Stop_Motor(uint32_t CAN_ID, uint8_t clear_error){
	uint32_t Send_ID;
	uint8_t temp[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	temp[0]=clear_error;
  send_command(CAN_ID, 4, temp); 
}

void Set_Motor_Parameter(uint32_t CAN_ID,uint16_t Index,float Value,char Value_type){
	uint32_t Send_ID;
	uint8_t Send_Data[8];
	Send_Data[0]=Index;
	Send_Data[1]=Index>>8;
	Send_Data[2]=0x00;
	Send_Data[3]=0x00;
  
	if(Value_type == 'f'){
		uint8_t byte[4];
    Float_to_Byte(Value, byte);
		Send_Data[4]=byte[3];
		Send_Data[5]=byte[2];
		Send_Data[6]=byte[1];
		Send_Data[7]=byte[0];		
	}
	else if(Value_type == 's'){
		Send_Data[4]=(uint8_t)Value;
		Send_Data[5]=0x00;
		Send_Data[6]=0x00;
		Send_Data[7]=0x00;				
	}
  send_command(CAN_ID, 18, Send_Data); 
  
	//Send_Msg(Send_Data,Send_ID);	
}

void Set_Mode(uint32_t CAN_ID ,float Mode){
  Set_Motor_Parameter(CAN_ID,0x7005	,Mode,'s');
}

/* Exported function declarations --------------------------------------------*/

void setup()
{
  Serial.begin(115200); // Serial은 Xiaomi 모터를 제어하기 위한 CAN 모듈에 연결
  Serial1.begin(115200); // Serial1은 디버그 데이터를 출력
}


void loop()
{  
  Init_Motor(CAN_ID,  Speed_mode);
  Set_Motor_Parameter(CAN_ID,  Spd_Ref,  0 * R_MIN_RAD_S,  'f');
  delay(1000);

  Set_Motor_Parameter(CAN_ID,  Limit_Cur,  15,  'f');
  Set_Motor_Parameter(CAN_ID,  Spd_Ref,  50 * R_MIN_RAD_S,  'f');

  delay(1000);
  Set_Motor_Parameter(CAN_ID,  Spd_Ref,  500 * R_MIN_RAD_S,  'f');
  delay(1000);

  Stop_Motor(CAN_ID,  1);
  delay(1000);
}

// 명령 예제
// 170 1 0 8 3 0 0 1 0 0 0 0 0 0 0 0  활성화
// 170 1 0 8 18 0 0 1 5 112 0 0 2 0 0 0  운동 모드로 설정
// 170 1 0 8 18 0 0 1 24 112 0 0 0 0 112 65  전류 매개변수 설정
// 170 1 0 8 18 0 0 1 10 112 0 0 210 83 251 65  속도 매개변수 설정
// 170 1 0 8 4 0 0 1 1 0 0 0 0 0 0 0 정지



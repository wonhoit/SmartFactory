#include <MsTimer2.h> 
#include <VitconBrokerComm.h>

#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <util/delay.h>

volatile int timerFlag = 0; 

using namespace vitcon;
// Pins
#define APDS9960_INT    2 // Needs to be an interrupt pin
#define CONVEYOR_RELAY  6
#define LED_INSPECT     4

#define DELAY_INT   300     // button debounce

// Constants
#define PROX_INT_HIGH   254 // Proximity level for interrupt
#define PROX_INT_LOW    200  // No far interrupt

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
uint8_t proximity_data = 0;
volatile bool isr_flag = false;
// volatile bool conveyor_flag = false;

/* VITCON FUNCTION */
// 9: ROBOT RED LIGHT, 8: ROBOT GREEN LIGHT, 7: CONVEYOR RED LIGHT, 6: CONVEYOR GREEN LIGHT
const int RELAY[] = { 9, 8, 7, 6 }; 
const int Input[] = { A0, A1 }; // A0 : CONVEYOR BTN, A1 : ROBOT BTN
bool btn_states[3] = { false };   // 버튼입력 관련 변수
bool rly_states[4] = { true, false, true, false };   // 릴레이 관련 변수
unsigned long InputTime[] = { 0, 0 }; // 조광 푸쉬 버튼 채터링 방지

bool swsel_val[2] = { false };   // RELAY-LINK 인터락 관련 변수선언
bool onoff_val[2] = { false };   // RELAY-LINK ON/OFF 관련 변수선언
bool modeSel = false;             // 수동 자동 모드관련 변수선언
bool Sig1PushAlarm = false;       // 1번신호 pushAlarm알람관련 변수선언
bool Sig2PushAlarm = false;       // 2번신호 pushAlarm알람관련 변수선언
bool Sig3PushAlarm = false;       // 3번신호 pushAlarm알람관련 변수선언
bool Sig4PushAlarm = false;       // 4번신호 pushAlarm알람관련 변수선언


#define ITEM_COUNT 27
void sel_out(bool val) { modeSel = val; } // 어플,웹상으로 모드 스위치를 눌렀을때 값을 modeSel 변수로 받음
IOTItemBin selBt(sel_out);                // 모드 스위치 관련 아이템선언
IOTItemBin selBt_status;                  // 모드 스위치 관련 아이템선언2
IOTItemBin m_auto;                        // 자동모드시 상태 알림등 아이템선언
IOTItemBin m_manual;                      // 수동모드시 상태 알림등 아이템선언 
IOTItemBin SIG1;                          // 자동모드시 신호 알림등 아이템선언
IOTItemBin SIG2;
IOTItemBin Robot_R;
IOTItemBin Robot_G;
IOTItemBin Conv_R;
IOTItemBin Conv_G;

   

void interlock1(bool val) { swsel_val[0] = val; }   // RELAY-LINK 인터락 관련 변수선언
void interlock2(bool val) { swsel_val[1] = val; }
//void interlock3(bool val) { swsel_val[2] = val; }

void in1_onoff(bool val) { onoff_val[0] = val; }    // RELAY-LINK ON/OFF 관련 변수선언
void in2_onoff(bool val) { onoff_val[1] = val; }
//void in3_onoff(bool val) { onoff_val[2] = val; }

IOTItemBin sw0(in1_onoff);                          // RELAY-LINK ON/OFF 관련 아이템선언
IOTItemBin sw1(in2_onoff);
IOTItemBin sw2(interlock1);                         // RELAY-LINK 인터락 관련 아이템선언
IOTItemBin sw3(interlock2);
IOTItemPsh pushAlarm;                               // pushAlarm알람 아이템 선언

//IOTItemBin sw_status[6];                            // RELAY-LINK 제어 토글스위치 관련 아이템선언
IOTItemBin sw_status[4];

IOTItemInt Accept;  // 웹에표시될 변수
IOTItemInt Defect;
IOTItemInt RED_num;
IOTItemInt GREEN_num;
IOTItemInt BLUE_num;
IOTItemInt ORANGE_num;
IOTItemInt Accept_rate;
IOTItemInt Total;

/* 물체 카운팅 변수 */
int _Accept, _Defect, _RED_num, _GREEN_num, _BLUE_num, _ORANGE_num, sum_num = 0;
int _Accept_rate = 0;

IOTItem *items[ITEM_COUNT] = { &selBt_status, &selBt, &m_auto, &m_manual, &sw_status[0],      //0 ~ 4
                               &sw0, &sw_status[1], &sw1, &sw_status[2], &sw2,                //5 ~ 9
                               &sw_status[3], &sw3, &SIG1, &SIG2, &Accept,                    //10 ~ 14
                               &Defect, &RED_num, &GREEN_num, &pushAlarm, &BLUE_num,         //15 ~ 19
                               &Accept_rate, &Total, &ORANGE_num,                           //20 ~ 22
                               &Robot_R, &Robot_G, &Conv_R, &Conv_G                         //23 ~ 26  추가한 버튼 06.05 mjmjmj 8888888888888888888888888888888
                                };                                                    


/* 현재 장비의 ID */
const char device_id[] = "14bddda2feaa210818adea0d7bd76477"; 
BrokerComm comm(&Serial, device_id, items, ITEM_COUNT);

// 카운트
void count_flag() 
{ 
  timerFlag++;
}

//타이머 카운트
void temp_check(){
  if(timerFlag == 10){
    timerFlag = 0; 
  }
}

void setup() {
  pinMode(APDS9960_INT, INPUT);
  pinMode(LED_INSPECT, OUTPUT);
  
  // pinMode(CONVEYOR_INT, INPUT_PULLUP);

  /* VITCON SETUP */
  for(int i = 0; i < 4; i++) { pinMode(RELAY[i], OUTPUT); } // 6 >> 4
  for(int i = 0; i < 2; i++) { pinMode(Input[i], INPUT); }  // 4 >> 2
  digitalWrite(CONVEYOR_RELAY, HIGH);
  digitalWrite(LED_INSPECT, HIGH);
  
  Serial.begin(250000);

  // Count Timer
  MsTimer2::set(100, count_flag); // 0.1 sec period 
  MsTimer2::start();

  comm.SetInterval(200);     


  // Initialize interrupt service routine
  attachInterrupt(0, interruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
//    Serial.println(F("APDS-9960 initialization complete"));
  } else {
//    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
//    Serial.println(F("Light sensor is now running"));
  } else {
//    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
//    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Set proximity interrupt thresholds
  if ( !apds.setProximityIntLowThreshold(PROX_INT_LOW) ) {
//    Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setProximityIntHighThreshold(PROX_INT_HIGH) ) {
//    Serial.println(F("Error writing high threshold"));
  }
  
  // Start running the APDS-9960 proximity sensor (interrupts)
  if ( apds.enableProximitySensor(true) ) {
//    Serial.println(F("Proximity sensor is now running"));
  } else {
//    Serial.println(F("Something went wrong during sensor init!"));
  }

  // Wait for initialization and calibration to finish
  delay(500);
}

/* 버튼 모드 */
void button_mode() {
  // add button flag
  if ( !btn_states[0] && digitalRead(Input[0]) == 1 ) {
    rly_states[0] = !rly_states[0];
    rly_states[1] = !rly_states[1];
    if(onoff_val[0]) digitalWrite(RELAY[0], rly_states[0]);   // 1번 릴레이
    else if(!onoff_val[0]) digitalWrite(RELAY[0], LOW);
    if(onoff_val[0]) digitalWrite(RELAY[1], rly_states[1]);   // 2번 릴레이
    else if(!onoff_val[0]) digitalWrite(RELAY[1], LOW);
    btn_states[0] = true;
    InputTime[0] = millis();
  }
  else if ( digitalRead(Input[0]) == 0 && millis() - InputTime[0] > DELAY_INT ) {
    btn_states[0] = false;
  }

  if ( !btn_states[1] && digitalRead(Input[1]) == 1 ) {
    rly_states[2] = !rly_states[2];
    rly_states[3] = !rly_states[3];
    if(onoff_val[1]) digitalWrite(RELAY[2], rly_states[2]);   // 3번 릴레이
    else if(!onoff_val[1]) digitalWrite(RELAY[2], LOW);
    if(onoff_val[1]) digitalWrite(RELAY[3], rly_states[3]);   // 4번 릴레이
    else if(!onoff_val[1]) digitalWrite(RELAY[3], LOW);
    btn_states[1] = true;
    InputTime[1] = millis();
  }
  else if ( digitalRead(Input[1]) == 0 && millis() - InputTime[1] > DELAY_INT ) {
    btn_states[1] = false;
  }

  m_auto.Set(HIGH);
  m_manual.Set(LOW);
}

/* 수동 모드 */
void iot_mode() {

  //# 2 ?? 확인해서 수정필요
for(int i=0; i<2; i++) {
  if(onoff_val[i]) {
    if(!swsel_val[i]) { digitalWrite(RELAY[i*2], HIGH); digitalWrite(RELAY[i*2+1], LOW); }
    if(swsel_val[i]) { digitalWrite(RELAY[i*2], LOW); digitalWrite(RELAY[i*2+1], HIGH); }
  }
  if(!onoff_val[i]) {
    digitalWrite(RELAY[i*2], LOW);
    digitalWrite(RELAY[i*2+1], LOW);
  }
}
  
  m_auto.Set(LOW);
  m_manual.Set(HIGH);
}
void loop() {
  modeSel ? iot_mode() : button_mode();

  temp_check();

  //# 1번 신호 푸쉬알람  
  if(modeSel&&!Sig1PushAlarm && digitalRead(Input[0])==HIGH && digitalRead(Input[1])==LOW){
      pushAlarm.Set("1번 신호가 들어왔습니다!");
      Sig1PushAlarm = true;
     }
  else if(digitalRead(Input[0])==LOW || !modeSel){
    Sig1PushAlarm = false;
  }

  //# 2번 신호 푸쉬알람  
  if(modeSel&&!Sig2PushAlarm &&digitalRead(Input[1])==HIGH){
      pushAlarm.Set("2번 신호가 들어왔습니다!");
      Sig2PushAlarm = true;
     }
  else if(digitalRead(Input[1])==LOW|| !modeSel){
    Sig2PushAlarm = false;
  }

/* 자동모드 신호 LED*/
  if(digitalRead(Input[0])==HIGH) SIG1.Set(HIGH);
  else SIG1.Set(LOW);
  if(digitalRead(Input[1])==HIGH) SIG2.Set(HIGH);
  else SIG2.Set(LOW);

/*컨베이어 상태*/
  if(digitalRead(RELAY[3])==HIGH) {
    Conv_G.Set(HIGH);
    digitalWrite(LED_INSPECT, HIGH);
  }
  else Conv_G.Set(LOW);
  
  if(digitalRead(RELAY[2])==HIGH)
  {
    Conv_R.Set(HIGH);
    digitalWrite(LED_INSPECT, LOW);
  }
  else Conv_R.Set(LOW);        

/*로봇 상태*/
 if(digitalRead(RELAY[1])==HIGH) Robot_G.Set(HIGH);  
  else Robot_G.Set(LOW);
  
  if(digitalRead(RELAY[0])==HIGH) Robot_R.Set(HIGH);
  else Robot_R.Set(LOW);        
       
  
  //# 
  sw_status[0].Set(onoff_val[0]);
  sw_status[1].Set(onoff_val[1]);
  sw_status[2].Set(swsel_val[0]);
  sw_status[3].Set(swsel_val[1]);
  //sw_status[4].Set(swsel_val[1]);
  //sw_status[5].Set(swsel_val[2]);

  selBt_status.Set(modeSel);
  Accept.Set(_Accept);
  Defect.Set(_Defect);
  RED_num.Set(_RED_num);
  GREEN_num.Set(_GREEN_num);
  ORANGE_num.Set(_ORANGE_num);
  BLUE_num.Set(_BLUE_num);
  Accept_rate.Set(_Accept_rate);
  Total.Set(sum_num);

  comm.Run();          
  /* vitcon send END */
  
  // If interrupt occurs, print out the proximity level
  if ( ( digitalRead(CONVEYOR_RELAY) == HIGH ) && (isr_flag == HIGH) ) {
    // Read proximity level and print it out
    if ( !apds.readProximity(proximity_data) ) {
//      Serial.println("Error reading proximity value");
    } else {
//      Serial.print("Proximity detected! Level: ");
//      Serial.println(proximity_data);
      if (proximity_data > PROX_INT_HIGH) {
        delay(200);
        digitalWrite(CONVEYOR_RELAY, LOW);
        digitalWrite(7, HIGH);
        delay(500);
        read_light_level();
        delay(500);
        digitalWrite(CONVEYOR_RELAY, HIGH);
        digitalWrite(7, LOW);
        delay(2000);
      }
    }
    // Reset flag and clear APDS-9960 interrupt (IMPORTANT!)
    isr_flag = LOW;
    if ( !apds.clearProximityInt() ) {
//      Serial.println("Error clearing interrupt");
    }
  }
}


void interruptRoutine() {
  isr_flag = HIGH;
}

void read_light_level(){
  
    // Read the light levels (ambient, red, green, blue)
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
//    Serial.println("Error reading light values");
  } else {
    /* red green blue arrayADDR */
    double hsv_result[3] = {0,};
    rgbToHsv(red_light, green_light, blue_light, hsv_result);

    // 빨간색 물체
    if ( hsv_result[0] > 0.95 || hsv_result[0] < 0.08) {
      if( red_light < 420) _RED_num++;
      else _ORANGE_num++;
    }
  
    // 녹색 물체
    else if( hsv_result[0] > 0.40 && hsv_result[0] < 0.50) {
      _GREEN_num++;
    }

    // 파란색 물체
    else if ( hsv_result[0] > 0.50 && hsv_result[0] < 0.60) {
      _BLUE_num++;
    }
    
    // 불량품
    else {
    	_Defect++;
  	}
     
    _Accept = _RED_num + _GREEN_num + _BLUE_num + _ORANGE_num;
    sum_num = _Accept + _Defect;
    _Accept_rate = (int)(((float)_Accept / (float)sum_num) * 100);
  }
}

void rgbToHsv(uint16_t r, uint16_t g, uint16_t b, double hsv[]) {
    double rd = (double) r/255;
    double gd = (double) g/255;
    double bd = (double) b/255;
    double max = threeway_max(rd, gd, bd), min = threeway_min(rd, gd, bd);
    double h, s, v = max;

    double d = max - min;
    s = max == 0 ? 0 : d / max;

    if (max == min) { 
        h = 0; // achromatic
    } else {
        if (max == rd) {
            h = (gd - bd) / d + (gd < bd ? 6 : 0);
        } else if (max == gd) {
            h = (bd - rd) / d + 2;
        } else if (max == bd) {
            h = (rd - gd) / d + 4;
        }
        h /= 6;
    }

    hsv[0] = h;
    hsv[1] = s;
    hsv[2] = v;
}

double threeway_max(double a, double b, double c) {
    return max(a, max(b, c));
}

double threeway_min(double a, double b, double c) {
    return min(a, min(b, c));
}

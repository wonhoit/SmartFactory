#include <Servo.h>

#define DEBUG_PRINT 0

#define BASE 2
#define SHOULDER 3
#define ELBOW 9
#define WRIST 8
#define GRIP 5
#define GRIPPER 6
#define RELAY_AIR  11

#define SERVOS_CNT 6
int xIntPre, yIntPre, zIntPre= 0;
int xAxis, yAxis, zAxis = 0;
char preColor, objectColor = '\0';
int inCome = 0;

// const int SERVOS_CNT = 6;
const double Pi = 3.141592;
const double gravityConst = 6.673/2; // 6.673
int PIN[SERVOS_CNT], value[SERVOS_CNT], idle[SERVOS_CNT], currentAngle[SERVOS_CNT], MIN[SERVOS_CNT];
int MAX[SERVOS_CNT], INITANGLE[SERVOS_CNT], previousAngle[SERVOS_CNT], ANA[SERVOS_CNT];
Servo myservo[SERVOS_CNT];

void setup() {
  Serial.begin(115200);
  //#1 서보모터
  PIN[0] = BASE; //서보모터 IO를 3번핀으로 지정
  MIN[0] = 0; //서보모터 최소 회전각도
  MAX[0] = 180; //서보모터 최대 회전각도
  INITANGLE[0] = 90; //서보모터 초기각도
  ANA[0] = 0; //조이스틱스위치입력 IO를 아날로그 A0번핀으로 지정
  //#2 서보모터
  PIN[1] = SHOULDER;
  MIN[1] = 0; 
  MAX[1] = 180; 
  INITANGLE[1] = 30;
  ANA[1] = 1;
  //#3 서보모터
  PIN[2] = ELBOW;
  MIN[2] = 0;
  MAX[2] = 180;
  INITANGLE[2] = 15;
  ANA[2] = 2;
  //#4 서보모터
  PIN[3] = WRIST;
  MIN[3] = 0;
  MAX[3] = 180;
  INITANGLE[3] = 145;  
  ANA[3] = 3;
  //#5 서보모터
  PIN[4] = GRIP;
  MIN[4] = 0;
  MAX[4] = 180;
  INITANGLE[4] = 50;
  ANA[4] = 4;
  //#6 서보모터
  PIN[5] = GRIPPER;
  MIN[5] = 0;
  MAX[5] = 180;
  INITANGLE[5] = 90;
  ANA[5] = 5;
  for (int i = 0; i < SERVOS_CNT ; i++)
  {
    idle[i] = 0;
    value[i] = 0;
  }
  
  pinMode(RELAY_AIR, OUTPUT);
  digitalWrite(RELAY_AIR, LOW);  // AIR PUMP OFF
  
  set_arm_delay(INITANGLE, 5);
}

void loop() {
  Serial.print('$');
  Serial.flush();
  delay(500);
  if(read_serial_axis(&xAxis, &yAxis, &zAxis, &objectColor)) {
    if ( set_arm_inv(double(yAxis), double(0), double(xAxis), 98, 90, 5) ) {
        delay(300);
        digitalWrite(RELAY_AIR, HIGH);  // AIR PUMP ON
        delay(500);
        off_object_color(objectColor);  // off object
    }
    set_arm_delay(INITANGLE, 5);
    for (int i = 0; i < SERVOS_CNT ; i++) { idle[i] = 0; }
    while( Serial.available()  > 0 ) { Serial.read(); } // 동작 중에 수신한 데이터 비우기
  }
  
  detach_servo();
}

void off_object_color(char objectColorIn) {
  INITANGLE[0] = myservo[0].read();
  set_arm_delay(INITANGLE, 5);
  switch(objectColorIn) {
    case 'R':
      set_arm_inv(50, 20, -240, 95, 90, 5);
      break;
    case 'G':
      set_arm_inv(50, 20, -160, 95, 90, 5);
      break;
    case 'B':
      set_arm_inv(50, 20, 160, 95, 90, 5);
      break;
    case 'Y':
      set_arm_inv(50, 20, 240, 95, 90, 5);
      break;
  }
  delay(500);
  digitalWrite(RELAY_AIR, LOW);  // AIR PUMP OFF
}

/* working */ 
bool read_serial_axis(int* xAxis, int* yAxis, int* zAxis, char* objectColor) {
  if( Serial.available() ) {
    while( Serial.read() != '@' );  // 시작 문자 발견까지 시리얼 읽음
    String contString = Serial.readStringUntil('\n');  // readString(), readStringUntil()
    contString.trim();  // remove space
    #if DEBUG_PRINT
    Serial.println(contString);
    #endif
    int startIndex = 0;
    int xIndex = contString.indexOf(','); // find ',' position
    int yIndex = contString.indexOf(',', xIndex+1);
    int lastIndex = contString.length();
    *objectColor = contString.charAt(startIndex);
    String x_String = contString.substring(startIndex + 1, xIndex);
    String y_String = contString.substring(xIndex + 1, yIndex);
    String z_String = contString.substring(yIndex + 1, lastIndex);
    // if want to float, String.toFloat()
    *xAxis = x_String.toInt();
    *yAxis = y_String.toInt();
    *zAxis = z_String.toInt();
    preColor = *objectColor;
    xIntPre = *xAxis;
    yIntPre = *yAxis;
    zIntPre = *zAxis;
    return true;
  }
  return false;
}

void set_arm(int servo_angle[SERVOS_CNT]) {
  for (int i = 0; i < SERVOS_CNT; i++){
    if (!myservo[i].attached()){
      myservo[i].attach(PIN[i]);
    }
    #if DEBUG_PRINT
    switch( PIN[i] ){
      case BASE:
        Serial.print(" BASE \t: ");
        break;
      case SHOULDER:
        Serial.print(" SHOULDE: ");
        break;
      case ELBOW:
        Serial.print(" ELBOW\t: ");
        break;
      case WRIST:
        Serial.print(" WRIST\t: ");
        break;
      case GRIP:
        Serial.print(" GRIP\t: ");
        break;
      case GRIPPER:
        Serial.print(" GRIPPER: ");
        break;
    }
    Serial.println(servo_angle[i]);
    #endif
    // invalid value
    servo_angle[i] < MAX[i] ? servo_angle[i] : MAX[i];
    servo_angle[i] > MIN[i] ? servo_angle[i] : MIN[i];
    
    myservo[i].write(servo_angle[i]);
    value[i] = 0;
    idle[i] = 0;
    previousAngle[i]=servo_angle[i];
    currentAngle[i]=servo_angle[i];
  }
}

void detach_servo(){
  for (int i = 0; i < SERVOS_CNT; i++){
    idle[i]++;
    if (idle[i] > 30){
      myservo[i].detach(); //서보모터를 일정시간 사용하지 않으면 연결을 끊어둔다.
      #if DEBUG_PRINT
      Serial.print(" > DETACHED SERVO MOTOR ");
      Serial.println(PIN[i]);
      #endif
    }
  }
} 

void set_arm_delay(int servo_angle[SERVOS_CNT], int de) {
  int valueSum = 0;
  for (int i = 0; i < SERVOS_CNT; i++){
    #if DEBUG_PRINT
    switch( PIN[i] ){
      case BASE:
        Serial.print(" BASE\t: ");
        break;
      case SHOULDER:
        Serial.print(" SHOULDE: ");
        break;
      case ELBOW:
        Serial.print(" ELBOW\t: ");
        break;
      case WRIST:
        Serial.print(" WRIST\t: ");
        break;
      case GRIP:
        Serial.print(" GRIP\t: ");
        break;
      case GRIPPER:
        Serial.print(" GRIPPER: ");
        break;
    }
    Serial.println(servo_angle[i]);
    #endif
    // invalid value check
    servo_angle[i] < MAX[i] ? servo_angle[i] : MAX[i];
    servo_angle[i] > MIN[i] ? servo_angle[i] : MIN[i];
    ++value[i];
    if (!myservo[i].attached()){
      myservo[i].attach(PIN[i]);
    }
  }
  
  while(true){
    for (int i = 0; i < SERVOS_CNT; i++){
      currentAngle[i] = myservo[i].read();
      
      if ( currentAngle[i] < servo_angle[i] ) ++currentAngle[i];
      else if ( currentAngle[i] > servo_angle[i] ) --currentAngle[i];

      myservo[i].write(currentAngle[i]);

      if ( currentAngle[i] == servo_angle[i] ) value[i] = 0;
      delay(de);
    }

    /* END 판단 */
    valueSum = 0;
    for(int j = 0 ; j < SERVOS_CNT ; j++) {
      valueSum += value[j];
    }
    if(valueSum == 0) {
      #if DEBUG_PRINT
      Serial.println(" > END OF THE ROBOT ARM MOVING ");
      #endif
      break;
    }
  }
}

/* 좌표계 계산용 */
double rad_to_degree(double radValue)
{
  return radValue * 180.0 / Pi;
}

double degree_to_rad(double degreeValue)
{
  return degreeValue * Pi / 180.0;
}

bool check_value(double axisValue)
{
  if ( (axisValue > 180.001) || (axisValue < -0.001) || isnan(axisValue)) return true;
  else return false;
}

/* set_arm_inv();
 * X = PLUS horizontal axis, Y = Height, Z = vertical axis
 * MAX Xaxis 281
 * X, Y, Z, GRIP_DEGREE, GRIPPER_, de
 * 144.9 ~ 144.9 | 205, 0 ~ 0, 205 | Y is 0
 */
bool set_arm_inv(double axisX, double axisY, double axisZ, int GRIP_DEGREE, int GRIPPER_, int de) {
  // θ1 = BASE_ANGLE
  // θ2 = SHOULDER_ANGLE
  // θ3 = ELBOW_ANGLE
  // θ4 = WRIST_ANGLE
  // Φ = GRIP_DEGREE
  double BASE_HEIGHT = 75.0; // l1
  double SHOULDER_LENGTH = 107.0;  // l2
  double ELBOW_LENGTH = 98.0;   // l3
  double WRIST_LENGTH = 83.0;  // l4
  /* bandwidth MAX 205, coordinate 144.9 */

  // if Φ = 0; GRIP_DEGREE = 0
  double GRIP_ANGLE = 0; // GRIP_ANGLE is 0, l4 is parallel to X axis
  // cosθ3 = (X^2+Z^2+Y^2-l2^2-l3^3) / (2*l2*l3)
  double COStheta3 = ( axisX*axisX + axisZ*axisZ + axisY*axisY \
                    - SHOULDER_LENGTH*SHOULDER_LENGTH \
                    - ELBOW_LENGTH*ELBOW_LENGTH \
                    - 2*sqrt(axisX*axisX + axisZ*axisZ)*WRIST_LENGTH + WRIST_LENGTH*WRIST_LENGTH )\
                    / (2*SHOULDER_LENGTH*ELBOW_LENGTH);
  // sinθ3 = +-sqrt((1-(cosθ3)^2)
  double SINtheta3 = sqrt(1-COStheta3*COStheta3); // SINtheta3의 부호결정?

  // Xn = sqrt(X^2+Z^2) - l4*cosΦ
  double axisXn = sqrt((axisX*axisX) + (axisZ*axisZ)) - WRIST_LENGTH*cos(GRIP_ANGLE); // 로봇의 반경x'(R) = sqrt(X^2 + Z^2)
  double axisYn = axisY - WRIST_LENGTH*sin(GRIP_ANGLE);  // -BASE_HEIGHT?
  double axisK1 = SHOULDER_LENGTH + ELBOW_LENGTH*COStheta3;
  double axisK2 = ELBOW_LENGTH*SINtheta3;  // check ELBOW_LENGTH*SINtheta3*COStheta3
  
  double BASE_ANGLE = atan2(axisZ, axisX); // θ1 몸통 각도(BASE_DEGREE) = atan(Z/X)
  // θ2 = atan2(k1*yn-k2*xn, k1*xn-k2*yn)
  double SHOULDER_ANGLE = atan2((axisK1*axisYn - axisK2*axisXn), (axisK1*axisXn - axisK2*axisYn));
  // θ3 = atan2(sinθ3, cosθ3), servo inverted -> Pi - θ3
  double ELBOW_ANGLE = atan2(SINtheta3, COStheta3);
  // θ4 = Φ - θ2 - θ3
  double WRIST_ANGLE = GRIP_ANGLE - SHOULDER_ANGLE - ELBOW_ANGLE;
  if( (SHOULDER_ANGLE < 0) || (SHOULDER_ANGLE > Pi) \
    || (ELBOW_ANGLE < -Pi*3/4) || (ELBOW_ANGLE > Pi/4) \
    || (WRIST_ANGLE < -Pi/2) || (WRIST_ANGLE > Pi/2) ) {
    SINtheta3 = -SINtheta3;
    axisK2 = -axisK2;
    SHOULDER_ANGLE = atan2((axisK1*axisYn - axisK2*axisXn), (axisK1*axisXn - axisK2*axisYn));
    ELBOW_ANGLE = atan2(SINtheta3, COStheta3);
    WRIST_ANGLE = GRIP_ANGLE - SHOULDER_ANGLE - ELBOW_ANGLE;
    #if DEBUG_PRINT
    Serial.println("\n# change SINtheta3 #");
    #endif
  }
  #if DEBUG_PRINT
  Serial.println("# Calculated 6DOF Inverse Kinematics");
  Serial.print("@ X : "); Serial.print(axisX);
  Serial.print(", Y : "); Serial.print(axisY);
  Serial.print(", Z : "); Serial.println(axisZ);
  Serial.print("@ COSθ3 : "); Serial.print(COStheta3);
  Serial.print(", SINθ3 : "); Serial.println(SINtheta3);
  Serial.print("@ Xn : "); Serial.print(axisXn);
  Serial.print(",\tYn : "); Serial.println(axisYn);
  Serial.print("@ K1 : "); Serial.print(axisK1);
  Serial.print(",\tK2 : "); Serial.println(axisK2);
  Serial.println("# 6DOF Inverse Kinematics");
  Serial.print("  BASE_ANGLE \t: "); Serial.println(rad_to_degree(BASE_ANGLE));
  Serial.print("  SHOULDER_ANGLE: "); Serial.println(rad_to_degree(SHOULDER_ANGLE));
  Serial.print("  ELBOW_ANGLE \t: "); Serial.println(rad_to_degree(ELBOW_ANGLE));
  Serial.print("  WRIST_ANGLE \t: "); Serial.println(rad_to_degree(WRIST_ANGLE));
  #endif

  double GRAVITY_ANGLE = SHOULDER_ANGLE + ELBOW_ANGLE;
  
  /* radian to servo angle*/
  BASE_ANGLE = Pi/2 - BASE_ANGLE; 
  SHOULDER_ANGLE = Pi - SHOULDER_ANGLE;
  ELBOW_ANGLE = Pi*3/4 + ELBOW_ANGLE;
  WRIST_ANGLE = Pi/2 + WRIST_ANGLE;

  // running ROBOT-ARM
  BASE_ANGLE = rad_to_degree(BASE_ANGLE);
  SHOULDER_ANGLE = rad_to_degree(SHOULDER_ANGLE);
  ELBOW_ANGLE = rad_to_degree(ELBOW_ANGLE);
  WRIST_ANGLE = rad_to_degree(WRIST_ANGLE);

  // gravity correction
  SHOULDER_ANGLE = SHOULDER_ANGLE - gravityConst*cos(GRAVITY_ANGLE);

  #if DEBUG_PRINT
  Serial.println("# 6DOF MOVING SERVO ANGLE ");
  Serial.print("  BASE_ANGLE \t: "); Serial.println(BASE_ANGLE);
  Serial.print("  SHOULDER_ANGLE: "); Serial.println(SHOULDER_ANGLE);
  Serial.print("  ELBOW_ANGLE \t: "); Serial.println(ELBOW_ANGLE);
  Serial.print("  WRIST_ANGLE \t: "); Serial.println(WRIST_ANGLE);
  #endif

  if (check_value(BASE_ANGLE)) return false;
  if (check_value(SHOULDER_ANGLE)) return false;
  if (check_value(ELBOW_ANGLE)) return false;
  if (check_value(WRIST_ANGLE)) return false;

  int cal_angle[SERVOS_CNT] = {BASE_ANGLE, SHOULDER_ANGLE, ELBOW_ANGLE, WRIST_ANGLE, GRIP_DEGREE, GRIPPER_};
  if ( read_arm_angle(cal_angle) ) {
//    INITANGLE[0] = myservo[0].read();
//    set_arm_delay(INITANGLE, de);
//    // MOVE BASE FIRST
//    while(currentAngle[0] != cal_angle[0]){
//      currentAngle[0] = myservo[0].read();
//      
//      if ( currentAngle[0] < cal_angle[0] ) ++currentAngle[0];
//      else if ( currentAngle[0] > cal_angle[0] ) --currentAngle[0];
//
//      myservo[0].write(currentAngle[0]);
//      delay(de*3);
//    }
    delay(100);
    set_arm_delay(cal_angle, de);
    return true;
  }
  return false;
}

bool read_arm_angle(int set_angle[SERVOS_CNT]){
  int checkSum = 0;
  int checkValue[SERVOS_CNT] = {0,};
  
  for (int i = 0; i < SERVOS_CNT; i++){
    previousAngle[i] = myservo[i].read();
    ++checkValue[i];
    if ( previousAngle[i] == set_angle[i] ) checkValue[i] = 0;
  }

  for(int j = 0 ; j < SERVOS_CNT ; j++) {
    checkSum += checkValue[j];
  }
  if(checkSum == 0) {
    #if DEBUG_PRINT
    Serial.println(" > NO NEED TO MOVE ");
    #endif
    return false;
  }
  return true;
}


void set_arm_joystick() {
  delay(30); // 로봇팔 속도조정을 위한 딜레이
  
    for (int i = 0; i < SERVOS_CNT; i++){
      value[i] = analogRead(ANA[i]);
      currentAngle[i] = myservo[i].read();
    
      if (value[i] > 612) {
        idle[i] = 0;
      
        if (currentAngle[i] < MAX[i]) ++currentAngle[i];
        if (!myservo[i].attached()){
          myservo[i].attach(PIN[i]);
        }
        myservo[i].write(currentAngle[i]);     
      } else if (value[i] < 412) {
      idle[i] = 0;
      if (currentAngle[i] > MIN[i]) --currentAngle[i];
      if (!myservo[i].attached()){
        myservo[i].attach(PIN[i]);
      }
      myservo[i].write(currentAngle[i]);    
    } else {
      ++idle[i];
    }
  }
//  if (idle[5] > 100){ // GRIPPER
//  myservo[5].detach(); //서보모터를 일정시간 사용하지 않으면 연결을 끊어둔다.
//  idle[5] = 0;
//  }
}

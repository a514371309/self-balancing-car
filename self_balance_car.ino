#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  
#define DEBUG 0
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define ZERO_SPEED 65535 //zero speed
#define MAX_ACCEL 7 //max ACCEL
#define MAX_THROTTLE 600 //max throttle 530
#define MAX_STEERING 136 //max steering
#define MAX_TARGET_ANGLE 12 //max target angle 12
#define I2C_SPEED 400000L //I2C speed
#define Gyro_Gain 0.03048//gyro gain
#define Gyro_Scaled(x) x*Gyro_Gain //return initial scaled gyro value every second
#define RAD2GRAD 57.2957795//57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489//0.01745329251994329576923690768489
// default control protocol  
#define KP 0.22 // 0.22        
#define KD 24  // 30 28  26  
#define KP_THROTTLE 0.065  //0.08//0.065
#define KI_THROTTLE 0.05//0.05
// PID parameters for raiseup
#define KP_RAISEUP 0.16 //kp raiseup
#define KD_RAISEUP 40   //kd raiseup
#define KP_THROTTLE_RAISEUP 0  //raiseup speed control kp
#define KI_THROTTLE_RAISEUP 0.0//raiseup speed control ki
#define ITERM_MAX_ERROR 40   // ITERM saturation constant
#define ITERM_MAX 5000
// MPU control/????state 
bool dmpReady = false;  // if setup is true then DMP initialize successfully
uint8_t mpuIntStatus;   // MPU real interupt status bytes
uint8_t devStatus;      // return state for each device operation(0=success!0=fail)
uint16_t packetSize;    // calcalate DMP packet size(18 bytes in our case)
uint16_t fifoCount;     // the current nunmber of bytes in FIFO
uint8_t fifoBuffer[18]; // FIFO storage cache
Quaternion q;
uint8_t loop_counter;       // generate a media 40 Hz loop counter
uint8_t slow_loop_counter;  // a slow loop counter 2HZ
long timer_old;//old timer
long timer_value;//timer value
int debug_counter;//debug counter
float dt;
int lkf;
// 0X68 default class I2C address is Ox68
MPU6050 mpu;
float angle_adjusted;//angle adjusted
float angle_adjusted_Old;//old angle adjusted
float Kp=KP;
float Kd=KD;
float Kp_thr=KP_THROTTLE;//throttle
float Ki_thr=KI_THROTTLE;
float Kp_user=KP;
float Kd_user=KD;
float Kp_thr_user=KP_THROTTLE;
float Ki_thr_user=KI_THROTTLE;
bool newControlParameters = false;//
bool modifing_control_parameters=false;//
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float kkll;
float steering;
float max_throttle = MAX_THROTTLE;//
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
int16_t motor1;
int16_t motor2;
int16_t speed_m[2];           // actual motor speed
uint8_t dir_m[2];             // actual motor direction
int16_t actual_robot_speed;          // actual robot speed
int16_t actual_robot_speed_Old;          //old robot speed
float estimated_speed_filtered;//estimated speed
uint16_t counter_m[2];        // cycle of counter
uint16_t period_m[2][8];      // 
uint8_t period_m_index[2];    //
int freeRam () { //
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
//DMP function
//this function is for weight calculation when sensor fusion
//default value 0x80

void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}
// fast calculation 
float dmpGetPhi() {
   mpu.getFIFOBytes(fifoBuffer, 16); // 
   mpu.dmpGetQuaternion(&q, fifoBuffer); 
   mpu.resetFIFO();  
   //retuen( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
   return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}
// PD function? DT in milisecond
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;
  error = setPoint-input;
  // Kd implemented in two steps
  // only use inout sensor instead of setup
  output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;       // + fault - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // the only input is error in Kd
  setPointOld = setPoint;
  return(output);
}
//P control
float speedPControl(float input, float setPoint,  float Kp)
{
  float error;
  error = setPoint-input;
  return(Kp*error);
}
// PI control, DT in miliseconds.
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;
  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);
  output = Kp*error + Ki*PID_errorSum*DT*0.001;
  return(output);
}
//16MHz 200ns=>4 commands
void delay_200ns()  
{
  __asm__ __volatile__ (
		"nop" "\n\t"
                "nop" "\n\t"
                "nop" "\n\t"
		"nop"); 
}
ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
    {
    counter_m[0] = 0;
    if (period_m[0][0]==ZERO_SPEED)
      return;
    if (dir_m[0])
      SET(PORTB,4);  //DIR motor 1
    else
      CLR(PORTB,4);
    // wait  200ns in case of step pulse
    period_m_index[0] = (period_m_index[0]+1)&0x07; // period M from 0 to 7
    //delay_200ns();
    SET(PORTD,7); // step motor 1
    delayMicroseconds(1);
    CLR(PORTD,7);
    }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
    {
    counter_m[1] = 0;
    if (period_m[1][0]==ZERO_SPEED)
      return;
    if (dir_m[1])
      SET(PORTC,7);   // DIR motor 2
    else
      CLR(PORTC,7);
    period_m_index[1] = (period_m_index[1]+1)&0x07;
    //delay_200ns();
    SET(PORTD,6); // step motor 1
    delayMicroseconds(1);
    CLR(PORTD,6);
    }
}
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;
  
  if (speed_m[motor] == 0)
    {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
    }
  if (speed_m[motor] > 0 )   // postive speed
    {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
    }
  else                       // negative speed
    {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
    }
    
  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed; 
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // optimize code to calcuate subperiod
  if (subperiod>0)
   period_m[motor][1]++;
  if (subperiod>1)
   period_m[motor][5]++;
  if (subperiod>2)
   period_m[motor][3]++;
  if (subperiod>3)
   period_m[motor][7]++;
  if (subperiod>4)
   period_m[motor][0]++;
  if (subperiod>5)
   period_m[motor][4]++;
  if (subperiod>6)
   period_m[motor][2]++; 
}
void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // limit max accel
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;  
  calculateSubperiods(motor);  //use four subperiod to improve resolution
  // save energy when it's not running
  if ((speed_m[0]==0)&&(speed_m[1]==0))
    digitalWrite(4,HIGH);   //stop motor
  else
    digitalWrite(4,LOW);   // run motor
}
void setup() 
{ 
  // ???? 
  pinMode(4,OUTPUT);  // ENABLE MOTORS
  pinMode(12,OUTPUT);  // STEP MOTOR 1 PORTD,7//6
  pinMode(13,OUTPUT);  // DIR MOTOR 1 8
  pinMode(6,OUTPUT); // STEP MOTOR 2 PORTD,6 12
  pinMode(8,OUTPUT); // DIR MOTOR 2 13
  digitalWrite(4,HIGH);   // Disbale motors
  Serial1.begin(9600);
  // add I2C bus
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L/I2C_SPEED)-16)/2;
  TWCR = 1<<TWEN;
  //mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  
  delay(1000);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
        // open DMP, now, it's ready
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        
    }
  // gyro calibration
  //robot must be stable in beginning
  delay(1500);   
  timer_old = millis();
  //step motor initialization
  //timer CTC mode
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);
  // output mode=00(close)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  //setup timer pre-devider
  //normally use a b devider to generate 16 Hz timer
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;
  delay(1000);
  //adjust sensor fusion gain
  dmpSetSensorFusionAccelGain(0x40);//0x20/////////////////////////////////////////////////////////
  delay(1000);
  TIMSK1 |= (1<<OCIE1A);  // timer interupt
  digitalWrite(4,LOW);    // enable step motor drive
  // robot ready
  for (uint8_t k=0;k<3;k++)
	{
	setMotorSpeed(0,3);
	setMotorSpeed(1,-3);
	delay(150);
	setMotorSpeed(0,-3);
	setMotorSpeed(1,3);
	delay(150);
	}  
  mpu.resetFIFO();
  timer_old = millis();
      
}
// main loop//////////////////////////////////////////////////////////////////////////+++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() 
{	   
   kongzhi();
   timer_value = millis();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount>=18)
      {
      if (fifoCount>18)  // if we have at least one packet, we then dump cache zone
        {
        mpu.resetFIFO();
        return;
        }
	loop_counter++;
        slow_loop_counter++;
    dt = (timer_value-timer_old);
    timer_old = timer_value;  
    angle_adjusted_Old = angle_adjusted;
    angle_adjusted = dmpGetPhi();
    mpu.resetFIFO();  // always reset FIFO
    // estimate speed of robot
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_m[1] - speed_m[0])/2;  
    int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*90.0;     
    int16_t estimated_speed = actual_robot_speed_Old - angular_velocity;     // ultilize speed of robot(T-1)or(T-2)to compensate delay
    estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;
    target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr); 
    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);  
    //output accel
    control_output += stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd);	
    control_output = constrain(control_output,-800,800);        
    
    motor1 = control_output + steering;
    motor2 = -control_output + steering;   //motor 2 run in opposite direction    
	// limit max speed
    motor1 = constrain(motor1,-600,600);   
    motor2 = constrain(motor2,-600,600);
     setMotorSpeed(0,motor1);
     setMotorSpeed(1,motor2); 
      }
}
void kongzhi()
{
    lkf = Serial1.read();
  switch(lkf)
  {
  case 'F':
     qian();
     lkf=0;    
     break; 
   case 'B':
    hou();
    lkf=0; 
     break;     
   case 'R':
     zuo();
     lkf=0; 
     break;
   case 'L':
     you();
     lkf=0; 
     break; 
   case 'S':
    ting();
    lkf=0; 
     break;  
  case '0': 
     kkll=0;
     lkf=0; 
     break;
   case '1': 
     kkll=55;
     lkf=0; 
     break;
   case '2': 
    kkll=110;
    lkf=0; 
    break;
   case '3':
     kkll=165;
    lkf=0;
     break;
  case '4':
     kkll=180;
    lkf=0;
     break;
  case '5':
    kkll=200;
    lkf=0;
    break;
  case '6': 
     kkll=240;
    lkf=0;
     break;
   case '7': 
     kkll=280;
    lkf=0;
     break;
   case '8': 
    kkll=320;
    lkf=0;
    break;
   case '9':
     kkll=360;
     lkf=0;
     break;
   }
}
void qian()
{
   throttle=-kkll;
}
void hou()
{
  throttle = 200;
}
void zuo()
{
    steering = -80;
}
void you()
{
    steering = 80;
}
void ting()
{
    throttle = 0;
   steering =0;
}
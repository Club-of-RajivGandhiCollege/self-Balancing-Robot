#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[128];   // FIFO storage buffer


// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector


//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;


//adjust these values to fit your own design
double Kp = 15; //15  
double Kd = 1.76;//1.76
double Ki = 500;//500
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;


//MOTOR CONTROLLER
/*
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 9;
const int IN4 = 8;
const int ENB = 10;
*/

const int ENA = 5;
const int IN1 = 7;
const int IN2 = 6;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);


volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high


void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup()
{
  //Serial.begin(9600);
    
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  
  devStatus = mpu.dmpInitialize();
  

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-1660);
  mpu.setYGyroOffset(150);//-31//100
  mpu.setZGyroOffset(56);
  mpu.setZAccelOffset(1702); // 1688 factory default for my test chip
  
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true; 
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(15);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));//F is used to store string on Flash memory and not RAM
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
}


void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    setPIDValues();
    
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
    //Serial.println(output);
  }

  motorController.move(0, MIN_ABS_SPEED);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
  }
}

float convert(String val)
{
  int i,len,decimal=0;
  double sum=0;
for(i=0;val[i]!='\0';i++)
     {
      if(val[i]=='.')
      decimal++;
      }
     len=i;
     if(decimal==0)
     {
      for(i=len-1;i>=0;i--){
      sum=sum+(val[i]-48)*pow(10,len-1-i);
     }
     }
     else{
          sum=floatConvert(val);
      }
     return sum;   
}

float floatConvert(String val){
    int i,len,len1,decimal=0;
    double sum=0;
    String dec;
  for(i=0;val[i]!='.';i++)
     {}
  len=i;
  
  for(i=len-1;i>=0;i--){
    sum=sum+(val[i]-48)*pow(10,len-1-i);
    }
    
  for(i=len+1;val[i]!='\0';i++)
     {}
  len1=i-(len+1);

  
  int p=0;
  for(i=len+1;val[i]!='\0';i++){

    sum=sum+(val[i]-48)*(1/pow(10,p+1));
    p++;
  }
  
  //Serial.print("decimal value-----");
  //Serial.println(dec);
 
  for(i=0;dec[i]!='\0';i++){
      sum=sum+(val[i]-48)*(1/pow(10,i+1));
    }
    
    return sum;    
}

void setPIDValues()
{
  if(Serial.available()>0)
  {
    //Serial.println("Hello");
    String btVal = Serial.readString();
    //Serial.println(btVal);
    double val = (double)convert(btVal);
   
   Serial.println(val);
    
    if(val>=3000 && val<4000)
    {
      val=val-3000;
      Kd=val;
      pid.SetTunings(Kp,Ki,Kd);
      //Serial.println("################################");
    }
    else if(val>=4000)
    {
      val = val-4000;
      Ki=val;
      pid.SetTunings(Kp,Ki,Kd);
      //Serial.println("Here***************************");
    }
    else
    {
      Kp=val;
      pid.SetTunings(Kp,Ki,Kd);
      //Serial.println("______________________________________");
    }
  }

}

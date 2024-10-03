/* KASIRGA AMFIBIC UNMANNED AIR VEHICLE
 * Project for Design Competition with Xsens Modules. 
 * Gazi University - Cem Avci & Berat Semercioglu
 * Author: Cem Avci, cemavci97@hotmail.com
 * Date: 20/11/2020
 */


/* Library Imports */
#include <XSens.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <math.h>
#include <TimerOne.h>


/* Servo devices */
Servo throttle;
Servo aileron;
Servo elevator;
Servo rudder;

/* Receiver Pin Connections */
#define CH1 3 //aileron
#define CH2 7 //elevator
#define CH3 5 //throttle
#define CH4 4 //rudder
#define CH10 6 //control switch

/* Initialize Xsens at 0x6b */
XSens xsens(0x6b);

RF24 Transmitter(9, 10);
const byte address[6] = {"00001"};     

struct MyStruct {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float akaliRoll;
  float akaliPic;
  float yaw;
  //float ch3;
};

MyStruct kasirgaData;

/* PID Params */

  float errorpitch,errorroll,pwmpitch, pwmroll,PIDpitch,PIDroll, previous_errorpitch,previous_errorroll;
  float pidpitch_p=0;
  float pidpitch_i=0;
  float pidpitch_d=0;
  
  float pidroll_p=0;
  float pidroll_i=0;
  float pidroll_d=0;
  
  /////////////////PID CONSTANTS/////////////////
  double kp=3;//3.55
  double ki=0.02;//0.003
  double kd=0.02;//2.05
  ///////////////////////////////////////////////
  
  float pitch;
  float roll;
  //float yaw;
  
  short desiredAnglePitch=0;
  short desiredAngleRoll=0;


  float timePrev;
  float elapsedTime;
  float time;
  float lastRecvTime;

  float ch1,ch2,ch3,ch4;

  int flag=0;
  char gcs="";
  


void setup() {
  
  Serial.begin(115200);
  throttle.attach(A0);
  aileron.attach(A1);
  elevator.attach(A2);
  rudder.attach(A3);
  Wire.begin();
  xsens.begin();

  Transmitter.begin(); 
  //Transmitter.openReadingPipe(0, address); 
  Transmitter.openWritingPipe(address); 
  Transmitter.setPALevel(RF24_PA_MIN); 
  Transmitter.setChannel(87); 
  Transmitter.setDataRate(RF24_2MBPS); 
  Transmitter.stopListening();  
  //Timer1.initialize(1000000);
  //Timer1.attachInterrupt(RX);
 
}

void xsensIMU(){
  
  //Read mesurements (Reads all data given by the AHRS)
  xsens.updateMeasures();
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
  
  if(isnan(xsens.getAccel()[2])==0){
     roll=(atan2(xsens.getAccel()[1],xsens.getAccel()[2])+3.14)*57.2957795;
     pitch=(atan2(xsens.getAccel()[0],xsens.getAccel()[2])+3.14)*57.2957795;
     kasirgaData.yaw=xsens.getHeadingYaw()*57.2957795;

     kasirgaData.accX=xsens.getAccel()[0];
     kasirgaData.accY=xsens.getAccel()[1];
     kasirgaData.accZ=xsens.getAccel()[2];

     kasirgaData.gyroX=xsens.getRot()[0]*57.29557795;
     kasirgaData.gyroY=xsens.getRot()[1]*57.29557795;
     kasirgaData.gyroZ=xsens.getRot()[2]*57.29557795;
     
     roll=roll-180;
     pitch=pitch-180;


     kasirgaData.akaliRoll = 0.5 *(kasirgaData.akaliRoll+(xsens.getRot()[0]*57.29557795*elapsedTime)) + 0.5*roll;
     
     kasirgaData.akaliPic = 0.5 *(kasirgaData.akaliPic+(xsens.getRot()[1]*57.29557795*elapsedTime)) + 0.5*pitch;   }
    /*
    Serial.print(kasirgaData.accX);
    Serial.print(",");
    Serial.print(kasirgaData.accY);
    Serial.print(",");
    Serial.print(kasirgaData.accZ);
    Serial.print(",");
    Serial.print(kasirgaData.gyroX);
    Serial.print(",");
    Serial.print(kasirgaData.gyroY);
    Serial.print(",");
    Serial.print(kasirgaData.gyroZ);
    Serial.print(",");
    Serial.print(kasirgaData.akaliRoll);
    Serial.print(",");
    Serial.println(kasirgaData.akaliPic);
    */
    delayMicroseconds(300);
    
     }
   

void loop() {

  xsensIMU();
  
  ch1=pulseIn(CH1,HIGH);
  ch2=pulseIn(CH2,HIGH);
  ch3=pulseIn(CH3,HIGH);
  ch4=pulseIn(CH4,HIGH);

  ch1=map(ch1,1000,2000,150,30);
  ch2=map(ch2,1000,2000,30,150);
  ch4=map(ch4,1000,2000,30,150);
  //kasirgaData.ch3=ch3/10.;

  Transmitter.write(&kasirgaData, sizeof(kasirgaData));
  //Serial.print(gcs);

  
  if (digitalRead(CH10)==0){
  //Serial.println("BERATOMOD");
  aileron.write(ch1);
  elevator.write(ch2);
  rudder.write(ch4);
  throttle.writeMicroseconds(ch3);
  delayMicroseconds(100);
  }

  else{
    //Serial.println("PID ZAMANI");
    //throttle.writeMicroseconds(1000);
    pid();
    
    
    }
}

void pid(){
  /*
  Serial.print("akaliPic : ");
  Serial.print(akaliPic);
  Serial.print("akali ROll: ");
  Serial.println(akaliRoll);
  */
  errorpitch = kasirgaData.akaliPic - desiredAnglePitch;
  errorroll = kasirgaData.akaliRoll- desiredAngleRoll;
  
      
  /*Next the proportional value of the PID is just a proportional constant
  *multiplied by the error*/
  
  pidpitch_p = kp*errorpitch;
  pidroll_p=kp*errorroll;
  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  if(-3 <errorpitch <3)
  {
    pidpitch_i = pidpitch_i+(ki*errorpitch);  
  }
  
  if(-3 <errorroll <3)
  {
    pidroll_i = pidroll_i+(ki*errorroll);  
  }
  
  /*The last part is the derivate. The derivate acts upon the speed of the error.
  As we know the speed is the amount of error that produced in a certain amount of
  time divided by that time. For taht we will use a variable called previous_error.
  We substract that value from the actual error and divide all by the elapsed time. 
  Finnaly we multiply the result by the derivate constant*/
  
  pidpitch_d = kd*((errorpitch - previous_errorpitch)/elapsedTime);
  
  pidroll_d = kd*((errorroll - previous_errorroll)/elapsedTime);
  
  /*The final PID values is the sum of each of this 3 parts*/
  PIDpitch = pidpitch_p + pidpitch_i + pidpitch_d;
  
  PIDroll = pidroll_p + pidroll_i + pidroll_d;
  
  
  
  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
  tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
  have a value of 2000us the maximum value taht we could sybstract is 1000 and when
  we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
  to reach the maximum 2000us*/
  
  
  /*
  if(PIDpitch < 30)
  {
    PIDpitch=30;
  }
  if(PIDpitch > 150)
  {
    PIDpitch=150;
  }
  
  if(PIDroll < 30)
  {
    PIDroll=30;
  }
  if(PIDroll > 150)
  {
    PIDroll=150;
  }
  
  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/

  pwmpitch = (ch2+ PIDpitch); //72
  pwmroll = ( ch1+PIDroll);   //85
/*
  Serial.print("AKALI PIC: ");
  Serial.print(akaliPic);
  Serial.print(" , ");
  Serial.print("AKALI ROLL: ");
  Serial.print(akaliRoll);
  Serial.print(" , ");
  Serial.print("pwm PIC: ");
  Serial.print(pwmpitch);
  Serial.print(" , ");
  Serial.print("pwm ROLL: ");
  Serial.println(pwmroll);
  delay(150);
*/
  
  /*Once again we map the PWM values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for 
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/

 
  //Right
  if(pwmpitch < 30)
  {
    pwmpitch= 30;
  }
  if(pwmpitch > 150)
  {
    pwmpitch=150;
  }
  //Left
  if(pwmroll < 30)
  {
    pwmroll= 30;
  }
  if(pwmroll > 150)
  {
    pwmroll=150;
  }



  /*
  Serial.print(pwmpitch);
  Serial.print(" , ");
  Serial.println(pwmroll);
  */
  
  throttle.writeMicroseconds(ch3);
  elevator.write(pwmpitch);
  rudder.write(ch4);
  aileron.write(pwmroll);
  delayMicroseconds(100);

}
/*
void RX(){
  Transmitter.startListening();
   if (Transmitter.available()) { 
    Transmitter.read(&gcs, sizeof(gcs));
    Transmitter.stopListening();
  }
  else{
    Transmitter.stopListening();
    }
}
*/

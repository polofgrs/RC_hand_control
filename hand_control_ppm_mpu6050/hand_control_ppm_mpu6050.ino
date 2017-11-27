/*Hand control for RC aircrafts with the FrSky protocols and MPU6050.
 * Code written by Paul Faugeras - <polofgrs@gmail.com>
 * PPM encoding code inspired by https://code.google.com/archive/p/generate-ppm-signal/
 * MPU6050 code inspired by Jeff Rowberg code : https://github.com/jrowberg/i2cdevlib <jeff@rowberg.net>
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//////////////////////PPM CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 4  //set the number of channels
#define AIL_CH 0
#define PIT_CH 1
#define THR_CH 2
#define YAW_CH 3
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define CHANNEL_MIN_VALUE 980
#define CHANNEL_MAX_VALUE 2020
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino
#define thr_pot 6 //signal from throttle potentiometer pin

int const ail_ang = 60; //angle that corresponds to max value (full stick)
int const pit_ang = 60;
int const yaw_ang = 60;

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

void setup()
{
  Serial.begin(115200);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //Serial.begin(115200);
    while (!Serial);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    pinMode(LED_PIN, OUTPUT);

  //initialize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++)
  {
      if (i!=THR_CH)
      {
        ppm[i]= CHANNEL_DEFAULT_VALUE;
      }
  }
  ppm[THR_CH]=CHANNEL_MIN_VALUE; //set throttle ppm value to 0
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

int premiere_it=0;
bool yaw_offset=false;
double initial_yaw;

void loop()
{
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //wait for the neutral yaw value to initialize
        if (premiere_it<1500)
        {
          initial_yaw=ypr[0]*180/M_PI;
          yaw_offset=(abs(initial_yaw)>135);
        }

        //yaw value treatment
        double yaw_deg = ypr[0]*180/M_PI;
        double yaw_deg_diff = yaw_deg - initial_yaw;

        //avoid yaw jumps from -180 to 180 deg
        if (yaw_offset)
        {
          if (initial_yaw>180-yaw_ang && yaw_deg<0)
          {
            yaw_deg_diff = 358 + yaw_deg - initial_yaw;
          }
          if (initial_yaw<-(180-yaw_ang) && yaw_deg>0)
          {
            yaw_deg_diff = 358 + initial_yaw - yaw_deg;
          }
        }

        //obtain and constrain angle values to desired intervals
        double yaw_deg_diff_constrain=constrain(-yaw_deg_diff,-yaw_ang,yaw_ang);
        double thr_1024=analogRead(thr_pot);
        double pitch_deg=constrain(-ypr[1]*180/M_PI,-pit_ang,pit_ang);
        double ail_deg=constrain(-ypr[2]*180/M_PI,-ail_ang,ail_ang);

        //maps degree angles to ppm values
        int thr_ppm=map(thr_1024,0,1023,CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
        int pitch_ppm=map(pitch_deg,-pit_ang,pit_ang,CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
        int ail_ppm=map(ail_deg,-ail_ang,ail_ang,CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
        int yaw_ppm=map(yaw_deg_diff_constrain,-yaw_ang,yaw_ang,CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);

        //updates ppm table to transmit
        ppm[THR_CH]=thr_ppm;
        ppm[PIT_CH]=pitch_ppm;
        ppm[AIL_CH]=ail_ppm;
        ppm[YAW_CH]=yaw_ppm;

        //debugging
        Serial.print(thr_ppm);
        Serial.print("\t");
        Serial.print(pitch_ppm);
        Serial.print("\t");
        Serial.print(ail_ppm);
        Serial.print("\t");
        Serial.println(yaw_ppm);

        //increment the iteration variable
        premiere_it++;
    }
}


ISR(TIMER1_COMPA_vect) //ppm transmission program
{
  static boolean state = true;
  TCNT1 = 0;
  if (state) //start pulse
  {
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  }
  else //end pulse and calculate when to start the next pulse
  {
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if(cur_chan_numb >= CHANNEL_NUMBER)
    {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

/*
@name: Drone Control Code
@version: 1.4
@description: This code reads UDP packages over LAN and decodes them using OSC formatted messages of size 12 to control the quad-copter. It also uses the values obtained from the MPU to auto-balance the drone while controlling it through the PCA's PWM signals.
@project:Edison Drone
@team:Segmentation Fault
@authors:Matt Almenshad, David Zimmerly, Jacob Lin, Joseph McLaughlin
@instructor: Boyana Norris
@last-update:03/11/2016
@notes: We used code from the MPU6050 library examples and the PID library examples under MIT licence
*/

/*[[-- Includes --]]*/
#include <PID_v1.h> // Proportional Integral Derivative

#include "I2Cdev.h" // Communicate with I2C devices
#include "Wire.h" // Communicate with external devices

#include <Adafruit_PWMServoDriver.h> // PWM generator with 12-bits of resolution
#include "MPU6050_6Axis_MotionApps20.h" // Motion Processor Unit 6050 control code

#include <EthernetUdp.h>        // UDP library to receive UDP packages

/*[[-- MPU --]]*/ //partially from MPU example sketch
MPU6050 m; // Instance of the MPU6050 driver
float ypr[3];  // float arrMPU_Calibration_ay to store yaw, pitch and roll values
Quaternion q;           // [w, x, y, z]         quaternion container
uint16_t packetSize2;    // expected DMP packet size(default = 42 bytes)
uint16_t fifoCount;     // count / bytes currently in FIFO
uint8_t fifoBuf[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            gravity vector

// Calibration
int MPU_Calibration_buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int MPU_Calibration_acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch mMPU_Calibration_ay not converge  (default:8)
int MPU_Calibration_giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch mMPU_Calibration_ay not converge  (default:1)

int16_t MPU_Calibration_ax, MPU_Calibration_ay, MPU_Calibration_az, MPU_Calibration_gx, MPU_Calibration_gy, MPU_Calibration_gz;

int mean_MPU_Calibration_ax, mean_MPU_Calibration_ay, mean_MPU_Calibration_az, mean_MPU_Calibration_gx, mean_MPU_Calibration_gy, mean_MPU_Calibration_gz, MPU_Calibration_state=0;
int MPU_Calibration_ax_offset, MPU_Calibration_ay_offset, MPU_Calibration_az_offset, MPU_Calibration_gx_offset, MPU_Calibration_gy_offset, MPU_Calibration_gz_offset;
// END CALIBRATION

double MPU_Yaw, MPU_Pitch, MPU_Roll;

/*[[-- PWM --]]*/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // an instance of PWM generator

#define FL_CHANNEL 0   // Front-Left motor channel
#define FR_CHANNEL 1  // Front-Right motor channel
#define BL_CHANNEL 2 // Back-Left motor channel
#define BR_CHANNEL 3// Back-Right motor channel

#define SERVOMIN  205 // minimum pulse length out of 4096: 205 = 5% duty
#define SERVOMAX  410 // MAXimum pulse length out of 4096: 410 = 10% duty

int escFRVal = SERVOMIN;   // uS value for front right ESC
int escFLVal = SERVOMIN;  // uS value for front left ESC
int escBRVal = SERVOMIN; // uS value for back right ESC
int escBLVal = SERVOMIN;// uS value for back left ESC

int throttle = SERVOMIN; // PWM throttle

/*[[-- UDP --]]*/
EthernetUDP Udp; // An EthernetUDP instance to send and receive packets over UDP

unsigned int localPort = 8888;      // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet of size UDP_TX_PACKET_MAX_SIZE

/*[[-- PID --]]*/ // Partially from PID libraray
//Define Variables we'll be connecting to
double PID_PITCH_setPoint, PID_ROLL_setPoint, PID_YAW_setPoint, PID_PITCH_Output, PID_ROLL_Output, PID_YAW_Output;
//Specify the links and initial tuning parameters
double Kp_PITCH=1.8, Ki_PITCH=0, Kd_PITCH=0;
double Kp_ROLL=Kp_PITCH, Ki_ROLL=Ki_PITCH, Kd_ROLL=Kd_PITCH;
double Kp_YAW=0.1, Ki_YAW=0, Kd_YAW=1;

PID PID_PITCH;// Instance to compute Pitch adjustment
PID PID_ROLL;//Instance to compute Roll adjustment
PID PID_YAW;//Instance to compute Yaw adjustment

void setup() {
  // Turn on LED indicating the start of setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // start Serial
  Serial.begin(115200);

  // init udp
  Serial.println("Initializing Udp..");
  Udp.begin(localPort); //initialize udp to listen to localport
  Serial.print("Udp started on port: ");
  Serial.println(localPort);

  // init pid
  PID_YAW.PIDsetup(&MPU_Yaw, &PID_YAW_Output, &PID_YAW_setPoint, Kp_YAW, Ki_YAW, Kd_YAW, DIRECT);
  PID_PITCH.PIDsetup(&MPU_Pitch, &PID_PITCH_Output, &PID_PITCH_setPoint, Kp_PITCH, Ki_PITCH, Kd_PITCH, DIRECT);
  PID_ROLL.PIDsetup(&MPU_Roll, &PID_ROLL_Output, &PID_ROLL_setPoint, Kp_ROLL, Ki_ROLL, Kd_ROLL, DIRECT);
  Serial.print("PID initiated");

  PID_PITCH_setPoint = 0; //target angle
  PID_ROLL_setPoint = 0;
  PID_YAW_Output = 0;

  //turn the PID on
  PID_PITCH.SetMode(AUTOMATIC);
  PID_ROLL.SetMode(AUTOMATIC);

  PID_PITCH.SetOutputLimits(-20, 20);// (min, max) throttle adjustment
  PID_ROLL.SetOutputLimits(-20, 20);

  // wire init
  Wire.begin();

  // pwm init
  pwm.begin();
  pwm.setPWMFreq(50); // set pwm period to 20ms

        // calibrating ESCs - send max
  pwm.setPWM(FR_CHANNEL, 0, SERVOMAX);  //410 = 10% OF 4096
  pwm.setPWM(FL_CHANNEL, 0, SERVOMAX);  //410 = 10% OF 4096
  pwm.setPWM(BR_CHANNEL, 0, SERVOMAX);  //410 = 10% OF 4096
  pwm.setPWM(BL_CHANNEL, 0, SERVOMAX);  //410 = 10% OF 4096
  Serial.println("Sent Maximum Signal: 10%");
  delay(2000); // wait 2 sec

        // calibrating ESCs - send mix
  pwm.setPWM(FR_CHANNEL, 0, SERVOMIN);  //205 = 5% OF 4096
  pwm.setPWM(FL_CHANNEL, 0, SERVOMIN);  //205 = 5% OF 4096
  pwm.setPWM(BR_CHANNEL, 0, SERVOMIN);  //205 = 5% OF 4096
  pwm.setPWM(BL_CHANNEL, 0, SERVOMIN);  //205 = 5% OF 4096

  Serial.println("Sent Minimum Signal: 5%");
  delay(2000);

  Serial.println("ESC(s) calibrated.");

  //mpu
  m.initialize();
      // MPU calibration (from MPU library)
          // start message
  Serial.println("\nMPU6050 Calibration started");
  delay(2000);
  Serial.println("\n MPU6050 should be placed in horizontal position.\nDon't touch it until you see a finish message.\n");
  delay(3000);
          // verify connection
  Serial.println(m.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
          // reset offsets
  m.setXAccelOffset(0);
  m.setYAccelOffset(0);
  m.setZAccelOffset(0);
  m.setXGyroOffset(0);
  m.setYGyroOffset(0);
  m.setZGyroOffset(0);

  while (MPU_Calibration_state!=3){
      if (MPU_Calibration_state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    MPU_Calibration_state++;
    delay(1000);
  }

  if (MPU_Calibration_state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    MPU_Calibration_state++;
    delay(1000);
  }

  if (MPU_Calibration_state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_MPU_Calibration_ax); 
    Serial.print("\t");
    Serial.print(mean_MPU_Calibration_ay); 
    Serial.print("\t");
    Serial.print(mean_MPU_Calibration_az); 
    Serial.print("\t");
    Serial.print(mean_MPU_Calibration_gx); 
    Serial.print("\t");
    Serial.print(mean_MPU_Calibration_gy); 
    Serial.print("\t");
    Serial.println(mean_MPU_Calibration_gz);
    Serial.print("Offsets:\t");
    Serial.print(MPU_Calibration_ax_offset); 
    Serial.print("\t");
    Serial.print(MPU_Calibration_ay_offset); 
    Serial.print("\t");
    Serial.print(MPU_Calibration_az_offset); 
    Serial.print("\t");
    Serial.print(MPU_Calibration_gx_offset); 
    Serial.print("\t");
    Serial.print(MPU_Calibration_gy_offset); 
    Serial.print("\t");
    Serial.println(MPU_Calibration_gz_offset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    MPU_Calibration_state++;
    }
  }
    // integrate offsets into MPU instance
  m.dmpInitialize(); 
  m.setXAccelOffset(MPU_Calibration_ax_offset);
  m.setYAccelOffset(MPU_Calibration_ay_offset);
  m.setZAccelOffset(MPU_Calibration_az_offset);
  m.setXGyroOffset(MPU_Calibration_gx_offset);
  m.setYGyroOffset(MPU_Calibration_gy_offset);
  m.setZGyroOffset(MPU_Calibration_gz_offset);
  m.setDMPEnabled(true);
  packetSize2 = m.dmpGetFIFOPacketSize();

  //turn off Edison LED to indicate [[END OF CALIBRATION]] 
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
}

void loop() {

  /*[[-- Reiveing Control --]]*/
  
  int packetSize = Udp.parsePacket(); //when you parse a packet the function returns its size, save it
  if (packetSize) {//if there is data in the packet
    Serial.print("Received packet of size ");
    Serial.println(packetSize);//print the size

    // Decode packet and send it to motors
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);//read the packet of size UDP_TX_PACKET_MAX_SIZE into packet buffer
    Serial.println("Decoded Content:");
      int decodedVal = decodeValue(packetBuffer); // translate OSC's little Endian to big Endian
      Serial.print("Command:");
      Serial.println(packetBuffer[1]);
      Serial.print("Value:");
      Serial.println(decodedVal);
      switch(packetBuffer[1]){
        case 'f':
          throttle = decodedVal; //throttle
          Serial.print("Frequency was set to: ");
          Serial.println(decodedVal);    
        break;

        case 'o':
          if (decodedVal == 1){
            throttle = SERVOMIN; //set front right ESC signal
            Serial.println("Frequency was set to minimum");
          }
        break;

        case 'm':
          if (decodedVal == 1){
            throttle = SERVOMAX; //set front right ESC signal
            Serial.println("Frequency was set to maximum");
          }
        break;

        default:
        break;
      }
  }
  while (fifoCount < packetSize2) fifoCount = m.getFIFOCount(); // poll the MPU data (42 bytes)
   fifoCount = m.getFIFOCount(); 
   m.getFIFOBytes(fifoBuf, packetSize2); // get polled data of size 42 byte
   fifoCount -= packetSize2;
   m.dmpGetQuaternion(&q, fifoBuf); 
   m.dmpGetGravity(&gravity, &q);
   m.dmpGetYawPitchRoll(ypr, &q, &gravity);
   MPU_Yaw = (double) (ypr[0]  * 180/M_PI); // translate the angle into readable degrees
   MPU_Pitch = (double) (ypr[1]  * 180/M_PI);
   MPU_Roll = (double) (ypr[2]  * 180/M_PI);
   
   Serial.print("ypr\t");
   Serial.print(ypr[0] * 180/M_PI);
   Serial.print("\t");
   Serial.print(ypr[1] * 180/M_PI);
   Serial.print("\t");
   Serial.print(ypr[2] * 180/M_PI); //uncomment for mpu ypr readings via serial

    // limit throttle bonds 
    
    if(throttle > SERVOMAX){
      throttle = SERVOMAX;
    }

    if(throttle < SERVOMIN){
      throttle = SERVOMIN;
    }

    //cast PID with proper rounding into an int
   int _PID_ROLL_Output = (int)(PID_ROLL_Output), _PID_PITCH_Output = (int)(PID_PITCH_Output);

    if (PID_PITCH_Output > 0){
      _PID_PITCH_Output = (int)(PID_PITCH_Output + 0.5);
    } else {
      _PID_PITCH_Output = (int)(PID_PITCH_Output - 0.5);
    }

    if (PID_ROLL_Output > 0){
      _PID_ROLL_Output = (int)(PID_ROLL_Output + 0.5);
    } else {
      _PID_ROLL_Output = (int)(PID_ROLL_Output - 0.5);
    }

   // calculate adjusted throttle
   escFRVal = throttle - ((int) _PID_PITCH_Output) + ((int) _PID_ROLL_Output);// - PID_YAW_Output; //Calculate the pulse for esc 1 (front-right - CCW)
   escBRVal = throttle + ((int) _PID_PITCH_Output) + ((int) _PID_ROLL_Output);// + PID_YAW_Output; //Calculate the pulse for esc 2 (rear-right - CW)
   escBLVal = throttle + ((int) _PID_PITCH_Output) - ((int) _PID_ROLL_Output);// - PID_YAW_Output; //Calculate the pulse for esc 3 (rear-left - CCW)
   escFLVal = throttle - ((int) _PID_PITCH_Output) - ((int) _PID_ROLL_Output);// + PID_YAW_Output; //Calculate the pulse for esc 4 (front-left - CW)

    // reassert bounds for all values
    if(escFRVal > SERVOMAX){
      escFRVal = SERVOMAX;
    }

    if(escFRVal < SERVOMIN){
      escFRVal = SERVOMIN;
    }

    if(escFLVal > SERVOMAX){
      escFLVal = SERVOMAX;
    }

    if(escFLVal < SERVOMIN){
      escFLVal = SERVOMIN;
    }

    if(escBLVal > SERVOMAX){
      escBLVal = SERVOMAX;
    }

    if(escBLVal < SERVOMIN){
      escBLVal = SERVOMIN;
    }

    if(escBRVal > SERVOMAX){
      escBRVal = SERVOMAX;
    }

    if(escBRVal < SERVOMIN){
      escBRVal = SERVOMIN;
    }


  // set PWM signals
  pwm.setPWM(FR_CHANNEL, 0, escFRVal); //set minimum front right ESC signal
  Serial.print("\tPWM:\t");
  Serial.print(escFLVal);
  pwm.setPWM(FL_CHANNEL, 0, escFLVal); //set minimum front left ESC signal
  Serial.print("\t");
  Serial.print(escFRVal);
  pwm.setPWM(BR_CHANNEL, 0, escBRVal); //set minimum back right ESC signal
  Serial.print("\t");
  Serial.print(escBLVal);
  pwm.setPWM(BL_CHANNEL, 0, escBLVal); //set minimum back left ESC signal
  Serial.print("\t");
  Serial.print(escBRVal);
  Serial.print("\t");
  // computer the next PID adjustment
  PID_PITCH.Compute();
  PID_ROLL.Compute();
  Serial.print("\tPID:\t");
  Serial.print(PID_PITCH_Output);
  Serial.print("\t");
  Serial.println(PID_ROLL_Output);
  
}


//////////////////////////////////////////////CALIBRATION FUNCTIONS////////////////////////////////////////////////////
void meansensors(){
  long i=0,buff_MPU_Calibration_ax=0,buff_MPU_Calibration_ay=0,buff_MPU_Calibration_az=0,buff_MPU_Calibration_gx=0,buff_MPU_Calibration_gy=0,buff_MPU_Calibration_gz=0;

  while (i<(MPU_Calibration_buffersize+101)){
    // read raw accel/MPU_Calibration_gyro measurements from device
    m.getMotion6(&MPU_Calibration_ax, &MPU_Calibration_ay, &MPU_Calibration_az, &MPU_Calibration_gx, &MPU_Calibration_gy, &MPU_Calibration_gz);
    
    if (i>100 && i<=(MPU_Calibration_buffersize+100)){ //First 100 measures are discarded
      buff_MPU_Calibration_ax=buff_MPU_Calibration_ax+MPU_Calibration_ax;
      buff_MPU_Calibration_ay=buff_MPU_Calibration_ay+MPU_Calibration_ay;
      buff_MPU_Calibration_az=buff_MPU_Calibration_az+MPU_Calibration_az;
      buff_MPU_Calibration_gx=buff_MPU_Calibration_gx+MPU_Calibration_gx;
      buff_MPU_Calibration_gy=buff_MPU_Calibration_gy+MPU_Calibration_gy;
      buff_MPU_Calibration_gz=buff_MPU_Calibration_gz+MPU_Calibration_gz;
    }
    if (i==(MPU_Calibration_buffersize+100)){
      mean_MPU_Calibration_ax=buff_MPU_Calibration_ax/MPU_Calibration_buffersize;
      mean_MPU_Calibration_ay=buff_MPU_Calibration_ay/MPU_Calibration_buffersize;
      mean_MPU_Calibration_az=buff_MPU_Calibration_az/MPU_Calibration_buffersize;
      mean_MPU_Calibration_gx=buff_MPU_Calibration_gx/MPU_Calibration_buffersize;
      mean_MPU_Calibration_gy=buff_MPU_Calibration_gy/MPU_Calibration_buffersize;
      mean_MPU_Calibration_gz=buff_MPU_Calibration_gz/MPU_Calibration_buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  MPU_Calibration_ax_offset=-mean_MPU_Calibration_ax/8;
  MPU_Calibration_ay_offset=-mean_MPU_Calibration_ay/8;
  MPU_Calibration_az_offset=(16384-mean_MPU_Calibration_az)/8;

  MPU_Calibration_gx_offset=-mean_MPU_Calibration_gx/4;
  MPU_Calibration_gy_offset=-mean_MPU_Calibration_gy/4;
  MPU_Calibration_gz_offset=-mean_MPU_Calibration_gz/4;
  while (1){
    int ready=0;
    m.setXAccelOffset(MPU_Calibration_ax_offset);
    m.setYAccelOffset(MPU_Calibration_ay_offset);
    m.setZAccelOffset(MPU_Calibration_az_offset);

    m.setXGyroOffset(MPU_Calibration_gx_offset);
    m.setYGyroOffset(MPU_Calibration_gy_offset);
    m.setZGyroOffset(MPU_Calibration_gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_MPU_Calibration_ax)<=MPU_Calibration_acel_deadzone) ready++;
    else MPU_Calibration_ax_offset=MPU_Calibration_ax_offset-mean_MPU_Calibration_ax/MPU_Calibration_acel_deadzone;

    if (abs(mean_MPU_Calibration_ay)<=MPU_Calibration_acel_deadzone) ready++;
    else MPU_Calibration_ay_offset=MPU_Calibration_ay_offset-mean_MPU_Calibration_ay/MPU_Calibration_acel_deadzone;

    if (abs(16384-mean_MPU_Calibration_az)<=MPU_Calibration_acel_deadzone) ready++;
    else MPU_Calibration_az_offset=MPU_Calibration_az_offset+(16384-mean_MPU_Calibration_az)/MPU_Calibration_acel_deadzone;

    if (abs(mean_MPU_Calibration_gx)<=MPU_Calibration_giro_deadzone) ready++;
    else MPU_Calibration_gx_offset=MPU_Calibration_gx_offset-mean_MPU_Calibration_gx/(MPU_Calibration_giro_deadzone+1);

    if (abs(mean_MPU_Calibration_gy)<=MPU_Calibration_giro_deadzone) ready++;
    else MPU_Calibration_gy_offset=MPU_Calibration_gy_offset-mean_MPU_Calibration_gy/(MPU_Calibration_giro_deadzone+1);

    if (abs(mean_MPU_Calibration_gz)<=MPU_Calibration_giro_deadzone) ready++;
    else MPU_Calibration_gz_offset=MPU_Calibration_gz_offset-mean_MPU_Calibration_gz/(MPU_Calibration_giro_deadzone+1);

    if (ready==6) break;
  }
}

// OSC

int decodeValue(char *packet){


    union {
        float f;
        uint8_t b[4];
    } u;
    char rData[4];
    rData[0] = packet[11];
    rData[1] = packet[10];
    rData[2] = packet[9];
    rData[3] = packet[8];
    memcpy(u.b, rData, 4);

    return (int) (u.f+0.5);

}


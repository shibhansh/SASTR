/***************************************************************************************************************
* Razor AHRS Firmware v1.4.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
*   * Add binary output of unfused sensor data for all 9 axes.
***************************************************************************************************************/

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 20  // in milliseconds

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
/*#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)*/
float GYRO_X_OFFSET=0.0;
float GYRO_Y_OFFSET=0.0;
float GYRO_Z_OFFSET=0.0;

// Altymeter
#define ALT_SEA_LEVEL_PRESSURE 102133

/*
// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)
*/

#include <Wire.h>
//these variables and libraries are for starting and gatting data from GPS
#include <TinyGPS.h>
#include<SoftwareSerial.h>

TinyGPS gps;
SoftwareSerial Serialg(10,11);
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);
//these will store the data we are getting from gps
float latitude;
float longitude;
//these are the coefficents of the current line on which we are going to move i.e. latcoff*latitude+longcoff*longitue+constant=0
float latcoff,longcoff,constant;
//these are our starting and next points namely waypoints
float prevlat,prevlong,nextlat,nextlong;

// Sensor calibration scale and offset values
/*#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))*/

/*#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))*/

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.6f)
#define Ki_ROLLPITCH (0.02f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// RAW sensor data
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float Ainv[3][3]={
                  {0.039272,.000037,-0.000133},
                  {0.000037,0.038853,-0.000037},
                  {-0.000133,-0.000037,0.039396},
                 };
float acc_bias[3][1]={-11.151513,-15.398577,-10.883802};
float ACCEL_X_OFFSET=acc_bias[1][0];
float ACCEL_Y_OFFSET=acc_bias[0][0];
float ACCEL_Z_OFFSET=acc_bias[2][0];
float ACCEL_X_SCALE=Ainv[1][1];
float ACCEL_Y_SCALE=Ainv[0][0];
float ACCEL_Z_SCALE=Ainv[2][2];
//float accel_min[3];
//float accel_max[3];
float magnetom[3];
float Minv[3][3]={
                  {0.002090,-0.000219,0.000010},
                  {-0.000219,.003007,-0.000327},
                  {0.000010,-0.000327,.002445}
                };
float mm_bias[3][1]={11.829816,-131.996512,34.301263}; 
float MAGN_X_OFFSET=-mm_bias[1][0];
float MAGN_Y_OFFSET=-mm_bias[0][0];
float MAGN_Z_OFFSET=-mm_bias[2][0];
float MAGN_X_SCALE=-Minv[1][1];
float MAGN_Y_SCALE=-Minv[0][0];
float MAGN_Z_SCALE=-Minv[2][2];
//float magnetom_min[3];
//float magnetom_max[3];

float gyro[3];
//float gyro_average[3];
//int gyro_num_samples = 0;

float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3]= {0, 0, 0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw, pitch, roll;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;
float accloff[3];
  double dt=0.036;
  //these noises are the noises in the measurment due to the instrument i.e. our GPS
  double noise[1][1];
  //W matrix is the error matrix in our model it will contain 2 elements one corresponding to the position and one containing velocity
  double W[2][1];
  //tracking the position and velocity and acc of our bot
  double pos=0;//it will store the diatance our bot has moves since the last turn of our bot
  double vel=0;//stores the current velocity of the robot
  double acc=0;//stores the resultant of all the accelerations 
  //these are the covariance matrices for the noise 1 is for sensor other one is for our model error
  double R[1][1],Q[2][2];
  //these two matrices are cloumn matrices
  double state[2][1]={
            {pos},
            {vel}
           };//will store the best possible estimation of our current state
  //constant matrices A,B and C
  double A[2][2]={
              {1,dt},
              {0,1}
            };
  
  double B[2][1]={
            {dt*dt/2},
            {dt}
          };
  
  double C[1][2]={
            {1,0}
          };
  //this matrix will contain the value of acc. coming from IMU
  double u[1][1]={
            {acc}
          };
  //these matrices will be used for predicating state
  double Axhat[2][1];
  double Bu[2][1];
  double xbar[2][1];//this is the sum of A*xhat and B*u 
/*  //these matrices are basically covariance matrices
  double P[2][2];
  //these will be used in the prediction of matrix Px
  double AP[2][2];
  double AT[2][2];
  double APAT[2][2];
  double Pbar[2][2];
  //this matrix K is the knob of our kalman filter
  double K[2][1];
  //these matrices will be used in the calculations of K 
  double CT[2][1];
  double PbarCT[2][1];
  double CPbarCT[1][1];
  double CPbarCTR[1][1];
  double CPbarCTRinv[1][1];
  //these will be used in the update of our state vector
  double Cbar[1][1];
  double Z[1][1]={pos};
  double gap[1][1];
  double kalmancorrection[2][1];//this is basically Z - Zbar
  //these will b used in the update of matrix P
  double KC[2][2];
  double I[2][2]={
          {1,0},
          {0,1},
        };
  double IminusKC[2][2];
*/
void ReadSensors() {
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();  
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  ReadSensors();
  
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}
//removes the offset from gyroscope
void gyro_offset(){
    int i=0;
    for(i=0;i<10;i++){
        ReadSensors();
        GYRO_X_OFFSET -= (gyro[1]/10);
        GYRO_Y_OFFSET -= (gyro[0]/10);
        GYRO_Z_OFFSET -= (gyro[2]/10);
    };
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
    //magnetometer's manual caliberation
    /*magnetom[0]=(Minv[0][0]*(magnetom[0]-mm_bias[0][0]))+(Minv[0][1]*(magnetom[1]-mm_bias[1][0]))+(Minv[0][2]*(magnetom[2]-mm_bias[2][0]));
    magnetom[1]=(Minv[1][0]*(magnetom[0]-mm_bias[0][0]))+(Minv[1][1]*(magnetom[1]-mm_bias[1][0]))+(Minv[1][2]*(magnetom[2]-mm_bias[2][0]));
    magnetom[2]=(Minv[2][0]*(magnetom[0]-mm_bias[0][0]))+(Minv[2][1]*(magnetom[1]-mm_bias[1][0]))+(Minv[2][2]*(magnetom[2]-mm_bias[2][0]));   */
    // Magnetometer axis mapping
    Magn_Vector[1] = -magnetom[0];
    Magn_Vector[0] = -magnetom[1];
    Magn_Vector[2] = -magnetom[2];

    // Magnetometer values mapping
    Magn_Vector[0] -= MAGN_X_OFFSET;
    Magn_Vector[0] *= MAGN_X_SCALE;
    Magn_Vector[1] -= MAGN_Y_OFFSET;
    Magn_Vector[1] *= MAGN_Y_SCALE;
    Magn_Vector[2] -= MAGN_Z_OFFSET;
    Magn_Vector[2] *= MAGN_Z_SCALE;
    
    //accelerometerl's manual caliberation
   /* accel[0]=(Ainv[0][0]*(accel[0]-acc_bias[0][0]))+(Ainv[0][1]*(accel[1]-acc_bias[1][0]))+(Ainv[0][2]*(accel[2]-acc_bias[2][0]));
    accel[1]=(Ainv[1][0]*(accel[0]-acc_bias[0][0]))+(Ainv[1][1]*(accel[1]-acc_bias[1][0]))+(Ainv[1][2]*(accel[2]-acc_bias[2][0]));
    accel[2]=(Ainv[2][0]*(accel[0]-acc_bias[0][0]))+(Ainv[2][1]*(accel[1]-acc_bias[1][0]))+(Ainv[2][2]*(accel[2]-acc_bias[2][0]));*/
    
    // Accelerometer axis mapping
    Accel_Vector[1] = accel[0];
    Accel_Vector[0] = accel[1];
    Accel_Vector[2] = accel[2];

    // Accelerometer values mapping
    Accel_Vector[0] -= ACCEL_X_OFFSET;
    Accel_Vector[0] *= ACCEL_X_SCALE;
    Accel_Vector[1] -= ACCEL_Y_OFFSET;
    Accel_Vector[1] *= ACCEL_Y_SCALE;
    Accel_Vector[2] -= ACCEL_Z_OFFSET;
    Accel_Vector[2] *= ACCEL_Z_SCALE;
    
    // Gyroscope axis mapping
    Gyro_Vector[1] = -gyro[0];
    Gyro_Vector[0] = -gyro[1];
    Gyro_Vector[2] = -gyro[2];

    // Gyroscope values mapping
    Gyro_Vector[0] -= GYRO_X_OFFSET;
    Gyro_Vector[0] *= GYRO_X_SCALE;
    Gyro_Vector[1] -= GYRO_Y_OFFSET;
    Gyro_Vector[1] *= GYRO_Y_SCALE;
    Gyro_Vector[2] -= GYRO_Z_OFFSET;
    Gyro_Vector[2] *= GYRO_Z_SCALE;
}

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT_BAUD_RATE);
  
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();
  gyro_offset();
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  //matrixR the covariance of the noice matrix here noise matrix is just the standard deviation of position 
 // covariance (noise[0],1,R[0]);
  //the matrix Q that is going to be used in the presdiction step of matrix P
 // covariance (W[0],2,Q[0]);
  //for communicating with GPS
  //removeaccoff();
  Serial.begin(57600);
  Serialg.begin(9600);  
  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  setupgps();
}

// Main loop
void loop()
{
  // Time to read the sensors again?
    if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    ReadSensors();
    
    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading

    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
   // Serial.println("going to get GPS data");
   // gpsdata();
//we take data form the IMU here
    u[0][0]=Accel_Vector[0]-accloff[0];
//if we have to then it's the palce where we will remove the g vector from our acceleration and we will get the acceleration as the resultant of our overall accleration

//now we will predict state fo our bot
    product(A[0],2,2,state[0],2,1,Axhat[0],2,1);
    product(B[0],2,1,u[0],1,1,Bu[0],2,1);
    summation(Axhat[0],2,1,Bu[0],2,1,xbar[0],2,1);
    Serial.print(u[0][0]); Serial.print("\t");
    Serial.print(xbar[1][0]); Serial.print("\t");
    Serial.print(Bu[0][0]*10000); Serial.print("\t");
    Serial.println(xbar[0][0]);
    state[0][0]=xbar[0][0];
    state[1][0]=xbar[1][0];
    digitalWrite(0,LOW);
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    digitalWrite(8,HIGH);
    analogWrite(5,250);
    analogWrite(6,250);
    delay(100);
    if(xbar[0][0]>=0.2){
      digitalWrite(0,LOW);
      digitalWrite(2,LOW);
      digitalWrite(3,LOW);
      digitalWrite(8,LOW);
      while(1){
         delay(1000);
         Serial.print("distance covered");
      }
    }
    //Serial.println("dsafsa");
//here xhat will store the value of our predicted state
//here we will take the data from GPS at frequency diffrent form our IMU.
 /* bool newdata = false;
  if (Serialg.available()) {
    char ch = Serialg.read();
    // Serial.print(c);  // uncomment to see raw GPS dat
    if (gps.encode(ch)) {
      newdata = true;
    }
  }
  
  if (newdata) {
    //basically this is the function that will give us the data from the GPS
    gpsdump(gps);
    //now here we have to change the latitudes and longitudes in terms of the distance from the start of our path 
    Z[0][0]=distance(latitude,longitude);
    //now we will predict the value of matrixPx
    product(A[0],2,2,P[0],2,2,AP[0],2,2);
    transpose(A[0],2,2,AT[0],2,2);
    product(AP[0],2,2,AT[0],2,2,APAT[0],2,2);
    summation(APAT[0],2,2,Q[0],2,2,Pbar[0],2,2);
    //now we will find out the value of K
    transpose(C[0],1,2,CT[0],2,1);
    product(Pbar[0],2,2,CT[0],2,1,PbarCT[0],2,1);
    product(C[0],1,2,PbarCT[0],2,1,CPbarCT[0],1,1);
    summation(CPbarCT[0],1,1,R[0],1,1,CPbarCTR[0],1,1);
    inverse(CPbarCT[0],1,1,CPbarCTRinv[0],1,1);
    product(PbarCT[0],2,1,CPbarCTRinv[0],1,1,K[0],2,1);
    //now we will update state vector
    product(C[0],1,2,state[0],2,1,Cbar[0],1,1);
    subtract(Z[0],1,1,Cbar[0],1,1,gap[0],1,1);
    product(K[0],2,1,gap[0],1,1,kalmancorrection[0],2,1);
    summation(xbar[0],2,1,kalmancorrection[0],2,1,state[0],2,1);
    //now we will update the matrix P
    product(K[0],2,1,C[0],1,2,KC[0],2,2);
    subtract(I[0],2,2,KC[0],2,2,IminusKC[0],2,2);
    product(IminusKC[0],2,2,Pbar[0],2,1,P[0],2,1);
    // we have our updated positon that we can rely upon
  }
//here we need to take the components of the pos on the line

// here we will check the conditions of error and all
      if(error(latitude,longitude)>=1.5){
      //stop the motors
      //wait for 5 seconds and take the average of GPS readings 
      
      //if again error >=1.5m then make this point as our new start as our new start point
          if(error(latitude,longitude)>=1.5){
              reset();
          }
      }

//here we check if we reached our next way point
    if(reached(latitude,longitude)){
       //first we will reset the current and nest way points
       prevlat=latitude;
       prevlong=longitude;
//       nextlat= ;
//       nextlong=;
       //in this portion we will give the instruction to turn
       turn(0);
        //now give the instruction to move forward
    }
    turn(0);
    
    /*Serial.print(Gyro_Vector[0]); Serial.print("\t");
    Serial.print(Gyro_Vector[1]); Serial.print("\t");
    Serial.print(Gyro_Vector[2]); Serial.println();*/
    /*Serial.print(Accel_Vector[0]); Serial.print("\t");
    Serial.print(Accel_Vector[1]); Serial.print("\t");
    Serial.print(Accel_Vector[2]); Serial.println("\t\t\t\t\t");*/
//    Serial.print(TO_DEG(yaw));    Serial.println();
//    Serial.print(TO_DEG(pitch));  Serial.print("\t");
//    Serial.print(TO_DEG(roll));   Serial.println();
//    delay(10);
    //Serial.print(temperature);    Serial.print(";");
    //Serial.print(pressure);       Serial.print(";");
    //Serial.print(altitude);       Serial.println();*/
    }
}

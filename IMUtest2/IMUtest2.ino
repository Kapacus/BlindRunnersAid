#include "quaternionFilters.h"
#include "MPU9250B.h"

MPU9250 myIMU;

const float RAD2DEG = (57.2957795131f);
const float DEG2RAD = (0.01745329251f);
int distance;  

bool DEBUG = false;
bool CALIBMAG = false;

//alpha = tau/(tau+dt)
static const float alphaComp = 1/(1+0.01);//0.98;
static const float alphaYaw = 5/(5+0.01);

float oldYaw;

bool setupIMU(MPU9250& IMU);
void readIMU(MPU9250& IMU);
void printImuData (MPU9250& IMU);
float fold360(float x);
void complementaryFilter (MPU9250& IMU);
void quarternionFilter(MPU9250& IMU);


//double foldTo180(double val);


void setup(){

  Wire.begin();
  Serial.begin(115200);
  bool setupComplete;
  setupComplete = setupIMU(myIMU);
  delay(1000);
  Serial.println("Time Gyro_x Gyro_y Gyro_z Acc_x Acc_y Acc_z Pitch_acc Roll_acc Pitch_gyr Roll_gyr Yaw_gyr Pitch Roll Yaw");
  myIMU.updateTime();
  
 
}// void setup

void loop()
{
  
  float pitchAcc, rollAcc, pitchGyr, rollGyr, yawGyr;
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  { 
    
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 10) 
    {
      readIMU(myIMU);
            
      pitchAcc = atan2f((float)myIMU.ax, sqrtf((float)myIMU.ay * (float)myIMU.ay + (float)myIMU.az * (float)myIMU.az));
      rollAcc = atan2f((float)myIMU.ay, sqrtf((float)myIMU.ax * (float)myIMU.ax + (float)myIMU.az * (float)myIMU.az));
      // Make acc dat range into 0 to 2*pi
      /*if (myIMU.az < 0.0f)
      {
        pitchAcc = M_PI - pitchAcc;
        rollAcc = M_PI - rollAcc;
      }
      else if(myIMU.az > 0.0f && pitchAcc < 0.0f)
      {
        pitchAcc = 2 * M_PI + pitchAcc;
      }
      else if(myIMU.az > 0.0f && rollAcc < 0.0f)
      {
        rollAcc = 2 * M_PI + rollAcc;
      }*/
      
      pitchAcc *= RAD_TO_DEG;
      rollAcc *= RAD_TO_DEG;

      myIMU.updateTime();

      pitchGyr = myIMU.pitch + (float)myIMU.gx * myIMU.deltat;
      rollGyr = myIMU.roll + (float)myIMU.gy * myIMU.deltat;
      yawGyr = myIMU.yaw + (float)myIMU.gz * myIMU.deltat;
      
      myIMU.pitch = alphaComp * (pitchGyr) + (1 - alphaComp) * pitchAcc; // Angle around the X-axis
      myIMU.pitch = fold360((float)myIMU.pitch);
      
      myIMU.roll = alphaComp * (rollGyr) + (1 - alphaComp) * rollAcc; // Angle around the Y-axis //IMU.roll += ((float)IMU.gy * IMU.deltat); // Angle around the Y-axis
      myIMU.roll = fold360((float)myIMU.roll);
      
      myIMU.yaw = myIMU.yaw + (alphaYaw) * ((float)myIMU.gz - myIMU.yaw);
      
      Serial.print(myIMU.sum);
      Serial.print(" ");
      Serial.print(myIMU.gx);
      Serial.print(" ");
      Serial.print(myIMU.gy);
      Serial.print(" ");
      Serial.print(myIMU.gz);
      Serial.print(" ");
      Serial.print(myIMU.ax);
      Serial.print(" ");
      Serial.print(myIMU.ay);
      Serial.print(" ");
      Serial.print(myIMU.az);
      Serial.print(" ");
      Serial.print(pitchAcc);
      Serial.print(" ");
      Serial.print(rollAcc);
      Serial.print(" ");
      Serial.print(pitchGyr);
      Serial.print(" ");
      Serial.print(rollGyr);
      Serial.print(" ");
      Serial.print(yawGyr);
      Serial.print(" ");
      Serial.print(myIMU.pitch);
      Serial.print(" ");
      Serial.print(myIMU.roll);
      Serial.print(" ");
      Serial.print(myIMU.yaw);
      Serial.println("");
  
      myIMU.count = millis(); 
      //sumCount = 0;
      //sum = 0; 
    }
      
  } // if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
}// void loop


bool setupIMU(MPU9250& IMU){
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71)
  {
    if (DEBUG){
    Serial.print("MPU9250 is online at address: 0x");
    Serial.println(c,HEX);
    Serial.println("Calibrating and initializing gyro and accelerometer...");

    //Run self-test
    IMU.MPU9250SelfTest(IMU.selfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU.selfTest[5],1); Serial.println("% of factory value");
    
    }
    //Calibrate gyro and accelerometers, load biases in bias registers
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
    //Initialize IMU for active mode
    IMU.initMPU9250();

    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    IMU.initAK8963(IMU.factoryMagCalibration);
    if (CALIBMAG == true) IMU.magCalMPU9250(IMU.magBias, IMU.magScale);
    else 
    {
      IMU.magBias[0] = (float)198.55;
      IMU.magBias[1] = (float)188.64;
      IMU.magBias[2] = (float)-35.67;
      IMU.magScale[0] = (float)1.16;
      IMU.magScale[1] = (float)1.06;
      IMU.magScale[2] = (float)0.84;
    }
    if (DEBUG){
    Serial.print("AK8963 is online at address: 0x");
    Serial.println(d,HEX);
    Serial.println("Initializing magnetometer...");
    Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(IMU.factoryMagCalibration[0], 2);
    Serial.println(IMU.magBias[0], 2);
    Serial.println(IMU.magScale[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(IMU.factoryMagCalibration[1], 2);
    Serial.println(IMU.magBias[1], 2);
    Serial.println(IMU.magScale[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(IMU.factoryMagCalibration[2], 2);
    Serial.println(IMU.magBias[2], 2);
    Serial.println(IMU.magScale[2], 2);
    }
    return true;
  }// if (==0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    return false;
  }
}

void readIMU(MPU9250& IMU)
{
  IMU.readAccelData(IMU.accelCount); //Read the accelerometer x/y/z adc value, store in accelCount
  IMU.getAres(); //Load accel resolution values
  //Calculate acceleration adc values into g's
  IMU.ax = (float)IMU.accelCount[0]*IMU.aRes - IMU.accelBias[0] + 0.090157972079356;
  IMU.ay = (float)IMU.accelCount[1]*IMU.aRes - IMU.accelBias[1] + 0.038743570903749;
  IMU.az = (float)IMU.accelCount[2]*IMU.aRes - IMU.accelBias[2];// - 0.992601028655367;
    
  IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values, store in gyroCount
  IMU.getGres();
  //Calculate the gyro adc value into degrees per second
  IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes - IMU.gyroBias[0] - 1.686017634092570;
  IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes - IMU.gyroBias[1] - 0.991355620867011;
  IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes - IMU.gyroBias[2] - 0.152751653196180;
   
  IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
  IMU.getMres();
  //Calculate the magnetometer adc value into milliGauss
  IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magScale[0] - IMU.magBias[0];
  IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magScale[1] - IMU.magBias[1];
  IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magScale[2] - IMU.magBias[2];
  
}//readIMU(MPU9250& IMU)


void printImuData ( MPU9250& IMU)
{
  Serial.print("accX = "); Serial.print(1000*IMU.ax);
  Serial.print(", accY = "); Serial.print(1000*IMU.ay);
  Serial.print(", accZ = "); Serial.print(1000*IMU.az); Serial.println(" mg");

  Serial.print("gyrX = "); Serial.print(IMU.gx);
  Serial.print(", gyrY = "); Serial.print(IMU.gy);
  Serial.print(", gyrZ = "); Serial.print(IMU.gz); Serial.println(" deg/s");

  Serial.print("magX = "); Serial.print(IMU.mx);
  Serial.print(", magY = "); Serial.print(IMU.my);
  Serial.print(", magZ = "); Serial.print(IMU.mz); Serial.println(" mG");

  Serial.print("Yaw = ");  Serial.print(IMU.yaw, 2);
  Serial.print(", Pitch = "); Serial.print(IMU.pitch, 2);
  Serial.print(", Roll = "); Serial.print(IMU.roll, 2); Serial.println(" deg");

//  Serial.print("Heading = "); Serial.println(-calcHeading(), 2);

  Serial.print("rate = ");
  Serial.print((float)IMU.sumCount/IMU.sum, 2);
  Serial.println(" Hz");
  Serial.println("");
}//printImuData ( MPU9250& IMU)

float fold360(float x)
{ 
  while (x < 0)
  {
    x += 360;
  }
  while (x >= 360)
  {
    x -= 360;
  }
  return x;
}


void complementaryFilter (MPU9250& IMU)
{
  float tau = 1.0/(2*M_PI);
  float alpha = 0.9876;
  float pitchAcc, rollAcc, yawAcc, yawMag;
  float oldYaw = IMU.yaw;
  
    //Find angle by integrating gyro data
  IMU.pitch += ((float)IMU.gx * IMU.deltat); // Angle around the X-axis
  IMU.roll -= ((float)IMU.gy * IMU.deltat); // Angle around the Y-axis //IMU.roll += ((float)IMU.gy * IMU.deltat); // Angle around the Y-axis
  IMU.yaw += ((float)IMU.gz * IMU.deltat);
  //IMU.yaw += (float)foldTo180(alpha*(IMU.gz * tau + (-calcHeading()*RAD2DEG - initHeading) - oldYaw));
  
  // Compensate for drift with accelerometer data
  int signZ = 1;
  if ( IMU.az < 0) signZ = -1;
  float forceMagnitudeApprox = abs(IMU.ax) + abs(IMU.ay) + abs(IMU.az);
  if (forceMagnitudeApprox > 0.5 && forceMagnitudeApprox < 2)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)IMU.ay, (float)IMU.az) * 180 / M_PI;
    //pitchAcc = atan2f((float)IMU.ay, (float)sqrt(IMU.ax * IMU.ax + IMU.az * IMU.az)) * 180 / M_PI;
    IMU.pitch = IMU.pitch * alpha + pitchAcc * (1-alpha);

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)IMU.ax, (float)IMU.az)* 180 / M_PI;
    //rollAcc = -(atan2f((float)IMU.ax, (float)signZ*sqrt(IMU.ay * IMU.ay * 0.01 + IMU.az * IMU.az)) * 180 / M_PI);
    IMU.roll = IMU.roll * alpha + rollAcc * (1-alpha);

    yawAcc = atan2f((float)IMU.ax, (float)IMU.ay) * 180 / M_PI;
    IMU.yaw = alpha * IMU.yaw + yawAcc * (1 - alpha);
    

//    yawMag = foldTo180(-calcHeading() - initHeading);
    //IMU.yaw = oldYaw + (1-alpha) * (IMU.yaw + foldTo180(yawMag-oldYaw));
    //IMU.yaw = alpha * (oldYaw + IMU.gz * IMU.deltat) + (1-alpha) *(yawMag);
    //IMU.yaw = foldTo180(IMU.yaw);
  }
  // Declination of DTU Electro (55°47'00.3"N 12°31'00.8"E) is
    //    3.58° E  ± 0.40° on 2017-03-07
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    //IMU.yaw   -= 3.58;
}//complementaryFilter (MPU9250& IMU)


void quarternionFilter(MPU9250& IMU)
{
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

  
// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
    IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));

    IMU.pitch *= RAD_TO_DEG;
    IMU.roll *= RAD_TO_DEG;
    IMU.yaw   *= RAD_TO_DEG;
    // Declination of DTU Electro (55°47'00.3"N 12°31'00.8"E) is
    //    3.58° E  ± 0.40° on 2017-03-07
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    IMU.yaw   -= 3.58;
}//quarternionFilter(MPU9250& IMU)

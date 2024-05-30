

#include <Wire.h>
#include <Kalman.h> 
#include <SFE_BMP180.h>
#include <PID_v1.h>
#include <Servo.h>
#include <EEPROM.h>


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

SFE_BMP180 pressure;

Servo Servo_X;
Servo Servo_Y;
Servo Servo_G;


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
unsigned long latest_reading;
uint8_t i2cData[14]; // Buffer for I2C data

double baseline; // baseline pressure
double alt[2] = {0, 0};

double Setpoint, Servo_Pos_X, Servo_Pos_Y;

double consKp_Y = 1.5, consKi_Y = 8, consKd_Y = 0.08;
double consKp_X = 1.5, consKi_X = 8, consKd_X = 0.08;

int address = 0;
bool Flag = false;
PID PID_Y(&kalAngleY, &Servo_Pos_Y, &Setpoint, consKp_Y, consKi_Y, consKd_Y, DIRECT);
PID PID_X(&kalAngleX, &Servo_Pos_X, &Setpoint, consKp_X, consKi_X, consKd_X, DIRECT);

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);

  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  pinMode(D3, OUTPUT); //Red led
  pinMode(D4, OUTPUT); //green led
  pinMode(3, INPUT_PULLUP);  // Push Button 03

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    digitalWrite(D3, HIGH);
    while (1);
  } else {
    while (!Flag) if (digitalRead(3) == 0) Flag = !Flag;
    digitalWrite(D4, HIGH);
  }
  pressure.begin();
  EEPROM.begin(512);

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  baseline = getPressure();

  Setpoint = 0;

  PID_X.SetMode(AUTOMATIC);
  PID_Y.SetMode(AUTOMATIC);
  PID_X.SetOutputLimits(45, 135);
  PID_Y.SetOutputLimits(45, 135);
  PID_X.SetTunings(consKp_X, consKi_X, consKd_X);
  PID_Y.SetTunings(consKp_Y, consKi_Y, consKd_Y);

  Servo_X.attach(D6); // x
  Servo_Y.attach(D5); // y
  Servo_G.attach(D8); // Gyro

  Servo_X.write(90);
  Servo_Y.write(90);
  Servo_G.write(90);

  latest_reading = millis();
  timer = micros();
}

void loop() {
  get_Angle();

  //Set Servo Motors
  PID_X.Compute();
  PID_Y.Compute();
  Servo_X.write(Servo_Pos_X);
  Servo_Y.write(Servo_Pos_Y);
  Servo_G.write(Servo_Pos_X);
#if 1
  Serial.print(kalAngleX); //roll

  Serial.print("\t");

  Serial.print(Servo_Pos_X);// pitch

  Serial.print("\t");

  Serial.print(kalAngleY);// pitch

  Serial.print("\t");

  Serial.print(Servo_Pos_Y);// pitch


#endif
  if (((millis() - latest_reading)) > 200) check_appugy();

  Serial.print("\r\n");
  /* Print Data */
  //#if 0 // Set to 1 to activate
  //  Serial.print(accX); Serial.print("\t");
  //  Serial.print(accY); Serial.print("\t");
  //  Serial.print(accZ); Serial.print("\t");
  //
  //  Serial.print("\t");
  //#endif

  delay(2);
}


//*********************************************************************************Angle
void get_Angle() {

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;


  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s


  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  //  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


}

//******************************************************************************ALT
double getPressure()
{
  //   Wire.begin(0x68);
  char status;
  double T, P, p0, a;
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
      }
    }
  }
  return 0.000001;
}

void check_appugy() {
  alt[1] = pressure.altitude(getPressure(), baseline);

  if (alt[1] > alt[0]) alt[0] = alt[1]; // set the max hight we reached

  if (((alt[1] - alt[0]) < -2) || ((alt[0] > 10) && (abs(kalAngleY) > 90))) {
    pinMode(D7, HIGH);
  }
  //  else   pinMode(3, HIGH);

  if ((address < 512)) {
    EEPROM.write(address, alt[1]);
    EEPROM.commit();
  }
  else EEPROM.commit();
  address++;

  latest_reading = millis();

//  Serial.print(alt[1], 2);

}

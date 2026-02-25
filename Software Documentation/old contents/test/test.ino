#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include "PID_v1.h"
#include <math.h>
// --- Create all the objects ---
MPU6050 mpu(Wire);
Adafruit_BMP280 bmp;
Servo ServoX1, ServoY1, servoParachute;

// --- PID Variables ---
double setpoint = 0.0;
double ixangle = 0.0, oxangle = 0.0;
double iyangle = 0.0, oyangle = 0.0;
double izangle = 0.0, ozangle = 0.0;

double raw_diff_x = 0.0, raw_diff_z = 0.0;

double servo_x = 0.0, servo_y = 0.0;

double kp = 2, ki = 0.00005, kd = 2.5; // need to tune this

const double DEADBAND = 1.0;

double baselineX = 0.0;
double baselineZ = 0.0;

float groundReferencePressure = 0.0;

double maxAltitude = -10000;
double currentAltitude = 0;
bool parachuteDeployed = false;
const float apogee_drop_thres = 2.0;
const float min_arm_altitude = 60.0;

PID myPIDX(&raw_diff_x, &oxangle, &setpoint, kp, ki, kd, DIRECT);
PID myPIDY(&iyangle, &oyangle, &setpoint, kp, ki, kd, DIRECT);
PID myPIDZ(&raw_diff_z, &ozangle, &setpoint, kp, ki, kd, DIRECT);

// Z axis of mpu = perpendicular to surface of mpu and in the direction from black processor in the centre of mpu
// X axis of mpu = in the direction connection black processor and yellowish gold rectange

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // --- Servo and PID Setup ---
  ServoX1.attach(24); //Check +Z and -X wrt mpu // X servo wire connects to lower pins of pcb when flight commputer placed in rocket with ground on right side(brown)
  ServoY1.attach(25); //Check +Z and +X wrt mpu // Y servo wire connects to upper pins of pcb when flight commputer placed in rocket
  ServoX1.write(90);
  ServoY1.write(153);

  servoParachute.attach(26);
  servoParachute.write(90);
// till here servo initialization is done

  if (!bmp.begin(0x76)) {
     if (!bmp.begin(0x77)) {
       Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
       while (1); // Halt if no sensor
     }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  groundReferencePressure = 0;
  for(int i=0; i<10; i++) {
    groundReferencePressure += bmp.readPressure();
    delay(10);
  }
  // Convert Pasals to Hectopascals (divide by 100) and average
  groundReferencePressure = (groundReferencePressure / 10.0) / 100.0;

  Serial.print("Ground Pressure: ");
  Serial.print(groundReferencePressure);
  Serial.println(" hPa");

  maxAltitude = bmp.readAltitude(groundReferencePressure);
// till here bmp initialization is done

  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  myPIDZ.SetMode(AUTOMATIC);

  myPIDX.SetOutputLimits(-90, 90);
  myPIDY.SetOutputLimits(-90, 90);
  myPIDZ.SetOutputLimits(-90, 90);
// till here pid initialization is done

  byte status = mpu.begin();
  if(status != 0) { while(1); }
  mpu.calcOffsets();

  Serial.println("Warming up filters...");
  
  for(int i=0; i<100; i++) {
    mpu.update();
    delay(10); 
  }
  
  baselineX = mpu.getAngleX();
  baselineZ = mpu.getAngleZ();
  Serial.print("Baseline X: "); 
  Serial.println(baselineX);
  Serial.print("Baseline Z: "); 
  Serial.println(baselineZ);
}

// --- KALMAN FILTER FUNCTION (The Suspect) ---
double KALMAN(double U, int axis) {
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P1 = 0; //Need to try other values like 1000
  static double U_hat1 = 0;
  static double K1 = 0.2;
  static double P2 = 0; // not needed
  static double U_hat2 = 0; // not needed
  static double K2 = 0.2; // not needed
  static double P3 = 0; //Need to try other values like 1000
  static double U_hat3 = 0;
  static double K3 = 0.2;

  
  if (axis == 1) {
    K1 = P1 * H / (H * P1 * H + R);
    U_hat1 += K1 * (U - H * U_hat1);
    P1 = (1 - K1 * H) * P1 + Q;
    return U_hat1;
  } 
  else if (axis == 2) { // not needed
    K2 = P2 * H / (H * P2 * H + R); // not needed
    U_hat2 += K2 * (U - H * U_hat2); // not needed
    P2 = (1 - K2 * H) * P2 + Q; // not needed
    return U_hat2; // not needed
  } // not needed
  else if (axis == 3) {
    K3 = P3 * H / (H * P3 * H + R);
    U_hat3 += K3 * (U - H * U_hat3);
    P3 = (1 - K3 * H) * P3 + Q;
    return U_hat3;
  }
  return 0;
}


void loop() {
  mpu.update();

  // --- THIS IS THE LINE WE ARE TESTING ---
  // The angle from the MPU is now passed through the Kalman filter

  currentAltitude = bmp.readAltitude(groundReferencePressure);
  double accelMag = sqrt(pow(mpu.getAccX(), 2) + pow(mpu.getAccY(), 2) + pow(mpu.getAccZ(), 2));

  if (currentAltitude > maxAltitude) {
    maxAltitude = currentAltitude;
  }

  if (!parachuteDeployed && maxAltitude > min_arm_altitude) {
    if (accelMag < 0.5) {
      if ((maxAltitude - currentAltitude) >= apogee_drop_thres) {
              servoParachute.write(180);

              parachuteDeployed = true;

              Serial.println("\n!!! APOGEE DETECTED - PARACHUTE DEPLOYED !!!");
      }
    }

  }

  double current_raw_x = mpu.getAngleX() - baselineX;
  double current_raw_z = mpu.getAngleZ() - baselineZ;

  ixangle = KALMAN(current_raw_x, 1);
  iyangle = KALMAN(mpu.getAngleY(), 2); // not needed
  izangle = KALMAN(current_raw_z, 3);

  raw_diff_x = ixangle;
  raw_diff_z = izangle;

  if (fabs(raw_diff_x) < DEADBAND && fabs(raw_diff_z) < DEADBAND) {
    servo_x = 90;
    servo_y = 90;
    oxangle = 0;
    ozangle = 0;
  }

  else {
    myPIDX.Compute();
    // myPIDY.Compute(); // not needed
    myPIDZ.Compute();
  // if()
    double raw_servo_x = 90 - (0.707 * (ozangle - oxangle));
    double raw_servo_y = 90 - (0.707 * (ozangle + oxangle));

        // 2. CONSTRAIN THEM (This fixes the 217 / -68 issue)
    servo_x = constrain(raw_servo_x, 0, 180);
    servo_y = constrain(raw_servo_y, 0, 180);  
  }
  
  ServoX1.write(servo_x);
  ServoY1.write(servo_y);

  // Print everything
  Serial.print("\nraw_diff X: ");
  Serial.print(raw_diff_x);
  Serial.print("\nraw_diff Z: ");
  Serial.print(raw_diff_z);

  Serial.print("\noutput x angle: ");
  Serial.print(oxangle);
  Serial.print("\noutput z angle: ");
  Serial.print(ozangle);
  Serial.print("\nKALMAN z angle: ");
  Serial.print(izangle);
  Serial.print("\nKALMAN x angle: ");
  Serial.print(ixangle);

  // Serial.print("\z angle: ");
  // Serial.print(mpu.getAngleZ());
  // Serial.print("\x angle: ");
  // Serial.print(mpu.getAngleX());

  Serial.print("\nServo X angle: ");
  Serial.print(servo_x);
  Serial.print("\nServo Y angle: ");
  Serial.print(servo_y);

  Serial.print("\nAlt: "); Serial.print(currentAltitude);
  Serial.print(" | MaxAlt: "); Serial.print(maxAltitude);
  Serial.print(" | Accel(g): "); Serial.print(accelMag);

  Serial.print("\n+++++++++++++++++++++++++++++++++");

// remove delay when not needed
  delay(10);
}
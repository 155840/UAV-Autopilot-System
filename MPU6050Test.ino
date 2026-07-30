#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 imu;

//Offsets for flat calibration
float pitchOffset = 0.0;
float rollOffset  = 0.0;

//Number of samples to average for calibration
const int CALIB_SAMPLES = 100;

//Calibration Function
void calibrateIMU(int samples = CALIB_SAMPLES) {
  float pitchSum = 0;
  float rollSum  = 0;

  Serial.println("Calibrating IMU, keep board flat...");

  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    //Compute pitch and roll for this sample
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    pitchSum += atan2(ax, az) * 180.0 / M_PI; //Convert rad → deg
    rollSum  += atan2(ay, az) * 180.0 / M_PI; //Convert rad → deg

    delay(10); //small delay between samples
  }

  pitchOffset = pitchSum / samples;
  rollOffset  = rollSum / samples;

  Serial.print("Pitch Offset: "); Serial.println(pitchOffset, 2);
  Serial.print("Roll Offset: ");  Serial.println(rollOffset, 2);
  Serial.println("Calibration complete!\n");
}

//Setup 
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  //Initialize MPU6050
  if (!imu.begin(0x68)) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 Ready\n");

  //Run calibration at startup
  calibrateIMU();
}

//Loop
void loop() {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  //Calculate pitch and roll with offsets applied
  float pitch = atan2(accel.acceleration.x, accel.acceleration.z) * 180.0 / M_PI - pitchOffset;
  float roll  = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / M_PI - rollOffset;

  Serial.print("Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Roll: "); Serial.println(roll, 2);

  delay(500); 
}











#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variables to store orientation data
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("Connection Successful!");
  } else {
    Serial.println("Connection failed!");
  }
}

void loop() {
  // Raw Sensor Data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Converting in terms of g m/s^2 (gravity)
  // 2 ^ 10 = 16384
  float Ax = ax / 16384.0;
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;

  // Converting in terms of angular velocity deg/s
  // = 131
  float Gx = gx / 131.0;
  float Gy = gy / 131.0;
  float Gz = gz / 131.0;

    // Calculate pitch and roll angles
  pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  // yaw = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;
  yaw = atan2(ay, az) * 180.0 / PI;

  Serial.print("Accelerometer: X = ");
  Serial.print(Ax);
  Serial.print(", Y = ");
  Serial.print(Ay);
  Serial.print(", Z = ");
  Serial.print(Az);
  Serial.println(" m/s^2");

  Serial.print("Gyroscope    : X = ");
  Serial.print(Gx);
  Serial.print(", Y = ");
  Serial.print(Gy);
  Serial.print(", Z = ");
  Serial.print(Gz);
  Serial.println(" deg/s");

  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print(" deg; Pitch = ");
  Serial.print(pitch);
  Serial.print(" deg; Yaw = ");
  Serial.print(yaw);
  Serial.println(" deg;");

  Serial.print("Temperature: ");
	Serial.print(mpu.getTemperature() / 340 + 36.53);
	Serial.println(" degC\n");

  delay(1000);
}

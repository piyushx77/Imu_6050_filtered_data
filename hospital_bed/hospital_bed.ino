#include <Wire.h>
#include <MPU6050.h>

// NodeMCU I2C pins
#define I2C_SDA D2  // GPIO4
#define I2C_SCL D1  // GPIO5

// AD0 control pins for each IMU
const int imuPins[3] = {D5, D6, D7}; // Only IMU0 enabled
 // D5=GPIO14, D6=GPIO12, D7=GPIO13

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Set AD0 control pins as outputs and LOW (default address 0x68)
  for (int i = 0; i < 3; i++) {
    pinMode(imuPins[i], OUTPUT);
    digitalWrite(imuPins[i], LOW);
  }
}

void selectIMU(int sensor) {
  // Set AD0 HIGH for selected IMU (address 0x69), others LOW (0x68)
  for (int i = 0; i < 3; i++) {
    digitalWrite(imuPins[i], (i == sensor) ? HIGH : LOW);
  }
  delay(10);  // Allow address change to settle
}

float angle_calc(int sensor) {
  selectIMU(sensor);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.print("IMU ");
    Serial.print(sensor);
    Serial.println(" not found!");
    return -999.0;
  }

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  return pitch;
}

void loop() {
  for (int sensor = 0; sensor < 3; sensor++) {
    float angle = angle_calc(sensor);
    if (angle > -998.0) {
      Serial.print("IMU ");
      Serial.print(sensor);
      Serial.print(" Angle: ");
      Serial.println(angle);
    }
  }
  delay(500);
}

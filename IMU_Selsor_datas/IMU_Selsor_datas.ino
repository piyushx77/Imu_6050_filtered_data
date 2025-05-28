#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float kalman_x = 0, kalman_y = 0, kalman_z = 0;
float kalman_Px = 1, kalman_Py = 1, kalman_Pz = 1;
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;


#define ALPHA 0.1  

void setup() {
    Serial.begin(115200);
    Wire.begin(D2, D1);

    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 Connected!");

    // Enable MPU6050 Digital Low-Pass Filter (DLPF)
    mpu.setDLPFMode(6);  // Mode 6 = 5Hz cutoff for best noise reduction

    // Set sampling rate to 1 kHz
    mpu.setRate(0);
}


void getRawAcceleration(float &ax, float &ay, float &az) {
    int16_t accel_x, accel_y, accel_z;
    mpu.getAcceleration(&accel_x, &accel_y, &accel_z);

    ax = accel_x / 16384.0;  // Convert to g-force (±2g range)
    ay = accel_y / 16384.0;
    az = accel_z / 16384.0;
}


float kalmanFilter(float newValue, float &kalman_state, float &kalman_covariance) {
    float K = kalman_covariance / (kalman_covariance + R_measure);
    kalman_state = kalman_state + K * (newValue - kalman_state);
    kalman_covariance = (1 - K) * kalman_covariance + Q_angle;
    return kalman_state;
}


float lowPassFilter(float newValue, float prevValue) {
    return ALPHA * newValue + (1 - ALPHA) * prevValue;
}


float median(float a, float b, float c) {
    if ((a > b) == (a < c)) return a;
    else if ((b > a) == (b < c)) return b;
    else return c;
}

void loop() {
    float ax, ay, az;
    getRawAcceleration(ax, ay, az);

    
    float ax_med = median(ax, ax, ax);
    float ay_med = median(ay, ay, ay);
    float az_med = median(az, az, az);

    
    float ax_kal = kalmanFilter(ax_med, kalman_x, kalman_Px);
    float ay_kal = kalmanFilter(ay_med, kalman_y, kalman_Py);
    float az_kal = kalmanFilter(az_med, kalman_z, kalman_Pz);

    
    static float ax_filt = 0, ay_filt = 0, az_filt = 0;
    ax_filt = lowPassFilter(ax_kal, ax_filt);
    ay_filt = lowPassFilter(ay_kal, ay_filt);
    az_filt = lowPassFilter(az_kal, az_filt);

    
    Serial.print("Raw:");
    Serial.print(ax, 3); Serial.print(",");
    Serial.print(ay, 3); Serial.print(",");
    Serial.print(az, 3); Serial.print(" | ");

    Serial.print("Kalman:");
    Serial.print(ax_kal, 3); Serial.print(",");
    Serial.print(ay_kal, 3); Serial.print(",");
    Serial.print(az_kal, 3); Serial.print(" | ");

    Serial.print("Final Filtered:");
    Serial.print(ax_filt, 3); Serial.print(",");
    Serial.print(ay_filt, 3); Serial.print(",");
    Serial.print(az_filt, 3); Serial.print(",");
    
    Serial.println();

 
    Serial.print("Raw_X:"); Serial.print(ax); Serial.print(" ");
    Serial.print("Raw_Y:"); Serial.print(ay); Serial.print(" ");
    Serial.print("Raw_Z:"); Serial.print(az); Serial.print(" ");

    Serial.print("Kalman_X:"); Serial.print(ax_kal); Serial.print(" ");
    Serial.print("Kalman_Y:"); Serial.print(ay_kal); Serial.print(" ");
    Serial.print("Kalman_Z:"); Serial.print(az_kal); Serial.print(" ");

    Serial.print("Filtered_X:"); Serial.print(ax_filt); Serial.print(" ");
    Serial.print("Filtered_Y:"); Serial.print(ay_filt); Serial.print(" ");
    Serial.print("Filtered_Z:"); Serial.print(az_filt); Serial.print(" ");

    delay(50); 
}

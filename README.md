# MPU6050 Accelerometer Data Filtering with Kalman and Low-Pass Filters

This Arduino project reads raw acceleration data from an MPU6050 sensor, applies Kalman filtering and low-pass filtering to smooth the data, and outputs the results via Serial Monitor.

##  Features
- Reads raw accelerometer data (X, Y, Z axes) from MPU6050.
- Applies a **Kalman filter** to reduce noise and estimate the true acceleration.
- Applies a **low-pass filter** (LPF) to smooth the signal further.
- Prints raw, Kalman-filtered, and final filtered data to the Serial Monitor.

---
##  Hardware Requirements
- Arduino board (e.g., ESP8266, ESP32, Arduino UNO)
- MPU6050 sensor module
- Jumper wires for connection

---

##  Wiring (for ESP8266 Example)
| MPU6050 Pin | ESP8266 Pin |
|-------------|--------------|
| VCC         | 3.3V or 5V   |
| GND         | GND          |
| SDA         | D2           |
| SCL         | D1           |

_Note: For other Arduino boards, use default `Wire.begin()` (no pin arguments) or specify correct pins._

---

##  Setup
1. Install **MPU6050** library by Electronic Cats or similar.
2. Include required libraries in your Arduino sketch:
   ```cpp
   #include <Wire.h>
   #include <MPU6050.h>
Upload the code to your Arduino board.
open serial plotter to see all the datas in plotted form.
Open the Serial Monitor at 115200 baud.
You’ll see continuous output like:

Raw:0.012, -0.034, 1.004 | Kalman:0.015, -0.031, 1.002 | Final Filtered:0.020, -0.028, 1.001

Kalman: Output from the Kalman filter, reducing high-frequency noise.

Final Filtered: Result after applying a low-pass filter to Kalman output.

Filter Tuning
Kalman Filter Parameters:

float Q_angle = 0.001;  // Process noise variance
float R_measure = 0.03; // Measurement noise variance
Tune these values to adjust responsiveness and noise suppression.

Low-Pass Filter Alpha:

#define ALPHA 0.1  // Smoothing factor (0 < ALPHA < 1)
Increase ALPHA for faster response (less smoothing), decrease for more smoothing.

 Notes
The median() function currently always returns the same value (median of same input). You can expand it to use multiple samples for real median filtering.

Adjust the delay(50) in the loop to control the sampling interval (e.g., 20ms for 50Hz).

 Troubleshooting
MPU6050 connection failed: Double-check wiring, and ensure Wire.begin(D2, D1) matches your board.

No data in Serial Monitor: Ensure the baud rate is set to 115200.

 Future Improvements
Implement a proper median filter with multiple samples.

Add gyro data processing and complementary filtering for orientation estimation.

Store filtered data for visualization or plotting.


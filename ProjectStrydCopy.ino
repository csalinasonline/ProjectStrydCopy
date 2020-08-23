
#include <Arduino_LSM9DS1.h>
 
int led = 13;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  Serial1.begin(115200);
  Serial1.println("[START]");

  // Check IMU
  if (!IMU.begin()) {
    Serial1.println("[INIT CHECK], IMU, IMU not initialized.");
    while (1);
  }
  Serial1.println("[INIT CHECK], IMU, IMU initialized.");
  
  Serial1.println("[VAL CHECK], IMU, ACCL sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, ACCL units: G's");   
  Serial1.println("[VAL CHECK], IMU, GYRO sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, GYRO units: deg/sec");  
  Serial1.println("[HEADER INFO]");  
  Serial1.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z");  
}

// the loop routine runs over and over again forever:
void loop() {
  float accl_x, accl_y, accl_z;
  float gyro_x, gyro_y, gyro_z;
  
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)

  Serial1.print(millis());
  Serial1.print(", ");
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accl_x, accl_y, accl_z);
    Serial1.print(accl_x);
    Serial1.print(", ");
    Serial1.print(accl_y);
    Serial1.print(", ");
    Serial1.print(accl_z);
  }

  Serial1.print(", ");
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    Serial1.print(gyro_x);
    Serial1.print(", ");
    Serial1.print(gyro_y);
    Serial1.print(", ");
    Serial1.println(gyro_z);
  }

  digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
  delay(10);               // wait for a bit
  
  
}

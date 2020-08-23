
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>

#define TIMEOUT 10

int led = 13;
unsigned long current_time;
unsigned long previous_time;
float accl_x, accl_y, accl_z;
float gyro_x, gyro_y, gyro_z;
float pressure;
float temperature;
float humidity;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the IO
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

  if (!BARO.begin()) {
    Serial.println("[INIT CHECK], Pressure, Pressure sensor not initialized.");
    while (1);
  }
  Serial.println("[INIT CHECK], Pressure, Pressure sensor initialized.");
  Serial1.println("[VAL CHECK], Pressure, Pressure units: kPA");   

  if (!HTS.begin()) {
    Serial.println("[INIT CHECK], TempHumid, TempHumid sensor not initialized.");
    while (1);
  }
  Serial.println("[INIT CHECK], TempHumid, TempHumid sensor initialized.");
  Serial1.println("[VAL CHECK], Temp, Temp units: C");  
  Serial1.println("[VAL CHECK], Humidity, Humidity units: %");  
  
  Serial1.println("[HEADER INFO]");  
  Serial1.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, pressure, temp, humidity"); 
  
  // Setup timer
  current_time = millis();
  previous_time = current_time;
}

// the loop routine runs over and over again forever:
void loop() {
  current_time = millis();
  if ( abs(current_time - previous_time) >= TIMEOUT ) {
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
        Serial1.print(gyro_z);
      }

      Serial1.print(", ");

      pressure = BARO.readPressure();
      Serial1.print(pressure);

      Serial1.print(", ");

      temperature = HTS.readTemperature();
      humidity    = HTS.readHumidity();
      Serial1.print(temperature);      
      Serial1.print(", ");
      Serial1.println(humidity);      
      
      previous_time = current_time;
      digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
    }
  }

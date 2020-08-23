
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>

#define TIMEOUT 10
#define TIMEOUT_2 1000
//#define DEBUG_MODE_DISABLED

int led = 13;
unsigned long current_time;
unsigned long previous_time;
unsigned long current_time_2;
unsigned long previous_time_2;
float accl_x, accl_y, accl_z;
float gyro_x, gyro_y, gyro_z;
float pressure;
float temperature;
float humidity;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the IO
  pinMode(led, OUTPUT);

#ifdef DEBUG_MODE_DISABLED
  Serial1.begin(115200);
  Serial1.println("[START]");
#else
  Serial.begin(115200);
  Serial.println("[START]");  
#endif

  // Check IMU
  if (!IMU.begin()) {
#ifdef DEBUG_MODE_DISABLED 
    Serial1.println("[INIT CHECK], IMU, IMU not initialized.");
#else
    Serial.println("[INIT CHECK], IMU, IMU not initialized.");
#endif    
    while (1);
  }
#ifdef DEBUG_MODE_DISABLED   
  Serial1.println("[INIT CHECK], IMU, IMU initialized.");
#else  
  Serial.println("[INIT CHECK], IMU, IMU initialized.");
#endif   

#ifdef DEBUG_MODE_DISABLED  
  Serial1.println("[VAL CHECK], IMU, ACCL sample rate: ");
  Serial1.print(IMU.accelerationSampleRate());
  Serial1.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, ACCL units: G's");   
  Serial1.println("[VAL CHECK], IMU, GYRO sample rate: ");
  Serial1.print(IMU.gyroscopeSampleRate());
  Serial1.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, GYRO units: deg/sec");   
#else    
  Serial.println("[VAL CHECK], IMU, ACCL sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("[VAL CHECK], IMU, ACCL units: G's");   
  Serial.println("[VAL CHECK], IMU, GYRO sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println("[VAL CHECK], IMU, GYRO units: deg/sec");
#endif

  if (!BARO.begin()) {
#ifdef DEBUG_MODE_DISABLED   
    Serial1.println("[INIT CHECK], Pressure, Pressure sensor not initialized.");
#else
    Serial.println("[INIT CHECK], Pressure, Pressure sensor not initialized.");
#endif    
    while (1);
  }
#ifdef DEBUG_MODE_DISABLED   
  Serial1.println("[INIT CHECK], Pressure, Pressure sensor initialized.");
  Serial1.println("[VAL CHECK], Pressure, Pressure units: kPA");   
#else
  Serial.println("[INIT CHECK], Pressure, Pressure sensor initialized.");
  Serial.println("[VAL CHECK], Pressure, Pressure units: kPA");   
#endif  

  if (!HTS.begin()) {
#ifdef DEBUG_MODE_DISABLED      
    Serial1.println("[INIT CHECK], TempHumid, TempHumid sensor not initialized.");
#else
    Serial.println("[INIT CHECK], TempHumid, TempHumid sensor not initialized.");
#endif    
    while (1);
  }
#ifdef DEBUG_MODE_DISABLED     
  Serial1.println("[INIT CHECK], TempHumid, TempHumid sensor initialized.");
  Serial1.println("[VAL CHECK], Temp, Temp units: C");  
  Serial1.println("[VAL CHECK], Humidity, Humidity units: %");  
#else
  Serial.println("[INIT CHECK], TempHumid, TempHumid sensor initialized.");
  Serial.println("[VAL CHECK], Temp, Temp units: C");  
  Serial.println("[VAL CHECK], Humidity, Humidity units: %");  
#endif

#ifdef DEBUG_MODE_DISABLED     
  Serial1.println("[HEADER INFO]");  
  Serial1.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, pressure, temp, humidity"); 
#else
  Serial.println("[HEADER INFO]");  
  Serial.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, pressure, temp, humidity"); 
#endif
  
  // Setup timer
  current_time = millis();
  previous_time = current_time;
  current_time_2 = millis();
  previous_time_2 = current_time_2;  

  // Read slow sensors before loop
  pressure = BARO.readPressure();
  temperature = HTS.readTemperature();
  humidity    = HTS.readHumidity();
}

// the loop routine runs over and over again forever:
void loop() {
  current_time = millis();
  current_time_2 = millis();
  
  if ( abs(current_time - previous_time) >= TIMEOUT ) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)

#ifdef DEBUG_MODE_DISABLED 
      Serial1.print(millis());
      Serial1.print(", ");
#else
      Serial.print(millis());
      Serial.print(", ");
#endif      
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accl_x, accl_y, accl_z);
#ifdef DEBUG_MODE_DISABLED         
        Serial1.print(accl_x);
        Serial1.print(", ");
        Serial1.print(accl_y);
        Serial1.print(", ");
        Serial1.print(accl_z);
#else
        Serial.print(accl_x);
        Serial.print(", ");
        Serial.print(accl_y);
        Serial.print(", ");
        Serial.print(accl_z);
#endif         
      }
      
#ifdef DEBUG_MODE_DISABLED     
      Serial1.print(", ");
#else
      Serial.print(", ");
#endif       
      
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
#ifdef DEBUG_MODE_DISABLED         
        Serial1.print(gyro_x);
        Serial1.print(", ");
        Serial1.print(gyro_y);
        Serial1.print(", ");
        Serial1.print(gyro_z);
#else
        Serial.print(gyro_x);
        Serial.print(", ");
        Serial.print(gyro_y);
        Serial.print(", ");
        Serial.print(gyro_z);
#endif         
      }

#ifdef DEBUG_MODE_DISABLED 
      Serial1.print(", ");
#else
      Serial.print(", ");
#endif       

#ifdef DEBUG_MODE_DISABLED       
      Serial1.print(pressure);
      Serial1.print(", ");
#else
      Serial.print(pressure);
      Serial.print(", ");
#endif       

#ifdef DEBUG_MODE_DISABLED       
      Serial1.print(temperature);      
      Serial1.print(", ");
      Serial1.println(humidity);
#else
      Serial.print(temperature);      
      Serial.print(", ");
      Serial.println(humidity);
#endif             
      
      previous_time = current_time;
      digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
    }

  if ( abs(current_time_2 - previous_time_2) >= TIMEOUT_2 ) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)

      // Update slow sensor values
      pressure = BARO.readPressure();
      temperature = HTS.readTemperature();
      humidity    = HTS.readHumidity();
      
      previous_time_2 = current_time_2;
      digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW      
  }
    
  }

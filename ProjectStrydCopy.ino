
// Includes
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>
#include <PDM.h>

// Defines
#define TIMEOUT 200UL
#define TIMEOUT_2 1000UL
//#define DEBUG_MODE_DISABLED
#define EQ_1_M 0.005f
#define EQ_2_B -0.005f
#define ANALOG_IN_VBATT A0

// Variables
int led = 13;
unsigned long current_time;
unsigned long previous_time;
unsigned long current_time_2;
unsigned long previous_time_2;
float accl_x, accl_y, accl_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float pressure;
float temperature;
float humidity;
float batt_lvl;
// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
// number of samples read
volatile int samplesRead;

// Prototypes
float get_batt_lvl(void);
void on_pdm_data(void);

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
  Serial1.print("[VAL CHECK], IMU, ACCL sample rate: ");
  Serial1.print(IMU.accelerationSampleRate());
  Serial1.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, ACCL units: G");   
  Serial1.print("[VAL CHECK], IMU, GYRO sample rate: ");
  Serial1.print(IMU.gyroscopeSampleRate());
  Serial1.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, GYRO units: deg/sec");  
  Serial1.print(IMU.magneticFieldSampleRate());
  Serial1.println(" Hz");
  Serial1.println("[VAL CHECK], IMU, MAG units: uT");    
#else    
  Serial.print("[VAL CHECK], IMU, ACCL sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("[VAL CHECK], IMU, ACCL units: G's");   
  Serial.println("[VAL CHECK], IMU, GYRO sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println("[VAL CHECK], IMU, GYRO units: deg/sec");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println("[VAL CHECK], IMU, MAG units: uT");  
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
  Serial1.println("[INIT CHECK], INT_ADC, INT_ADC vbatt initialized.");
  Serial1.println("[VAL CHECK], INT_ADC, vbatt units: V");   
#else
  Serial.println("[INIT CHECK], INT_ADC, INT_ADC vbatt initialized.");
  Serial.println("[VAL CHECK], INT_ADC, vbatt units: V"); 
#endif


  // configure the data receive callback
  PDM.onReceive(on_pdm_data);
  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate  
  if (!PDM.begin(1, 16000)) {
#ifdef DEBUG_MODE_DISABLED      
    Serial1.println("[INIT CHECK], PDM, PDM sensor not initialized.");
#else
    Serial.println("[INIT CHECK], PDM, PDM sensor not initialized.");
#endif 
    while (1);
  }
#ifdef DEBUG_MODE_DISABLED     
  Serial1.println("[INIT CHECK], PDM, PDM sensor initialized.");
  Serial1.println("[VAL CHECK], PDM, PDM units: counts 16bit");   
#else
  Serial.println("[INIT CHECK], PDM, PDM sensor initialized.");
  Serial.println("[VAL CHECK], PDM, PDM units: counts 16bit");   
#endif

#ifdef DEBUG_MODE_DISABLED     
  Serial1.println("[HEADER INFO]");  
  Serial1.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, pressure, temp, humidity, vbatt, mag_x, mag_y, mag_z, microphone"); 
#else
  Serial.println("[HEADER INFO]");  
  Serial.println("time, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, pressure, temp, humidity, vbatt, mag_x, mag_y, mag_z, microphone"); 
#endif
  
  // Setup timer
  current_time = millis();
  previous_time = current_time;
  current_time_2 = millis();
  previous_time_2 = current_time_2;  

  // Read slow sensors before loop
  pressure = BARO.readPressure();
  temperature = HTS.readTemperature();
  humidity = HTS.readHumidity();
  batt_lvl = get_batt_lvl();
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
      Serial1.print(humidity);
      Serial1.print(", ");      
#else
      Serial.print(temperature);      
      Serial.print(", ");
      Serial.print(humidity);
      Serial.print(", ");      
#endif    

#ifdef DEBUG_MODE_DISABLED       
      Serial1.print(batt_lvl); 
#else
      Serial.print(batt_lvl);         
#endif   

#ifdef DEBUG_MODE_DISABLED     
      Serial1.print(", ");
#else
      Serial.print(", ");
#endif       
      
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mag_x, mag_y, mag_z);
#ifdef DEBUG_MODE_DISABLED         
        Serial1.print(mag_x);
        Serial1.print(", ");
        Serial1.print(mag_y);
        Serial1.print(", ");
        Serial1.print(mag_z);
#else
        Serial.print(mag_x);
        Serial.print(", ");
        Serial.print(mag_y);
        Serial.print(", ");
        Serial.print(mag_z);
#endif         
      }   

#ifdef DEBUG_MODE_DISABLED     
      Serial1.print(", ");
#else
      Serial.print(", ");
#endif 
           
#ifdef DEBUG_MODE_DISABLED     
    Serial1.println(sampleBuffer[0]);
#else
    Serial.println(sampleBuffer[0]);
#endif    
      previous_time = current_time;
      digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
    }

  if ( abs(current_time_2 - previous_time_2) >= TIMEOUT_2 ) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)

      // Update slow sensor values
      pressure = BARO.readPressure();
      temperature = HTS.readTemperature();
      humidity = HTS.readHumidity();
      batt_lvl = get_batt_lvl();
      
      previous_time_2 = current_time_2;
      digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW      
  }
}

// fnc: get_batt_lvl
// gets: none
// returns: float
// desc: read raw batt adc and convert to batt voltage
// R1 / R2 is 0.219 ratio for divider, read voltage on R2 (0.82) and not R1 (0.18)
// So if scale 1023 * 0.82 = 839
// 4.2V <=> 839, 3.0 <=> 599
// (4.2-3) / (839-599) = 0.005
// y = mx + b
// where m = 0.005
// b = y - mx
// So let y = 4.2, x = 839, m = m = 0.005
// b = 4.2 - (0.005 * 839) = -0.005
// y = 0.005*x - 0.005
// Also clamp ADC to vbatt, if < 599 set vbatt to 0, if > 839 set vbatt to 4.2
// Test:
// y = 4.2 when x = 839 PASS
// Test:
// y = 3.0 when x = 599 PASS  
// So let R1 = 1.8k and R2 = 8.2k @ 1%
// Measure drop across R2
float get_batt_lvl(void)
{
  int battery = analogRead(ANALOG_IN_VBATT);
  if ( battery < 599 ) {
    return 0;
  }
  else if ( battery > 839 ) {
    return 4.2;
  }
  else {  
    float calc_1 = EQ_1_M * battery;
    float ret_val = calc_1 + EQ_2_B;
    return ret_val; 
  }
}

void on_pdm_data(void) {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

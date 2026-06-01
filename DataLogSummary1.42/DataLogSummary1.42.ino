/* DataLogSummary
 *   This code rapidly acquires data from 2 voltage inputs and streams to the serial port, 
 *   or calculates summary statistics and reports thise at a set interval.
 *      
 *   Additional parameters are: 
 *      interval: sampling interval in us (100 = 10 kHz)
 *      logging_ms: logging interval in ms (2000 = 2s or 0.5Hz)
 *      
 *   Connect the following pins:   
 *      IN1  pin A0
 *      IN2  pin A1
 *      TRIGGER_IN  pin *
 *      TRIGGER     pin *
 *      TRIG_READY  pin * (LED --> resistor --> GND)
 *      REC_ON      pin * (LED --> resistor --> GND)
 *      
 *   Recommended to use with M0 or faster boards. Arduino Uno 
 *   & variants should work but have limited memory (2048 bytes) 
 *   and therefore limited duration. Code tested on Adafruit Metro M0.     
 *   
 *   Updated:
 *     20220519 v1.1 DRA - added second input, sampling rate control and readout
 *     20220601 v1.11 DRA - added Uno support, 12-bit ADC, continuous recording 
 *                    option, and fixed longer interval timing (>1 ms)
 *     20230905 v1.2 DRA - add trigger option
 *     20250221 v1.3 DRA - add summary stats; remove outputs & burst mode
 *     20250227 v1.31 DRA - add hardware default via jumper to allow direct logging without serial input
 *     20250228 v1.32 DRA - add 2*sd/range as a shape parameter --> 1.0 = square; 0.71 = sine, 0.33 = gaussian noise 
 *     20250303 v1.33 DRA - change timing to ms-based if in summary mode, to prevent rollover for > 1h logging
 *     20260324 v1.35 DRA - change var types to long for compatibility with Uno
 *     20260511 v1.4 NS/DRA - add digital logging: TCS34725 color sensor
 *     20260601 v1.42 DRA - add 100k thermistor input
 *     
 *   Potential changes:  
 *     Save current settings in EEPROM memory for easy automation
 *     some form of median, quartiles, shape distibution,. e.g. by counting  
 */

#include <Wire.h>
#include "stats.h"
#include "Adafruit_TCS34725.h"
#include "INA226.h"
 
#define VERSION  1.4

#define VIN1 A0
#define VIN2 A1
#define TRIGGER_IN  12
#define TRIGGER 3
#define TRIG_READY 6
#define REC_ON 4
#define DEFAULT_IN 8
#define TEST 7

#define SERIAL_BAUD 500000

// Settings based on board used
#if defined(ARDUINO_SAMD_ZERO)
  #define MEMLEN 2000    // max size of data memory
  #define NBIT 12        // Analog in number of bits
  #define SET_BITS analogReadResolution(NBIT);
#elif defined(__AVR_ATmega328P__)
  #define MEMLEN 250    // max size of data memory
  #define NBIT 10        // Analog in number of bits
  #define SET_BITS 
#else
  #warning "Board not recognized. Add specs to code."
  #define MEMLEN 2000    // max size of data memory
  #define NBIT 12        // Analog in number of bits
  #define SET_BITS analogReadResolution(NBIT);
#endif

#define INPUT_SIZE 32  // max input serial string size
#define MAX_VALS 5     // max values to read in from serial

int interval = 500;            // interval in us (to set rate; 100 = 10kHz)
long logging_ms = 0;            // logging interval in ms (0 = same as sampling interval; 2000 = 2s or 0.5Hz)
long secondsToRecord = 0;       // seconds to record for continuous output
int n = 2;                     // number of channels to record
long time_unit_s = 1;           // seconds per time unit

int triggerPolarity = 0;  // 0 = LOW, 1 = HIGH

uint16_t val1 = 0;     // input values
uint16_t val2 = 0;

int digital = 0;     // specifies digital sensor number (0 = analog)

//defaults
float tempResistance = 100000.0; // Ohms at 25C
float beta = 3977.0; // check this; could also be 3950K

bool streamRecording = false; 
unsigned long int start_us = micros();
unsigned long int start_ms = millis();
unsigned long int prev_us = start_us;
unsigned long int prev_log_us = start_us;
int readout_ms = 0;

//unsigned int data1[MEMLEN];
//unsigned int data2[MEMLEN];
int t = 0;
int dt = 0;

String inputString = "";         // a string to hold incoming data
char inputBytes[INPUT_SIZE + 1];

statistic::Statistic<float, uint32_t, true> data1;
statistic::Statistic<float, uint32_t, true> data2;



/*initialize int time and gain value for color sensor*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

INA226 INA(0x40);

/*---------------------------------------------------*/

void setup() {
  pinMode(TRIGGER_IN, INPUT_PULLUP); 
  pinMode(DEFAULT_IN, INPUT_PULLUP); 
  pinMode(VIN1, INPUT);
  pinMode(VIN2, INPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(REC_ON, OUTPUT);
  pinMode(TRIG_READY, OUTPUT);

  pinMode(TEST, OUTPUT);
  analogWrite(TEST,128);

  attachInterrupt(digitalPinToInterrupt(TRIGGER_IN), triggerChange, CHANGE);

  SET_BITS // set ADC resolution
  
  Serial.begin(SERIAL_BAUD);       // max serial data rate
  Wire.begin();
  
  delay(500);
  
  showDetails();

  triggerChange();
}

void loop() {

  if (Serial.available()) {
    
    // Get next command from Serial (add 1 for termination character)
    byte size = Serial.readBytes(inputBytes, INPUT_SIZE);
    // Add the final 0 to end the string
    inputBytes[size] = 0;
    inputString = String(inputBytes);
    //int len = strlen(inputString);
 
    Serial.print("["); 
    //Serial.print(inputString); 
    Serial.write(inputString.c_str(), inputString.length() - 1);
    Serial.print("] "); 

    char first = inputString.charAt(0);

    if (first == 'i') {   // Change sampling interval (us)
      char* command = strtok(inputBytes, " ,") + 1;
      interval = atol(command);
      Serial.print(F("Sampling interval changed to "));
      Serial.print(interval);
      Serial.print(F(" us. Sampling rate = "));
      Serial.print(1000. / interval);
      Serial.print(F(" kHz."));
      Serial.println();
    }

    else if (first == 'L') {  // Change logging interval (ms)
      char* command = strtok(inputBytes, " ,") + 1;
      logging_ms = atol(command);
      Serial.print(F("Logging interval changed to "));
      Serial.print(logging_ms);
      Serial.print(F(" ms. Logging rate = "));
      if (logging_ms > 0) {
        Serial.print(1000. / logging_ms);
        Serial.print(F(" Hz."));
      } else {
        Serial.print(F("Sampling rate."));
      }
      Serial.println();
    }
    
    else if (first == 'n') {  // Change number of channels (1 or 2)
      char* command = strtok(inputBytes, " ,") + 1;
      n = atol(command);
      Serial.print(F("Sample channels changed to "));
      Serial.println(n);
    }

    else if (first == 'f') {  // Flip trigger polarity
      triggerPolarity = !triggerPolarity;
      triggerChange();
    }

    else if (first == 'U') {  // Change time unit
      char second = inputString.charAt(1);
      Serial.print(F("Time unit changed to: "));
      if (second == 'm') {
        time_unit_s = 60;
        Serial.println(F("minutes."));            
      }
      else if (second == 'h') 
      {
        time_unit_s = 60*60;
        Serial.println(F("hours.")); 
      }
      else if (second == 'd') {
        time_unit_s = 24*60*60;
        Serial.println(F("days.")); 
      }
      else 
      {
        time_unit_s = 1;
        Serial.println(F("seconds.")); 
      }
    }

    else if (first == 'R' || first == 'T') {  // Set Recording duration
      char* command = strtok(inputBytes, " ,") + 1;
      secondsToRecord = (long)atol(command) * time_unit_s;
      Serial.print(F("Continuous streaming for "));
      Serial.print(secondsToRecord);
      Serial.print(F(" seconds. "));
      
      if (first == 'T') {  // trigger mode
        Serial.print(F("Waiting for trigger...")); 
        waitForTrigger();
      }
      
      Serial.println();
      startLogging();
    }

    else if (first == 'C') {  // choose color sensor
      if (tcs.begin()) {
        Serial.println(F("Color Sensor TCS34725 detected. Mode changed to digital. Colors blue and red"));
        digital = 1;
      }
      else {
        Serial.println(F("Color Sensor TCS34725 NOT detected."));
      }
    }

    else if (first == 'P') {  // choose voltage/current sensor
      if (INA.begin()) {
        Serial.println(F("INA226 detected. Mode changed to digital. Voltage (mV) and current (0.1 mA)"));
        INA.setMaxCurrentShunt(0.5, 0.100);
        digital = 2;
      }
      else {
        Serial.println(F("INA226 NOT detected."));
      }
    }
    else if (first == 'A') {   // Analog voltage recording
      digital = 0;
      Serial.println(F("Mode changed to analog"));
    }
    else if (first == 't') {   // Analog "100k" thermistor recording
      char* command = strtok(inputBytes, " ,") + 1;
      tempResistance = (float)atol(command);

      Serial.print(F("Mode changed to analog, thermistor. Resistance @ 25C: "));
      Serial.print(tempResistance / 1000.);
      Serial.println(F(" kOhms"));
      digital = -1;
      interval = 5000; // max sampling interval 5ms
    }
    
    else
    {
      showDetails();
    } 

  }

  /*---------------------------------*/
  
  if (streamRecording) {

    // Get current data values 
    switch (digital) {
      
      case 0:  // read two voltage inputs
        val1 = analogRead(VIN1);
        if (n>1) val2 = analogRead(VIN2);
        break;
        
      case 1:
        uint16_t g, c;
        tcs.getRawData(&val1, &g, &val2, &c);  // val1 represents red light and val2 blue
        break;

      case 2:
        val1 = (uint16_t) (INA.getBusVoltage()*1000);
        val2 = (uint16_t) (INA.getCurrent_mA()*10);
        break;

      case -1:  // read analog thermistor voltage inputs
        val1 = get_tempC_x100(VIN1,tempResistance,beta);
        if (n>1) val2 = get_tempC_x100(VIN2,tempResistance,beta);
        break;
      default:
        break;
    }
    
//    if (digital==0) {  // read two voltage inputs
//      val1 = analogRead(VIN1);
//      if (n>1) val2 = analogRead(VIN2);
//    }
//    
//    else {   // read red and blue values
//      uint16_t g, c;
//      tcs.getRawData(&val1, &g, &val2, &c);  // val1 represents red light and val2 blue
//    }
    
    unsigned long int ms = millis();
    unsigned long int t_ms = ms - start_ms;
    
    unsigned long int us = micros();
    unsigned long int t_us = us - start_us;
    
    unsigned int dt = us - prev_us + 5;  // allow 5 us for commands
    //if (dt > 0) 
    if (dt < interval) delayMicroseconds(interval-dt);
    prev_us = micros(); // ***check later if the 5us add can be avoided by setting prev_us = us; ***

    if (logging_ms == 0) {  // for fast stream, no stats
      Serial.print(t_us);
      Serial.print(",");
      Serial.print(val1);
      if (n>1) {
        Serial.print(",");
        Serial.print(val2);
      } 
      Serial.println();
    } else {                // for summary logging
      // add current vals to stats
      data1.add(val1);
      if (n>1 || digital>0) data2.add(val2);

      // when logging interval has elapsed, calculate summary stats
      if ((us - prev_log_us) >= logging_ms * 1000) {  
        // prev_log_us = us;
        prev_log_us += (logging_ms * 1000);
        Serial.print((t_ms / 1000.),1);
        Serial.print(", ");
        
        Serial.print(val1);
        Serial.print(",");
        Serial.print(data1.average(),1);
        Serial.print(",");
        Serial.print(data1.minimum(),0);
        Serial.print(",");
        Serial.print(data1.maximum(),0);
        Serial.print(",");
        Serial.print(data1.count());
        Serial.print(",");
        Serial.print(data1.rise_count() * 1000. / logging_ms, 1);
        Serial.print(",");
        Serial.print(data1.pop_stdev(),1);
        Serial.print(",");
        Serial.print(2 * data1.pop_stdev() / data1.range(),2);

        if (n>1 || digital>0) {
          Serial.print(", ");
          Serial.print(val2);
          Serial.print(",");
          Serial.print(data2.average(),1);
          Serial.print(",");
          Serial.print(data2.minimum(),0);
          Serial.print(",");
          Serial.print(data2.maximum(),0);
          Serial.print(",");
          Serial.print(data2.count());
          Serial.print(",");
          Serial.print(data2.rise_count() * 1000. / (logging_ms - readout_ms), 1);
          Serial.print(",");
          Serial.print(data2.pop_stdev(),1); 
          Serial.print(",");
          Serial.print(2 * data2.pop_stdev() / data2.range(),2);
        }
        Serial.println();
        
        data1.clear();
        if (n>1) data2.clear();

        readout_ms = millis() - ms;
      }
    
    }

    // if (t_us >= (1000000 * secondsToRecord) + 1000*logging_ms/2) {  // end recording
    if (t_ms >= (1000 * secondsToRecord) + logging_ms/2) {  // end recording
      streamRecording = false; 
      digitalWrite(REC_ON,LOW);
      Serial.println("END");
    }
  }

  /* if not stream recording */
  else {
    digitalWrite(REC_ON,LOW);
    delay(25);
    //Serial.println('.');

    // if default jumper is set, then set defaults and wait for button press
    if (digitalRead(DEFAULT_IN)==LOW) {

      Serial.print(F("Default triggered logging: "));

      // set defaults
      n = 1;
      interval = 500; // us = 2kHz
      logging_ms = 2000; // ms = 0.5Hz
      secondsToRecord = 24 * 60 * 60; // 24h
      triggerPolarity = 0;

      Serial.print(secondsToRecord); 
      Serial.print("  ");
      
      showLoggingInfo();
      
      // wait for trigger then stream recording
      waitForTrigger();
      startLogging();
    }  

  }
}

void waitForTrigger() {
    digitalWrite(TRIG_READY,HIGH);
    while(digitalRead(TRIGGER_IN)!=triggerPolarity) {
      delay(25);
    }
    digitalWrite(TRIG_READY,LOW);
}

void showLoggingInfo() {
      Serial.print(1000./interval,1);
      Serial.print("kHz, log every ");
      Serial.print(logging_ms / 1000.,1);
      Serial.print("s for ");
      Serial.print(secondsToRecord);
      Serial.print("s = ");
      Serial.print(secondsToRecord / 60);
      Serial.print("min = ");
      Serial.print(secondsToRecord / 60 / 60);
      Serial.println("hr.");
}

void startLogging() {
      streamRecording = true;
      digitalWrite(REC_ON,HIGH);
      
      Serial.println(F("Data output format:"));
      if (logging_ms == 0) {
        Serial.println(F("Time(us),data1,data2"));
      } else {
        Serial.println(F("Time(s),{instant,mean,min,max,#,freq(Hz),stdev,shape:sq1.0,sin0.7,tri0.6,n0.3} data1, ... data2"));
      }
      Serial.println();
      Serial.println(F("Logging data..."));

      t = 0;
      start_ms = millis();
      start_us = micros();
      prev_us = start_us;
      prev_log_us = start_us;
}

void showDetails() {
  Serial.print(F("FastDataLogMulti v"));
      Serial.println(VERSION);
      Serial.println(F("=========================="));
      Serial.print(F("Records up to two inputs at pins ")); 
      Serial.print(VIN1); Serial.print(" "); Serial.println(VIN2);
      Serial.print(F("Trigger input at pin ")); 
      Serial.println(TRIGGER_IN);
      Serial.print(F("Max data points: "));
      Serial.println(MEMLEN);
      Serial.print(F("Serial baud rate: "));
      Serial.println(SERIAL_BAUD);
      Serial.println();
      Serial.println(F("Enter timing using one of the following syntaxes for continuous recording:"));
      Serial.println(F("U{s/m/h} defines time units as [s]econds, [m]inutes or [h]ours (s = default)"));\
      Serial.println(F("R# where # is integer time units to record"));\
      Serial.println(F("T# where # is integer time units to record, waiting for trigger"));
      Serial.println(F("   default trigger is LOW"));
      Serial.println(F("f  toggles trigger level LOW <-> HIGH"));
      Serial.println();
      Serial.println(F("?     Print help options"));
      Serial.println();
      Serial.println(F("i###  Sampling interval in us (500 us = 2 kHz default)"));
      Serial.println(F("L###  Logging interval in ms (2000 ms = 0.5 Hz; 0 = sampling, default)"));
      Serial.println(F("n#    Number of input channels (default = 2)"));
      Serial.println();
      Serial.println(F("A     Analog voltage input (Default)"));
      Serial.println(F("C     Digital color sensor, red and blue"));
      Serial.println(F("P     Digital voltage and current (mV, mA)"));
      Serial.println(F("t###  Analog 100k thermistor (x100 degC) ### = ohms for calibration"));
    
      Serial.println();
      Serial.print(F("*** For default, automatic log settings with no Serial input needed, connect GND to pin "));
      Serial.println(DEFAULT_IN);
      Serial.println(F("*** Default settings will be shown. Logging begins upon regular trigger. "));
      Serial.println();
      Serial.println(F("Current settings:"));
      Serial.print("  i"); Serial.print(interval);
      Serial.print("  L"); Serial.print(logging_ms);
      Serial.print("  n"); Serial.print(n);
      Serial.println();
      Serial.print(F("Current mode:  "));
      Serial.println(digital);
//      if (digital) {
//        Serial.println(F("Digital"));
//      }
//      else {
//        Serial.println(F("Analog"));
//      }
      Serial.println();
}

void triggerChange() {
  digitalWrite(TRIGGER, (triggerPolarity == digitalRead(TRIGGER_IN)) ? HIGH : LOW);
}

int get_tempC_x100(int pin, float R1, float beta) {
  const float roomTemp = 298.15;
  const int num_to_avg = 50;
  R1 = 101300.;
  float v0 = 0;
  for(int i=0;i<num_to_avg;i++) v0 += analogRead(pin);
  v0 = v0 / (float)num_to_avg;
  float R2 = R1 / ((( (float)(pow(2,NBIT))-1.0) / v0) - 1.0);
    
  float temp_C = (beta * roomTemp) /(beta + (roomTemp * log(R2 / R1))) - 273.15;
  int temp_x100_C = (int)(100 * temp_C); 
  return temp_x100_C;
}

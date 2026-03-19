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
 *      TRIGGER_IN  pin 3
 *      TRIGGER     pin 4
 *      TRIG_READY  pin 5 (LED --> resistor --> GND)
 *      REC_ON      pin 6 (LED --> resistor --> GND)
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
 *     
 *   Potential changes:  
 *     Save current settings in EEPROM memory for easy automation
 *     some form of median, quartiles, shape distibution,. e.g. by counting  
 */

#include "stats.h"
 
#define VERSION  1.33

#define VIN1 A0
#define VIN2 A1
#define TRIGGER_IN  12
#define TRIGGER 4
#define TRIG_READY 13
#define REC_ON 6
#define DEFAULT_IN 7
#define TEST 9

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
int logging_ms = 0;            // logging interval in ms (0 = same as sampling interval; 2000 = 2s or 0.5Hz)
int secondsToRecord = 0;       // seconds to record for continuous output
int n = 2;                     // number of channels to record
int time_unit_s = 1;           // seconds per time unit

int triggerPolarity = 0;  // 0 = LOW, 1 = HIGH

int val1 = 0;     // input values
int val2 = 0;

bool streamRecording = false; 
unsigned long int start_us = micros();
unsigned long int start_ms = millis();
unsigned long int prev_us = start_us;
unsigned long int prev_log_us = start_us;

//unsigned int data1[MEMLEN];
//unsigned int data2[MEMLEN];
int t = 0;
int dt = 0;

String inputString = "";         // a string to hold incoming data
char inputBytes[INPUT_SIZE + 1];

statistic::Statistic<float, uint32_t, true> data1;
statistic::Statistic<float, uint32_t, true> data2;

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

  //if (NBIT == 12) analogReadResolution(NBIT);
  SET_BITS
  
  Serial.begin(SERIAL_BAUD);       // max serial data rate

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
 
    Serial.print(F("Input string: ")); Serial.println(inputString);

    char first = inputString.charAt(0);
    
    if (first == 'i') {
      char* command = strtok(inputBytes, " ,") + 1;
      interval = atol(command);
      Serial.print(F("Sampling interval changed to "));
      Serial.print(interval);
      Serial.print(F(" us. Sampling rate = "));
      Serial.print(1000. / interval);
      Serial.print(F(" kHz."));
      Serial.println();
    }

    else if (first == 'L') {
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

    else if (first == 'n') {
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

    else if (first == 'R' || first == 'T') {
      char* command = strtok(inputBytes, " ,") + 1;
      secondsToRecord = atol(command) * time_unit_s;
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
    
    else
    {
      showDetails();
    } 

   
  }
  
  if (streamRecording) {
  
    val1 = analogRead(VIN1);
    if (n>1) val2 = analogRead(VIN2);
    
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
      if (n>1) data2.add(val2);

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

        if (n>1) {
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
          Serial.print(data2.rise_count() * 1000. / logging_ms, 1);
          Serial.print(",");
          Serial.print(data2.pop_stdev(),1); 
          Serial.print(",");
          Serial.print(2 * data2.pop_stdev() / data2.range(),2);
        }
        Serial.println();
        
        data1.clear();
        if (n>1) data2.clear();
      }
    
    }

    // if (t_us >= (1000000 * secondsToRecord) + 1000*logging_ms/2) {  // end recording
    if (t_ms >= (1000 * secondsToRecord) + logging_ms/2) {  // end recording
      streamRecording = false; 
      digitalWrite(REC_ON,LOW);
    }
  }
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
        Serial.println(F("Time(s),{instant,mean,min,max,#,freq(Hz),stdev} data1, ... data2"));
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
      Serial.print(F("*** For default, automatic log settings with no Serial input needed, connect GND to pin "));
      Serial.println(DEFAULT_IN);
      Serial.println(F("*** Default settings will be shown. Logging begins upon regular trigger. "));
      Serial.println();
      Serial.println(F("Current settings:"));
      Serial.print("  i"); Serial.print(interval);
      Serial.print("  L"); Serial.print(logging_ms);
      Serial.print("  n"); Serial.print(n);
      Serial.println();
}

void triggerChange() {
  digitalWrite(TRIGGER, (triggerPolarity == digitalRead(TRIGGER_IN)) ? HIGH : LOW);
}

/* FastDataLogMulti
 *   This code rapidly acquires data from 2 voltage inputs and stores 
 *   in memory. Up to three outputs can be controlled via serial port, 
 *   by specifying as comma separated values:
 *      out1 duration, out2 delay, out2 duration, out3 delay, out3 duration
 *      
 *   Additional parameters are: 
 *      interval: sampling interval in us (100 = 10 kHz)
 *      preRecord: us of data logging prior to out1 actuation
 *      postRecord: us of data logging after out1 actuation
 *      
 *   Connect the following pins:   
 *      OUT1 pin 7
 *      OUT2 pin 6
 *      OUT3 pin 5
 *      IN1  pin A0
 *      IN2  pin A1
 *      
 *   Recommended to use with M0 or faster boards. Arduino Uno 
 *   & variants should work but have limited memory (2048 bytes) 
 *   and therefore limited duration. Code tested on Adafruit Metro M0.     
 *   
 *   Updated:
 *     20220519 v1.1 DRA - added second input, sampling rate control and readout
 *     20220601 v1.11 DRA - added Uno support, 12-bit ADC, continuous recording 
 *                    option, and fixed longer interval timing (>1 ms)
 */
 
#define VERSION  1.11

#define OUTPUT1  7
#define OUTPUT2  6
#define OUTPUT3  5
#define VIN1 A0
#define VIN2 A1

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
#endif

#define INPUT_SIZE 32  // max input serial string size
#define MAX_VALS 5     // max values to read in from serial

int interval = 500;            // interval in us (to set rate; 100 = 10kHz)
long int preRecord = 0;        // us to record before switch
long int postRecord = 100000;  // us to record after switch
int secondsToRecord = 0;       // seconds to record for continuous output

bool out1 = 0;     // output states
bool out2 = 0;
bool out3 = 0;
int data1 = 0;     // input values
int data2 = 0;

bool startRecording = false; 
bool streamRecording = false; 
long int start_us = micros();
long int prev_us = start_us;

long int timing[MAX_VALS];
unsigned int us_array[MEMLEN];
unsigned int data_array[MEMLEN];
unsigned int data2_array[MEMLEN];
int t = 0;
int dt = 0;

String inputString = "";         // a string to hold incoming data
char inputBytes[INPUT_SIZE + 1];

void setup() {
  pinMode(OUTPUT1, OUTPUT);
  pinMode(OUTPUT2, OUTPUT);
  pinMode(OUTPUT3, OUTPUT);
  pinMode(VIN1, INPUT);
  pinMode(VIN2, INPUT);

  //if (NBIT == 12) analogReadResolution(NBIT);
  SET_BITS
  
  Serial.begin(SERIAL_BAUD);       // max serial data rate

  delay(500);
  
  showDetails();
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
    
    if ((first >= '0') & (first <= '9')) {
    
      int i = 0;
      char* command = strtok(inputBytes, " ,");
      while (command != 0)
      {
        if (i < MAX_VALS) {
          timing[i] = atol(command) * 1000;  // read in ms timing and convert to us
          Serial.print(i); Serial.print(": "); Serial.println(timing[i]);
        } else {
          Serial.println(F("Max values exceeded. Some ignored."));
        }
          
        // Find the next command in input string
        command = strtok(0, " ,");
        i++;
      }
      Serial.println();
  
      startRecording = true;
  
      //delay(1000);
  
      t = 0;
      start_us = micros();
      prev_us = start_us;
    } 
    
    else if (first == 'i') {
      char* command = strtok(inputBytes, " ,") + 1;
      interval = atol(command);
      Serial.print(F("Sampling interval changed to "));
      Serial.print(interval);
      Serial.print(F(" us. Sampling rate = "));
      Serial.print(1000. / interval);
      Serial.print(F(" kHz."));
      Serial.println();
    }
    
    else if (first == 'p') {
      char* command = strtok(inputBytes, " ,") + 1;
      preRecord = atol(command);
      Serial.print(F("Pre-recording time changed to "));
      Serial.print(preRecord);
      Serial.print(F(" us."));
      Serial.println();
    }
    
    else if (first == 'P') {
      char* command = strtok(inputBytes, " ,") + 1;
      postRecord = atol(command);
      Serial.print(F("Post-recording time changed to "));
      Serial.print(postRecord);
      Serial.print(F(" us."));
      Serial.println();
    }

    else if (first == 'R') {
      char* command = strtok(inputBytes, " ,") + 1;
      secondsToRecord = atol(command);
      Serial.print(F("Continuous streaming for "));
      Serial.print(secondsToRecord);
      Serial.print(F(" seconds."));
      Serial.println();

      streamRecording = true;

      Serial.println(F("Data output format:"));
      Serial.println(F("Time(us),out1,out2,out3,data1,data2"));
      Serial.println();
      Serial.println(F("Logging data..."));

      t = 0;
      start_us = micros();
      prev_us = start_us;
    }
    
    else
    {
      showDetails();
    } 
  }
  
  if (startRecording) {
    
    //--------------------------------------------------------
    // control outputs & log data
   
    data1 = analogRead(VIN1);
    data2 = analogRead(VIN2);
    long int us = micros();
    long int t_us = us - start_us;
    
    int dt = us - prev_us + 5;  // allow 5 us for commands
    if (dt < interval) delayMicroseconds(interval-dt);
    prev_us = micros();

    out1 = (t_us >= preRecord && t_us <= preRecord+timing[0]);
    out2 = (t_us >= preRecord+timing[1] && t_us <= preRecord+timing[2]+timing[1]);
    out3 = (t_us >= preRecord+timing[3] && t_us <= preRecord+timing[4]+timing[3]);

    if (out1) {
      digitalWrite(OUTPUT1, HIGH);
    } else {
      digitalWrite(OUTPUT1, LOW);
    }

    if (out2) {
      digitalWrite(OUTPUT2, HIGH);
    } else {
      digitalWrite(OUTPUT2, LOW);
    }    
    
    if (out3) {
      digitalWrite(OUTPUT3, HIGH);
    } else {
      digitalWrite(OUTPUT3, LOW);
    }

    // log data
    //Serial.print(t_us % 1000);
    //Serial.print(',');
    //Serial.print(out1);
    //Serial.print(',');
    //Serial.print(out2); 
    //Serial.print(',');
    //Serial.print(out3); 
    //Serial.print(out1+2*out2+4*out3);   
    //Serial.print(',');
    //Serial.println(data);

    // Save data to memory 
    us_array[t] = t_us % 1000;
    data_array[t] = data1 + out1*pow(2,NBIT) + out2*pow(2,NBIT+1) + out3*pow(2,NBIT+2);  // add output states as bits to data output
    data2_array[t] = data2;
    t++;

    if (t_us > preRecord+postRecord || t > MEMLEN) {
      startRecording = false;

      Serial.print(t);
      Serial.print(F(" data points collected in "));
      Serial.print(t_us);
      Serial.println(F(" microseconds."));
      Serial.print(F("Average sampling rate: "));
      Serial.print(1000.*t/t_us);
      Serial.println(" kHz");
      Serial.println();

      Serial.println(F("Data output format:"));
      Serial.println(F("Time(us),out1,out2,out3,data1,data2"));
      Serial.println();
      Serial.println(F("Logging data..."));
      float avg_int_us = 0;
      long int t_us = 0;
      for (int i = 0; i<t; i++) {
        // log data
        
//        if (i>1 && (us_array[i] < us_array[i-1])) {
//          t_us = ceil(t_us / 1000.)*1000 + us_array[i];
//        } else {
//          t_us = floor(t_us / 1000.)*1000 + us_array[i];
//        }

        if (i==0) t_us = us_array[i];
        else //if ((us_array[i] + 16) < us_array[i-1]) {
          t_us = us_array[i] + round((t_us+interval-us_array[i]) / 1000.)*1000;
        //} else {
        //  t_us = floor(t_us / 1000.)*1000 + us_array[i] + floor(interval / 1000.)*1000;
        //}
        Serial.print(t_us);
        Serial.print(',');
        Serial.print((int) (data_array[i] >> NBIT) % 2);
        Serial.print(',');
        Serial.print((int) (data_array[i] >> (NBIT+1)) % 2);
        Serial.print(',');
        Serial.print((int) (data_array[i] >> (NBIT+2)) % 2);
        Serial.print(',');
        Serial.print(data_array[i] % (int)pow(2,NBIT));
        Serial.print(',');
        Serial.println(data2_array[i]);
        //Serial.print(',');
        //int dt = us_array[i+1] - us_array[i]; 
        //if (dt < 0) dt = dt + 1000;
        //Serial.println(dt);
        //if (i < (t-1)) avg_int_us = (avg_int_us * (float)(i / (i+1.))) + (float)(dt / (i+1.));
        //Serial.print(',');
        //Serial.println(avg_int_us);   
      }
      
      //Serial.println();
      //Serial.print("*** Average rate: ");
      //Serial.print(1000/avg_int_us);
      //Serial.println(" kHz");
    }
  } 
  else if (streamRecording) {

    data1 = analogRead(VIN1);
    data2 = analogRead(VIN2);
    
    long int us = micros();
    long int t_us = us - start_us;
    
    int dt = us - prev_us + 5;  // allow 5 us for commands
    if (dt < interval) delayMicroseconds(interval-dt);
    prev_us = micros();

    Serial.print(t_us);
    Serial.print(",0,0,0,");
    Serial.print(data1);
    Serial.print(',');
    Serial.println(data2);

    if (t_us >= (1000000 * secondsToRecord)) streamRecording = false; 
  }
  else {
    delay(25);
  }
}

void showDetails() {
  Serial.print(F("FastDataLogMulti v"));
      Serial.println(VERSION);
      Serial.println(F("=========================="));
      Serial.print(F("Records two inputs at pins ")); 
      Serial.print(VIN1); Serial.print(" "); Serial.println(VIN2);
      Serial.print(F("Controls three outputs at pins ")); 
      Serial.print(OUTPUT1); Serial.print(" "); 
      Serial.print(OUTPUT2); Serial.print(" "); 
      Serial.println(OUTPUT3);
      Serial.print(F("Max data points: "));
      Serial.println(MEMLEN);
      Serial.print(F("Serial baud rate: "));
      Serial.println(SERIAL_BAUD);
      Serial.println();
      Serial.println(F("Enter timing using one of the following syntaxes:"));
      Serial.println(F("out1_dur (output 1 duration in ms)"));
      Serial.println(F("out1_dur, out2_delay, out2_dur"));
      Serial.println(F("out1_dur, out2_delay, out2_dur, out3_delay, out3_dur"));
      Serial.println();
      Serial.println(F("?     Print help options"));
      Serial.println();
      Serial.println(F("i###  Sampling interval in us (500 us = 2 kHz default)"));
      Serial.println(F("p###  preRecording us (0 default)"));
      Serial.println(F("P###  postRecording us (100000 default)"));
      Serial.println();
      Serial.println(F("Current settings:"));
      Serial.print("  i"); Serial.print(interval);
      Serial.print(", p"); Serial.print(preRecord);
      Serial.print(", P"); Serial.println(postRecord);
      Serial.println();
      Serial.print(F("Expected data collection: "));
      Serial.print(preRecord+postRecord);
      Serial.print(F(" microseconds at "));
      Serial.print(interval);
      Serial.print(F(" us intervals = "));
      int expected = (preRecord+postRecord)/interval;
      Serial.print(expected);
      Serial.println(F(" data points."));
      if (expected > MEMLEN) {
        Serial.println(F("*** WARNING: NOT ENOUGH MEMORY ***"));
        Serial.print(F("Will only record "));
        Serial.print((long int)interval*MEMLEN);
        Serial.println(F(" microseconds."));
      }
      Serial.println();
}

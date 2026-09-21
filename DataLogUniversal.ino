/* DataLogSummary

* This code rapidly acquires data from 2 voltage inputs and streams to the serial port,
* or calculates summary statistics and reports these at a set interval.
* A fast "stream to memory" mode captures analog data at up to ~30 kHz and then
* writes the buffered values to the serial port after recording completes.
*
* Additional parameters are:
* interval: sampling interval in us (100 = 10 kHz)
* logging_ms: logging interval in ms (2000 = 2s or 0.5Hz)
*
* Connect the following pins:
* IN1 pin A0
* IN2 pin A1
* TRIGGER_IN pin 12
* TRIGGER pin 3   (echoes TRIGGER_IN, possibly polarity-flipped)
* TRIG_READY pin 6 (LED --> resistor --> GND)
* REC_ON pin 4    (LED --> resistor --> GND)
* OUT1 pin 5      (recording-active signal, echoes REC_ON; goes HIGH during any recording)
* DEFAULT_IN pin 8 (connect to GND for default no-program mode)
* TEST pin 7      (PWM test signal)
*
* Recommended to use with M0 or faster boards. Arduino Uno
* & variants should work but have limited memory (2048 bytes)
* and therefore limited duration. Code tested on Adafruit Metro M0.
*
* Updated:
* 20220519 v1.1  DRA - added second input, sampling rate control and readout
* 20220601 v1.11 DRA - added Uno support, 12-bit ADC, continuous recording
*                      option, and fixed longer interval timing (>1 ms)
* 20230905 v1.2  DRA - add trigger option
* 20250221 v1.3  DRA - add summary stats; remove outputs & burst mode
* 20250227 v1.31 DRA - add hardware default via jumper to allow direct logging without serial input
* 20250228 v1.32 DRA - add 2*sd/range as a shape parameter --> 1.0 = square; 0.71 = sine, 0.33 = gaussian noise
* 20250303 v1.33 DRA - change timing to ms-based if in summary mode, to prevent rollover for > 1h logging
* 20260324 v1.35 DRA - change var types to long for compatibility with Uno
* 20260511 v1.4  NS/DRA - add digital logging: TCS34725 color sensor
* 20260601 v1.42 DRA - add 100k thermistor input
* 20260603 v1.43 DRA - add live mode
* 20260604 v1.44 DRA - add defined trigger polarity and analog scaling
* 20260921 v1.50 DRA - add fast stream-to-memory mode (m/M commands); add OUT1 recording signal
*
* Potential changes:
* Save current settings in EEPROM memory for easy automation?
* Add SD card logging?
*/

#include <Wire.h>
#include "stats.h"
#include "Adafruit_TCS34725.h"
#include "INA226.h"

#define VERSION 1.50

#define VIN1 A0
#define VIN2 A1
#define TRIGGER_IN  12
#define TRIGGER      3
#define TRIG_READY   6  // LED
#define REC_ON       4  // LED
#define OUT1         5  // recording-active signal (echoes REC_ON, not hardwired to LED)
#define DEFAULT_IN   8  // connect to GND for default (no-program mode)
#define TEST         7  // PWM test signal

#define SERIAL_BAUD 500000

// Settings based on board used
#if defined(ARDUINO_SAMD_ZERO)
  #define MEMLEN 2000       // max size of data memory
  #define NBIT 12           // Analog in number of bits
  #define SET_BITS analogReadResolution(NBIT);
#elif defined(__AVR_ATmega328P__)
  #define MEMLEN 250        // max size of data memory
  #define NBIT 10           // Analog in number of bits
  #define SET_BITS
#else
  #warning "Board not recognized. Add specs to code."
  #define MEMLEN 2000
  #define NBIT 12
  #define SET_BITS analogReadResolution(NBIT);
#endif

// Timestamp modulus for memory mode.
// Storing t_us % US_MOD fits in uint16_t (max 65535).
// 10000 us (10 ms) windows are human-readable and prevent false rollovers
// from jitter as long as jitter << 10 ms (easily satisfied in practice).
#define US_MOD 10000

#define INPUT_SIZE 32   // max input serial string size
#define MAX_VALS 5      // max values to read in from serial

// --- Timing / recording parameters ---
int interval         = 500;   // sampling interval in us (500 us = 2 kHz)
long logging_ms      = 0;     // logging interval in ms (0 = stream every sample)
long secondsToRecord = 0;     // seconds to record for continuous output
int n                = 2;     // number of channels to record
long time_unit_s     = 1;     // seconds per time unit
int triggerPolarity  = 0;     // 0 = LOW, 1 = HIGH

// --- Sensor values ---
uint16_t val1 = 0;
uint16_t val2 = 0;

// --- Sensor / scaling options ---
int digital = 0;              // <=0 analog, >0 digital sensor
float tempResistance = 100000.0;
float beta           = 3950.0;
float scaling        = 1.;
uint16_t zero_offset = 0;

// --- Mode flags ---
bool streamRecording = false;
bool liveStream      = false;
bool memoryMode      = false;   // true when in m or M memory-capture mode
bool memoryTriggered = false;   // true when M (triggered) variant

// --- Memory-mode arrays ---
// Always allocated; used only during memoryMode.
uint16_t us_array[MEMLEN];    // t_us % US_MOD per sample
uint16_t data_array[MEMLEN];  // channel 1 samples
uint16_t data2_array[MEMLEN]; // channel 2 samples
int memPoints = MEMLEN;       // number of points to capture (≤ MEMLEN)

// --- Timing state ---
unsigned long int start_us   = 0;
unsigned long int start_ms   = 0;
unsigned long int prev_us    = 0;
unsigned long int prev_log_us = 0;
int readout_ms = 0;

// --- Misc ---
int t  = 0;
int dt = 0;
String inputString = "";
char inputBytes[INPUT_SIZE + 1];

// --- Summary-stats objects ---
statistic::Statistic<float, uint32_t, true> data1stat;
statistic::Statistic<float, uint32_t, true> data2stat;

/*---------------------------------------------------*/
// Digital I2C sensors
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
INA226 INA(0x40);
/*---------------------------------------------------*/

/* ================================================================
   setup
   ================================================================ */
void setup() {
  pinMode(TRIGGER_IN, INPUT_PULLUP);
  pinMode(DEFAULT_IN, INPUT_PULLUP);
  pinMode(VIN1, INPUT);
  pinMode(VIN2, INPUT);
  pinMode(TRIGGER,    OUTPUT);
  pinMode(REC_ON,     OUTPUT);
  pinMode(TRIG_READY, OUTPUT);
  pinMode(OUT1,       OUTPUT);
  pinMode(TEST,       OUTPUT);

  analogWrite(TEST, 128);  // test signal = 50% duty cycle

  attachInterrupt(digitalPinToInterrupt(TRIGGER_IN), triggerChange, CHANGE);

  SET_BITS

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  delay(500);

  showDetails();
  triggerChange();
}

/* ================================================================
   loop
   ================================================================ */
void loop() {

  /* ---- Serial command handling ---- */
  if (Serial.available()) {

    if (liveStream) {
      liveStream       = false;
      streamRecording  = false;
      Serial.flush();
    }
    if (memoryMode) {
      // Abort any in-progress memory capture
      memoryMode      = false;
      memoryTriggered = false;
      setRecordingOff();
    }

    byte size = Serial.readBytes(inputBytes, INPUT_SIZE);
    inputBytes[size] = 0;
    inputString = String(inputBytes);

    Serial.print("[");
    Serial.write(inputString.c_str(), inputString.length() - 1);
    Serial.print("] ");

    char first = inputString.charAt(0);

    /* -- Sampling interval -- */
    if (first == 'i') {
      char* command = strtok(inputBytes, " ,") + 1;
      interval = atol(command);
      Serial.print(F("Sampling interval changed to "));
      Serial.print(interval);
      Serial.print(F(" us. Sampling rate = "));
      Serial.print(1000. / interval);
      Serial.println(F(" kHz."));
    }

    /* -- Logging interval -- */
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

    /* -- Number of channels -- */
    else if (first == 'n') {
      char* command = strtok(inputBytes, " ,") + 1;
      n = atol(command);
      Serial.print(F("Sample channels changed to "));
      Serial.println(n);
    }

    /* -- Trigger polarity -- */
    else if (first == 'f') {
      char* command = strtok(inputBytes, " ,") + 1;
      triggerPolarity = (atol(command) == 1);
      Serial.print(F("Trigger polarity changed to "));
      Serial.println(triggerPolarity ? "HIGH" : "LOW");
      triggerChange();
    }

    /* -- Time unit -- */
    else if (first == 'U') {
      char second = inputString.charAt(1);
      Serial.print(F("Time unit changed to: "));
      if      (second == 'm') { time_unit_s = 60;          Serial.println(F("minutes.")); }
      else if (second == 'h') { time_unit_s = 3600;        Serial.println(F("hours."));   }
      else if (second == 'd') { time_unit_s = 86400;       Serial.println(F("days."));    }
      else                    { time_unit_s = 1;           Serial.println(F("seconds.")); }
    }

    /* -- Record (R = immediate, T = triggered) -- */
    else if (first == 'R' || first == 'T') {
      char* command = strtok(inputBytes, " ,") + 1;
      secondsToRecord = (long)atol(command) * time_unit_s;
      Serial.print(F("Continuous streaming for "));
      Serial.print(secondsToRecord);
      Serial.print(F(" seconds. "));
      if (first == 'T') {
        Serial.print(F("Waiting for trigger..."));
        waitForTrigger();
      }
      Serial.println();
      startLogging();
    }

    /* -- Live mode -- */
    else if (first == 'G') {
      secondsToRecord = 86400;  // 1 day max
      liveStream = true;
      startLogging();
    }

    /* -- Memory mode: m = immediate, M = triggered --
     * Optional point count: m500 or M1000 (capped at MEMLEN)
     */
    else if (first == 'm' || first == 'M') {
      // Parse optional point count from remainder of command
      char* rest = inputBytes + 1;
      long requested = atol(rest);
      if (requested > 0 && requested < MEMLEN) {
        memPoints = (int)requested;
      } else {
        memPoints = MEMLEN;
      }

      memoryTriggered = (first == 'M');
      memoryMode      = true;

      Serial.print(F("Memory mode: capturing "));
      Serial.print(memPoints);
      Serial.print(F(" points at "));
      Serial.print(1000. / interval);
      Serial.print(F(" kHz (~"));
      Serial.print((long)memPoints * interval / 1000);
      Serial.print(F(" ms). Analog only. "));
      if (memoryTriggered) {
        Serial.print(F("Waiting for trigger..."));
        waitForTrigger();
      }
      Serial.println();

      startMemoryCapture();
    }

    /* -- Stop / Abort -- */
    else if (first == 'X' || first == 'x') {
      streamRecording  = false;
      memoryMode       = false;
      memoryTriggered  = false;
      setRecordingOff();
    }

    /* -- Digital sensor: Color -- */
    else if (first == 'C') {
      if (tcs.begin()) {
        Serial.println(F("Color Sensor TCS34725 detected. Mode changed to digital."));
        digital = 1;
      } else {
        Serial.println(F("Color Sensor TCS34725 NOT detected."));
      }
    }

    /* -- Digital sensor: Power -- */
    else if (first == 'P') {
      if (INA.begin()) {
        Serial.println(F("INA226 detected. Mode changed to digital. Voltage (mV) and current (0.1 mA)"));
        INA.setMaxCurrentShunt(0.5, 0.100);
        digital = 2;
      } else {
        Serial.println(F("INA226 NOT detected."));
      }
    }

    /* -- Analog mode -- */
    else if (first == 'A') {
      digital = 0;
      Serial.println(F("Mode changed to analog."));
    }

    /* -- Analog scaled mode -- */
    else if (first == 'a') {
      char* command = strtok(inputBytes, " ,") + 1;
      zero_offset = (command != 0) ? atol(command) : 0;
      command = strtok(0, " ,");
      scaling = (command != 0) ? atof(command) : 1.;
      digital = -1;
      Serial.print(F("Mode changed to analog, scaled: y = "));
      Serial.print(scaling);
      Serial.print(F("(x - "));
      Serial.print(zero_offset);
      Serial.println(")");
    }

    /* -- Thermistor mode -- */
    else if (first == 't') {
      char* command = strtok(inputBytes, " ,") + 1;
      tempResistance = (float)atol(command) * 100.;
      Serial.print(F("Mode changed to analog, thermistor. Resistance @ 25C: "));
      Serial.print(tempResistance / 1000.);
      Serial.println(F(" kOhms"));
      digital = -1;
      if (interval < 5000) interval = 5000;
    }

    /* -- Help / unknown -- */
    else {
      showDetails();
    }
  }

  /* ================================================================
     MEMORY MODE EXECUTION
     Runs a complete capture-then-dump cycle each time through loop()
     while memoryMode is true.
     ================================================================ */
  if (memoryMode) {

    /* --- Capture phase --- */
    // startMemoryCapture() already set up timing; run the tight loop here.
    // (Re-entry after re-arm falls through to this block immediately.)

    t = 0;
    start_us = micros();
    prev_us  = start_us;

    while (t < memPoints) {
      // Check for abort
      if (Serial.available()) {
        // Peek: if 'X' or 'x', abort
        char c = Serial.peek();
        if (c == 'X' || c == 'x') {
          Serial.read(); // consume
          memoryMode      = false;
          memoryTriggered = false;
          setRecordingOff();
          Serial.println(F("Memory capture aborted."));
          return;
        }
      }

      unsigned long int us    = micros();
      unsigned long int t_us  = us - start_us;

      // Pace to interval
      int elapsed = (int)(us - prev_us) + 5;
      if (elapsed < interval) delayMicroseconds(interval - elapsed);
      prev_us = micros();

      // Sample (analog only in memory mode)
      us_array[t]   = (uint16_t)(t_us % US_MOD);
      data_array[t] = analogRead(VIN1);
      if (n > 1) data2_array[t] = analogRead(VIN2);

      t++;
    }

    /* --- Recording done: signal off --- */
    setRecordingOff();

    /* --- Dump phase --- */
    Serial.print(t);
    Serial.print(F(" points captured. Average rate: "));
    unsigned long int total_us = (unsigned long int)(t - 1) * interval; // approximate
    Serial.print(1000. * t / (micros() - start_us + (unsigned long int)interval), 2);
    Serial.println(F(" kHz"));
    Serial.println();
    Serial.println(F("Time(us),ch1,ch2"));

    // Reconstruct full timestamps from US_MOD-modulo offsets
    unsigned long int t_reconstructed = 0;
    uint16_t prev_offset = 0;
    long window = 0; // which US_MOD window we are in

    for (int i = 0; i < t; i++) {
      uint16_t offset = us_array[i];

      if (i == 0) {
        t_reconstructed = offset;
      } else {
        // Rollover: offset decreased relative to previous sample
        if (offset < prev_offset) {
          window++;
        }
        t_reconstructed = (unsigned long int)window * US_MOD + offset;
      }
      prev_offset = offset;

      Serial.print(t_reconstructed);
      Serial.print(',');
      Serial.print(data_array[i]);
      if (n > 1) {
        Serial.print(',');
        Serial.print(data2_array[i]);
      }
      Serial.println();
    }

    Serial.println(); // blank line separates bouts

    /* --- Re-arm or exit --- */
    if (memoryTriggered) {
      // Stay in memoryMode; wait for next trigger then loop
      Serial.println(F("Waiting for next trigger... (X to stop)"));
      waitForTrigger();
      // Signal recording-active again (startMemoryCapture sets pins)
      startMemoryCapture();
      // Loop back to capture block at top of next loop() iteration
    } else {
      memoryMode = false;
    }

    return; // skip stream-recording block this iteration
  }

  /* ================================================================
     STREAM RECORDING EXECUTION (unchanged from v1.44)
     ================================================================ */
  if (streamRecording) {

    // Read current sensor values
    switch (digital) {
      case 0:
        val1 = analogRead(VIN1);
        if (n > 1) val2 = analogRead(VIN2);
        break;
      case 1: {
        uint16_t g, c;
        tcs.getRawData(&val1, &g, &val2, &c);
        break;
      }
      case 2:
        val1 = (uint16_t)(INA.getBusVoltage() * 1000);
        val2 = (uint16_t)(INA.getCurrent_mA() * 10);
        break;
      case -1:
        val1 = (uint16_t)((analogRead(VIN1) - zero_offset) * scaling);
        if (n > 1) val2 = (uint16_t)((analogRead(VIN2) - zero_offset) * scaling);
        break;
      case -2:
        val1 = get_tempC_x100(VIN1, tempResistance, beta);
        if (n > 1) val2 = get_tempC_x100(VIN2, tempResistance, beta);
        break;
      default:
        break;
    }

    unsigned long int ms   = millis();
    unsigned long int t_ms = ms - start_ms;
    unsigned long int us   = micros();
    unsigned long int t_us = us - start_us;

    unsigned int dt = us - prev_us + 5;
    if (dt < interval) delayMicroseconds(interval - dt);
    prev_us = micros();

    if (logging_ms == 0 || liveStream) {
      // Fast stream: one line per sample
      if (!liveStream) {
        Serial.print(t_us);
        Serial.print(",");
      }
      Serial.print(val1);
      if (n > 1) {
        Serial.print(",");
        Serial.print(val2);
      }
      Serial.println();

    } else {
      // Summary logging: accumulate stats, print at logging interval
      data1stat.add(val1);
      if (n > 1 || digital > 0) data2stat.add(val2);

      if ((us - prev_log_us) >= (unsigned long int)logging_ms * 1000) {
        prev_log_us += (unsigned long int)logging_ms * 1000;

        Serial.print((t_ms / 1000.), 1);
        Serial.print(", ");
        Serial.print(val1);
        Serial.print(","); Serial.print(data1stat.average(), 1);
        Serial.print(","); Serial.print(data1stat.minimum(), 0);
        Serial.print(","); Serial.print(data1stat.maximum(), 0);
        Serial.print(","); Serial.print(data1stat.count());
        Serial.print(","); Serial.print(data1stat.rise_count() * 1000. / logging_ms, 1);
        Serial.print(","); Serial.print(data1stat.pop_stdev(), 1);
        Serial.print(","); Serial.print(2 * data1stat.pop_stdev() / data1stat.range(), 2);

        if (n > 1 || digital > 0) {
          Serial.print(", ");
          Serial.print(val2);
          Serial.print(","); Serial.print(data2stat.average(), 1);
          Serial.print(","); Serial.print(data2stat.minimum(), 0);
          Serial.print(","); Serial.print(data2stat.maximum(), 0);
          Serial.print(","); Serial.print(data2stat.count());
          Serial.print(","); Serial.print(data2stat.rise_count() * 1000. / (logging_ms - readout_ms), 1);
          Serial.print(","); Serial.print(data2stat.pop_stdev(), 1);
          Serial.print(","); Serial.print(2 * data2stat.pop_stdev() / data2stat.range(), 2);
        }
        Serial.println();

        data1stat.clear();
        if (n > 1) data2stat.clear();
        readout_ms = millis() - ms;
      }
    }

    // Check end-of-recording
    if (t_ms >= (unsigned long int)(1000 * secondsToRecord) + logging_ms / 2) {
      streamRecording = false;
      setRecordingOff();
      Serial.println(F("END"));
    }

  } // end streamRecording

  /* ================================================================
     IDLE
     ================================================================ */
  else {
    setRecordingOff();
    delay(25);

    // Default jumper: auto-configure summary logging and wait for trigger
    if (digitalRead(DEFAULT_IN) == LOW) {
      Serial.print(F("Default triggered logging: "));
      n               = 1;
      interval        = 500;
      logging_ms      = 2000;
      secondsToRecord = 86400;
      triggerPolarity = 0;
      Serial.print(secondsToRecord);
      Serial.print(" ");
      showLoggingInfo();
      waitForTrigger();
      startLogging();
    }
  }
}

/* ================================================================
   HELPER FUNCTIONS
   ================================================================ */

void setRecordingOff() {
  digitalWrite(REC_ON, LOW);
  digitalWrite(OUT1,   LOW);
}

void setRecordingOn() {
  digitalWrite(REC_ON, HIGH);
  digitalWrite(OUT1,   HIGH);
}

void startMemoryCapture() {
  setRecordingOn();
  // Timing is re-initialized at the top of the capture while-loop in loop()
}

void startLogging() {
  streamRecording = true;
  setRecordingOn();

  if (!liveStream) {
    Serial.println(F("Data output format:"));
    if (logging_ms == 0) {
      Serial.println(F("Time(us),data1,data2"));
    } else {
      Serial.println(F("Time(s),{instant,mean,min,max,#,freq(Hz),stdev,shape:sq1.0,sin0.7,tri0.6,n0.3} data1, ... data2"));
    }
    Serial.println();
    Serial.println(F("Logging data..."));
  }

  t            = 0;
  start_ms     = millis();
  start_us     = micros();
  prev_us      = start_us;
  prev_log_us  = start_us;
}

void waitForTrigger() {
  digitalWrite(TRIG_READY, HIGH);
  while (digitalRead(TRIGGER_IN) != triggerPolarity) {
    delay(25);
  }
  digitalWrite(TRIG_READY, LOW);
}

void showLoggingInfo() {
  Serial.print(1000. / interval, 1);
  Serial.print(F(" kHz, log every "));
  Serial.print(logging_ms / 1000., 1);
  Serial.print(F("s for "));
  Serial.print(secondsToRecord);
  Serial.print(F("s = "));
  Serial.print(secondsToRecord / 60);
  Serial.print(F("min = "));
  Serial.print(secondsToRecord / 3600);
  Serial.println(F("hr."));
}

void showDetails() {
  Serial.print(F("DataLogSummary v"));
  Serial.println(VERSION);
  Serial.println(F("=========================="));
  Serial.print(F("Analog inputs: A0 (ch1), A1 (ch2). Digital sensors via I2C."));
  Serial.println();
  Serial.print(F("Trigger input: pin ")); Serial.println(TRIGGER_IN);
  Serial.print(F("Max memory points: ")); Serial.println(MEMLEN);
  Serial.print(F("Serial baud rate: ")); Serial.println(SERIAL_BAUD);
  Serial.println();

  Serial.println(F("--- Continuous stream / summary modes ---"));
  Serial.println(F("U{s/m/h/d}  Time unit: [s]econds, [m]inutes, [h]ours, [d]ays"));
  Serial.println(F("R#          Record for # time units (immediate)"));
  Serial.println(F("T#          Record for # time units (wait for trigger)"));
  Serial.println(F("G           Live mode (no timestamp; use Serial Plotter; any key to stop)"));
  Serial.println(F("f#          Trigger polarity: 0=LOW (default), 1=HIGH"));
  Serial.println();

  Serial.println(F("--- Fast memory mode (analog only) ---"));
  Serial.println(F("m[#]        Capture to memory immediately; optional # = point count (default=MEMLEN)"));
  Serial.println(F("M[#]        Capture to memory on trigger; re-arms after each dump until X"));
  Serial.println(F("            Output: Time(us), ch1 [, ch2]"));
  Serial.println();

  Serial.println(F("X / x       STOP / Abort any recording"));
  Serial.println();

  Serial.println(F("--- Sampling parameters ---"));
  Serial.println(F("i###   Sampling interval in us (500 us = 2 kHz default)"));
  Serial.println(F("L###   Logging interval in ms  (0 = per-sample; 2000 = 0.5 Hz summary)"));
  Serial.println(F("n#     Number of input channels (1 or 2)"));
  Serial.println();

  Serial.println(F("--- Sensor mode ---"));
  Serial.println(F("A         Analog voltage input (default)"));
  Serial.println(F("a#,#      Analog scaled: zero_offset, scaling"));
  Serial.println(F("C         Digital color sensor (TCS34725): red, blue"));
  Serial.println(F("P         Digital power sensor (INA226): voltage (mV), current (0.1 mA)"));
  Serial.println(F("t###      Thermistor: ### = resistance x10 kΩ at 25C (e.g. 1000 = 100 kΩ)"));
  Serial.println();

  Serial.print(F("Default auto-log: connect GND to pin ")); Serial.println(DEFAULT_IN);
  Serial.println(F("(Summary logging; starts on trigger; no serial input needed.)"));
  Serial.println();

  Serial.println(F("Current settings:"));
  Serial.print(F("  i")); Serial.print(interval);
  Serial.print(F("  L")); Serial.print(logging_ms);
  Serial.print(F("  n")); Serial.print(n);
  Serial.print(F("  memPoints=")); Serial.print(memPoints);
  Serial.println();
  Serial.print(F("  Sensor mode: ")); Serial.println(digital);
  Serial.println();
}

void triggerChange() {
  digitalWrite(TRIGGER, (triggerPolarity == digitalRead(TRIGGER_IN)) ? HIGH : LOW);
}

int get_tempC_x100(int pin, float R1, float beta) {
  const float roomTemp = 298.15;
  const float R0       = 100000;
  const int   num_to_avg = 50;
  float v0 = 0;
  for (int i = 0; i < num_to_avg; i++) v0 += analogRead(pin);
  v0 /= (float)num_to_avg;
  float R2     = R1 / (((float)(pow(2, NBIT)) - 1.0) / v0 - 1.0);
  float temp_C = (beta * roomTemp) / (beta + roomTemp * log(R2 / R0)) - 273.15;
  return (int)(100 * temp_C);
}

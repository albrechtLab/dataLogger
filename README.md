# dataLogger
Arduino and compatible data loggers, simple and scriptable

Reads 2 analog inputs and controls up to 3 digital outputs with microsecond precision. Data readout can be fast (memory storage) or continuous. 

Can be uploaded to Arduino Uno, Metro M0, Metro M4, etc. with configurable input and output pin numbers (FastDataLogMulti1.11.ino)

Speed characteristics:
* Uno:       250 data points memory up to 2.8 kHz, continuous up to 1 kHz 
* Metro M0: 2000 data points memory up to >15 kHz, continuous up to ?? kHz
* Metro M4: not tested 

Scriptable via serial port control. Use the MATLAB functions to automatically stimulate and record (BlastMicrochamberTestMultiNew.m) or for continuous data collection for a prescribed duration (PinballPressureLog.m).

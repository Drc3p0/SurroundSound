//volume controlled by ultrasonic sensor.  


//Ultrasonic Sensor SoftPing  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <NewPing.h>
#define PING_PIN 14
#define MAX_DISTANCE 200 //in inches...max is 400-500cm
NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE); //NewPing setup of pins and max distance.
const unsigned int SECOND = 1000;
const unsigned int PING_CHECK_INTERVAL = SECOND / 10;  //how often the ultrasonic sensor is checked
unsigned long currentTime;
unsigned long nextTime = 0;

//Teensy Audio System Design Tool 
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
AudioInputI2S            audioInput;           //xy=190,281
AudioAnalyzeNoteFrequency notefreq1;      //xy=345,357
AudioSynthWaveformSineModulated sine_fm1;       //xy=366,288
AudioOutputI2S           audioOutput;           //xy=547,292
AudioConnection          patchCord1(audioInput, 1, notefreq1, 0);
//AudioConnection          patchCord2(audioInput, 1, sine_fm1, 0);
//AudioConnection          patchCord3(sine_fm1, 0, audioOutput, 1);
AudioConnection          patchCord2(audioInput, 1, audioOutput, 1);
//end autogenerated code
AudioControlSGTL5000 audioShield;
const float MAX_RAW_VOLUME = 20;
unsigned int raw_vol = 0;
float vol = 0.0;

void setup() {
  Serial.begin(115200);  //higher baud for ping
  AudioMemory(10);   // detailed information, see the MemoryAndCpuUsage example
  audioShield.enable();
 // audioShield.inputSelect(AUDIO_INPUT_LINEIN);
  audioShield.inputSelect(AUDIO_INPUT_MIC);
 // audioShield.volume(vol);

}

void loop() {
  unsigned int raw_interactive_vol = 0;

  currentTime = millis();

  //begin(threshold); //Initialize and start detecting frequencies, with an initial threshold (the amount of allowed uncertainty).
  //available(); //Returns true (non-zero) when a valid frequency is detected.
  //read(); //Read the detected frequency.

  //Grab Ultrasonic sensor values and apply it to the audio volume
  if (currentTime >= nextTime) {  //true the first time
    nextTime = currentTime + PING_CHECK_INTERVAL; 
    byte rawPing = sonar.ping_in(); //raw ultrasonic value
        if (rawPing == 0 or rawPing > 25) { //if sensor is within the detection range
         raw_interactive_vol = MAX_RAW_VOLUME / 2; 
         } 
          else {
           raw_interactive_vol = constrain(rawPing, 1, MAX_RAW_VOLUME);
           }

    if (raw_vol < raw_interactive_vol) {  //ramp up/down to sensor volume 
      raw_vol += 1;
      } 
        else if (raw_vol > raw_interactive_vol) {
        raw_vol -= 1;
        }

    vol = raw_vol / MAX_RAW_VOLUME;
     audioShield.volume(vol);  //final sound output.  list here, and not in setup

    
    Serial.print("   raw_interactive_vol=");
    Serial.print(raw_interactive_vol); 
    Serial.print("   raw_vol=");
    Serial.print(raw_vol); 
    Serial.print("   vol=");
    Serial.print(vol); 
    Serial.print("  rawPing=");
    Serial.println(rawPing);
  } 
}


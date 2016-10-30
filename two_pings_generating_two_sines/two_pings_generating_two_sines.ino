//2 ping sensors playing 2 sine waves for a dual handed theremin.

//Ultrasonic Sensor SoftPing  https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <NewPing.h>
#define MAX_DISTANCE 200 //in inches...max is 400-500cm
#define SONAR_NUM 2 //number of ping sensors
#define PING_PIN_A 14 //echo and trigger are combined
#define PING_PIN_B 20
#define PING_INTERVAL 10 //milliseconds between sensor pings (29ms is about the mid to avoid cross-sensor echo).
//NewPing setup of pins and max distance.  should be calling 0 and 1
NewPing sonar1(PING_PIN_A, PING_PIN_A, MAX_DISTANCE);  //sonar[1] for amplitude
NewPing sonar2(PING_PIN_B, PING_PIN_B, MAX_DISTANCE);  //sonar[2] for frequency

//timing and sensor vals
const unsigned int SECOND = 1000;
const unsigned int PING_CHECK_INTERVAL = SECOND / 75;  //how often the ultrasonic sensor is checked
unsigned long currentTime;
unsigned long nextTime = 0;
unsigned int rawSine2 = 0;
unsigned int rawSine1 = 0;
byte rawPing1 = 0;
byte rawPing2 = 0;
byte whichPing = 1;

//Teensy Audio System Design Tool
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
// GUItool: begin automatically generated code
AudioInputI2S            Input;           //xy=158,120
AudioSynthWaveformSineModulated sine_fm1;       //xy=206,269
AudioSynthWaveformSineModulated sine_fm2;       //xy=224,319
AudioMixer4              mixer;         //xy=406,197
AudioOutputI2S           Output;           //xy=547,292
AudioConnection          patchCord1(Input, 1, mixer, 0);
AudioConnection          patchCord2(sine_fm1, 0, mixer, 1);
AudioConnection          patchCord3(sine_fm2, 0, mixer, 2);
AudioConnection          patchCord4(mixer, 0, Output, 1);
// GUItool: end automatically generated code
AudioControlSGTL5000 audioShield;
float vol = 0.5;
const float MAX_RAW_VOLUME = 20;
unsigned int raw_vol = 0;
unsigned int raw_interactive_vol = 0;
unsigned int smoothSine = 0;
unsigned int Sine_ramper = 0;


void setup() {
        AudioMemory(100); // detailed information, see the MemoryAndCpuUsage example
        audioShield.enable();
        audioShield.inputSelect(AUDIO_INPUT_LINEIN);
        //audioShield.inputSelect(AUDIO_INPUT_MIC); // audioShield.inputSelect(AUDIO_INPUT_LINEIN);
        //pinMode(LED_BUILTIN, OUTPUT);
        //Serial.begin(115200); //higher baud for ping
        //Serial1.begin(115200);
        // Configure the PWM bit depth & rate for the Teensy to 5 (0-31) and 125 kHz (LCM of highest supported rates given clock speed)
        analogWriteResolution(5);
        analogWriteFrequency(3, 125000);  // setting this on pin 3 will also affect pin 4
}

void loop() {
        //set timing checks, Grab Ultrasonic sensor values and apply it to the frequency and volume
        currentTime = millis();
        if (currentTime >= nextTime) { //will always be true the first time...Timer incrememnter
                // Serial.println (nextTime);
                nextTime = currentTime + PING_CHECK_INTERVAL;
                if (whichPing ==1) {
                        whichPing = 2;
                        rawPing1 = sonar1.ping_cm(); //raw ultrasonic value
                        //Serial.print ("rawPing1=  ");
                        //Serial.print (rawPing1);
                        analogWrite(3, rawPing1);

                      //sine generator interjected into 1st ping check
                        if (rawPing1 >= 1 && rawPing1 <= 30) { //if sensor is triggered within the detection range
                                rawSine1 =  map(rawPing1, 0, 30, 20, 250);//map it to the frequency range desired...  could this be done on a logarithmic curve??
                                
                                //sine_fm1.frequency(rawSine1);
                                //sine_fm1.phase(0); //angle 0-360                             
                                sine_fm1.amplitude(1.0);  //Set the amplitude, from 0 to 1.0.
                                
                                //inserted trying to ramp sine
                      //           if (Sine_ramper < rawSine1) { //ramp up/down to sensor volume
                      //                  Sine_ramper += 1;}
                      //           else if (Sine_ramper > rawSine1) {
                      //               Sine_ramper -= 1; }
                        }
                        else {
                                //Sine_ramper = 0;  //if sensor is outside of range, set sine to 0
                                rawSine1 = 0;

                        }
                        //Serial.print(" rawSine1= ");
                        //Serial.print (rawSine1);
                        sine_fm1.frequency(rawSine1);
                        //sine_fm1.frequency(Sine_ramper);
                        //Serial.print (" Sine_ramper ");
                        //Serial.print (Sine_ramper);
                     


                }
                //get values from ping 2
                else {
                        rawPing2 = sonar2.ping_cm(); //raw ultrasonic value
                        //Serial.print ("rawPing2=  ");
                        //Serial.print (rawPing2);
                        whichPing = 1;
                        analogWrite(4, rawPing2);

                        //sine generator interjected into 2nd ping check
                        if (rawPing2 >= 1 && rawPing2 <= 30) { //if sensor is triggered within the detection range
                                rawSine2 =  map(rawPing2, 0, 30, 20, 250);//map it to the frequency range desired...  could this be done on a logarithmic curve??
                                
                                //sine_fm1.frequency(rawSine2);
                                //sine_fm1.phase(0); //angle 0-360

                                sine_fm1.amplitude(1.0);  //Set the amplitude, from 0 to 1.0.
                                
                                //inserted trying to ramp sine
                      //           if (Sine_ramper < rawSine2) { //ramp up/down to sensor volume
                      //                  Sine_ramper += 1;}
                      //           else if (Sine_ramper > rawSine2) {
                      //               Sine_ramper -= 1; }
                        }
                        else {
                                //Sine_ramper = 0;  //if sensor is outside of range, set sine to 0
                                rawSine2 = 0;

                        }
                        //Serial.print(" rawSine2= ");
                        //Serial.println (rawSine2);
                        sine_fm1.frequency(rawSine2);
                        //sine_fm1.frequency(Sine_ramper);
                        //Serial.print (" Sine_ramper ");
                        //Serial.print (Sine_ramper);
                     
                }
        }

}

/*

   sine_fm1.amplitude(level); //Set the amplitude, from 0 to 1.0.
   sine_fm1.frequency(freq);
   Set the center frequency, from 0 to 11000. The output will be this center frequency when the input modulation signal is zero.
   Modulation input 1.0 causes the frequency to double, and input -1.0 causes zero Hz (DC) output.
   For less modulation, attenuate the input signal (perhaps with a mixer object) before it arrives here.
   sine_fm1.phase(angle); 0-360
   Cause the generated waveform to jump to a specific point within its cycle. Angle is from 0 to 360 degrees.
   When multiple objects are configured, AudioNoInterrupts() should be used to guarantee all new settings take effect together.
 */

/**
  Moth Sonic Art Framework Firmware
  Written by Nathan Villicana-Shaw in 2020
  The runtime, boot, and all other configurations are found in the Configuration.h file
*/
// configuration needs to be added first to determine what mode will be added
#include "Configuration.h"

// mechanisms needs be be added before the Mode file
#if ARTEFACT_TYPE == EXPLORATOR
#include <Mechanisms.h>
#include <PlaybackEngine.h>
#endif

/////////////////////////// Include the appropiate Mode.h file //////////////
#if FIRMWARE_MODE == CICADA_MODE && ARTEFACT_TYPE == SPECULATOR
#include "ModeCicada.h"
#elif FIRMWARE_MODE == PITCH_MODE && ARTEFACT_TYPE == SPECULATOR
#include "ModePitch.h"
#endif

////////////////////////////// Libraries needed for Every Artefact /////////////////////////////////////
#include <LuxManager.h>
#include <PrintUtils.h>
#include <UIManager.h>
#include <WS2812Serial.h>
#include <NeopixelManager.h>
#include <Audio.h>
#include <AudioEngine.h>
#include <Wire.h>
#include <SPI.h>

///////////////////////////// Libraries only needed for certain Artefacts ///////////////////////////////
#if (ARTEFACT_TYPE == SPECULATOR && HV_MAJOR == 3) || (ARTEFACT_TYPE == EXPLORATOR)
#define WEATHER_MANAGER_PRESENT        true
#include <WeatherManager.h>
#else
#define WEATHER_MANAGER_PRESENT        false
#endif

#if ARTEFACT_TYPE == EXPLORATOR && BODY_TYPE == WOODPECKER_BODY
// TODO - update to the way that motors are taken care of in the main_simple program

//////////////////////////////////////////////////////////////////////////
///////////////////// H-Bridge Motor Driver //////////////////////////////
//////////////////////////////////////////////////////////////////////////

#include <MAX14870Motors.h>

MAX14870Motors motors = MAX14870Motors(MOT1_DIR_PIN, MOT1_PWM_PIN,
                                       MOT2_DIR_PIN, MOT2_PWM_PIN,
                                       MOT3_DIR_PIN, MOT3_PWM_PIN,
                                       MOT1_EN_PIN, MOT2_EN_PIN, MOT3_EN_PIN,
                                       MOT1_FAULT_PIN, MOT2_FAULT_PIN, MOT3_FAULT_PIN);
/*
  DualMAX14870MotorShield motors[3] = {DualMAX14870MotorShield(MOT1_DIR_PIN, MOT1_PWM_PIN, 24, 24, MOT1_EN_PIN, MOT1_FAULT_PIN),
                                     DualMAX14870MotorShield(MOT2_DIR_PIN, MOT2_PWM_PIN, 24, 24, MOT2_EN_PIN, MOT2_FAULT_PIN),
                                     DualMAX14870MotorShield(MOT3_DIR_PIN, MOT3_PWM_PIN, 24, 24, MOT3_EN_PIN, MOT3_FAULT_PIN)
                                    };
*/
void rampMotor(int which, int16_t start, int16_t target, int ramp_total_time) {
  Serial.print("Ramping Motor - ");
  Serial.print(which);
  if (which > 2) {
    return;
  };

  int difference = target - start;
  int step_delay = abs(difference / ramp_total_time);

  Serial.print(" dif: ");
  Serial.print(difference);
  Serial.print(" stepd: ");
  Serial.print(step_delay);
  motors.enableDrivers(which);

  if (difference > 0) {
    for (int16_t i = start; i <= target; i++) {
      if (which == 0) {
        motors.setM1Speed(i);
      }
      else if (which == 1) {
        motors.setM2Speed(i);
      }
      else if (which == 2) {
        motors.setM3Speed(i);
      }
      delay(step_delay);
      // Serial.println(i);
    }
  } else {
    for (int16_t i = start; i > target; i--) {
      if (which == 0) {
        motors.setM1Speed(i);
      }
      else if (which == 1) {
        motors.setM2Speed(i);
      }
      else if (which == 2) {
        motors.setM3Speed(i);
      }
      delay(step_delay);
      // Serial.println(i);
    }
  }
  if (target == 0) {
    motors.disableDrivers(which);
  }
  Serial.println("Disabled Drivers");
}

#endif

#if DATALOG_ACTIVE == true
#include <SD.h>
#include <SerialFlash.h>
#include <EEPROM.h>
#endif // DATALOG_ACTIVE

#if ARTEFACT_TYPE == EXPLORATOR && BODY_TYPE == BELL_BODY
/* Mechatronic Creatures
  "Bowl Bot" Genus
  using the Adafruit Huzzah ESP8266 Microcontroller
*/
/////////////////////////////// Playback Engine  /////////////////////////////////
// the playback engine handles the playback of melodies and rhythms through motors
// and solenoids, as of right now, the only bot which makes use of this is the
// Explorator
Rhythm rhythm[10] = {
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm()
};

RhythmBank rhythm_bank = RhythmBank();
PlaybackEngine playback_engine = PlaybackEngine();

// actuator pin, dampener pin, frequency, on_time
BellMechanism bells[3] = {
  BellMechanism(s_pins[0], s_pins[1], 20, 100.0,  40),
  BellMechanism(s_pins[2], s_pins[3], 20, 500.0,  40),
  BellMechanism(s_pins[4], s_pins[5], 20, 1000.0, 40)
};

#elif ARTEFACT_TYPE == EXPLORATOR && BODY_TYPE == WOODPECKER_BODY
/////////////////////////////// Playback Engine  /////////////////////////////////
// the playback engine handles the playback of melodies and rhythms through motors
// and solenoids, as of right now, the only bot which makes use of this is the
// Explorator
Rhythm rhythm[10] = {
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm(),
  Rhythm()
};
RhythmBank rhythm_bank = RhythmBank();
PlaybackEngine playback_engine = PlaybackEngine();
// actuator pin, reference to motor, motor number, minimum_on_time for the solenoid
// maximum on time for the solenoid, min_time between actuations for the solenoid
WoodpeckerMechanism pecker[1] = {WoodpeckerMechanism(s_pins[0], &motors, 0, S1_MIN, S1_MAX, S1_BETWEEN)};

#endif // FIRMWARE_MODE and 

///////////////////////////////  LuxManager //////////////////////////////////////
// All artefacts will have a lux_manager
LuxManager lux_manager = LuxManager(lux_min_reading_delay, lux_max_reading_delay, LUX_MAPPING_SCHEMA);

/////////////////////////////// NeoPixelManager //////////////////////////////////
// for the explorators, there are three NeoPixels Strips
#if ARTEFACT_TYPE == SPECULATOR
WS2812Serial leds[1] = WS2812Serial(LED1_COUNT, displayMemory[0], drawingMemory[0], LED1_PIN, WS2812_GRB);
NeoGroup neos[1]     = NeoGroup(&leds[0], 0, LED1_COUNT - 1, "All Neos", MIN_FLASH_TIME, MAX_FLASH_TIME);

#elif ARTEFACT_TYPE == EXPLORATOR
// TODO - dynamically create these objects based on info in the configuration file
WS2812Serial leds[num_active_led_channels] = {
#if LED1_ACTIVE == true
  WS2812Serial(LED1_COUNT, displayMemory[0], drawingMemory[0], LED1_PIN, WS2812_GRB)
#if LED2_ACTIVE + LED3_ACTIVE > 0
  ,
#endif
#endif
#if  LED2_ACTIVE == true
  WS2812Serial(LED2_COUNT, displayMemory[1], drawingMemory[1], LED2_PIN, WS2812_GRB)
#if LED3_ACTIVE == true
  ,
#endif
#endif
#if LED3_ACTIVE == true
  WS2812Serial(LED3_COUNT, displayMemory[2], drawingMemory[2], LED3_PIN, WS2812_GRB)
#endif
};

// TODO - likewise for these
NeoGroup neos[num_active_led_channels] = {
#if LED1_ACTIVE == true
  NeoGroup(&leds[0], 0, LED1_COUNT, LED1_NAME)
#if LED2_ACTIVE + LED3_ACTIVE > 0
  ,
#endif
#endif
#if  LED2_ACTIVE == true
  NeoGroup(&leds[1], 0, LED2_COUNT, LED2_NAME)
#if LED3_ACTIVE == true
  ,
#endif
#endif
#if LED3_ACTIVE == true
  NeoGroup(&leds[2], 0, LED3_COUNT, LED3_NAME)
#endif
};

#endif // ARTEFACT_TYPE == xxxx

/////////////////////////////// FFTManager ///////////////////////////////////////
// all artefacts will have an input FFTManager
FFTManager1024 fft_manager = FFTManager1024(FFT_LOWEST_BIN, FFT_HIGHEST_BIN, "Input FFT");

/////////////////////////////// FeatureCollector /////////////////////////////////
// all artefacts will have a single FeatureCollector . . .
FeatureCollector feature_collector = FeatureCollector("All");

/////////////////////////////// WeatherManager   /////////////////////////////////
#if WEATHER_MANAGER_PRESENT
WeatherManager weather_manager = WeatherManager(HUMID_EXTREME_THRESH, TEMP_EXTREME_THRESH, TEMP_HISTERESIS, WEATHER_MANAGER_UPDATE_RATE);
#endif // WEATHER_MANAGER_PRESENT

//////////////////////////////////// User Controls ///////////////////////////////
UIManager uimanager = UIManager(UI_POLLING_RATE, P_UIMANAGER);

////////////////////////////// Audio System ///////////////////////////////////////
#if (ARTEFACT_TYPE == EXPLORATOR)
AudioInputI2S            i2s;              //xy=634,246
AudioAmplifier           amp1;      //xy=777.1429023742676,277.14284896850586
AudioFilterBiquad        biquad1;
AudioAnalyzePeak         peak1;             //xy=1139.4286575317383,258.42859840393066
AudioAnalyzeFFT1024      input_fft;           //xy=1145.4286575317383,290.42859840393066
AudioAnalyzeClipCounter  clip_counter;
AudioOutputUSB           usb_output;       //xy=1147.4286575317383,194.42859840393066
AudioConnection          patchCord1(i2s, 0, biquad1, 0);
AudioConnection          patchCord2(biquad1, amp1);
AudioConnection          patchCord3(amp1, clip_counter);
AudioConnection          patchCord6(amp1, peak1);
AudioConnection          patchCord7(amp1, input_fft);
AudioConnection          patchCord10(amp1, 0, usb_output, 0);
AudioConnection          patchCord11(amp1, 0, usb_output, 1);

#elif (ARTEFACT_TYPE == SPECULATOR) && (FIRMWARE_MODE == PITCH_MODE)
////////////////////////// Audio Objects //////////////////////////////////////////
AudioInputI2S            i2s1;           //xy=55,291.8571424484253
AudioFilterBiquad        biquad2;        //xy=217.00389099121094,302.0039281845093
AudioFilterBiquad        biquad1;        //xy=219.00390625,270.00391578674316
AudioAmplifier           amp2;           //xy=378.79129791259766,302.57704162597656
AudioAmplifier           amp1;           //xy=380.2198715209961,264.0055875778198
AudioAnalyzePeak         peak2;          //xy=517.0039100646973,316.003924369812
AudioAnalyzePeak         peak1;          //xy=521.00390625,221.0039176940918
AudioAnalyzeFFT1024      input_fft;      //xy=521.3627586364746,251.71987438201904
AudioAnalyzeRMS          rms1;            //xy=650.0000076293945,151.00000190734863
AudioAnalyzeRMS          rms2;            //xy=650.0000076293945,151.00000190734863

AudioConnection          patchCord10(amp1, rms1);
AudioConnection          patchCord11(amp2, rms2);

#if AUDIO_USB > 0
AudioOutputUSB           usb1;           //xy=519.142822265625,284.71433544158936
AudioConnection          patchCord5(amp2, 0, usb1, 1);
AudioConnection          patchCord7(amp1, 0, usb1, 0);
#endif // DEBUG_USB_AUDIO

AudioConnection          patchCord1(i2s1, 0, biquad1, 0);
AudioConnection          patchCord2(i2s1, 1, biquad2, 0);
AudioConnection          patchCord3(biquad2, amp2);
AudioConnection          patchCord4(biquad1, amp1);
AudioConnection          patchCord6(amp2, peak2);
AudioConnection          patchCord8(amp1, input_fft);
AudioConnection          patchCord9(amp1, peak1);
#endif // ARTEFACT_TYPE and BODY_TYPE and FIRMWARE_MODE

//////////////////////////////////////////////////////////////////////////////////////
// || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || ||
// || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || ||
// || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || || ||


void explainSerialCommands(bool p_user_controls) {
  dprintln(p_user_controls, "The Following Serial Commands are Supported: ");
  dprintln(p_user_controls, "Print Commands, denoted by a p prefix: ");
  dprintln(p_user_controls, "brightness_scaler (bs)");
  dprintln(p_user_controls, "datalogs (dl)");
  dprintMinorDivide(p_user_controls);
}

void listenForSerialCommands(bool p_user_controls) {
  if (Serial.available() > 0) {
    int input = Serial.read();
    dprint(p_user_controls, "incbyte : ");
    dprintln(p_user_controls, input);
    //////////////// PRINT COMMANDS ////////////////
    if (input == 'p') {
      input = Serial.read();
      // datalogs
      if (input == 'd') {
        input = Serial.read();
        if (input == 'l') {
          // this is the command to print the datalog
          // datalog_manager.printAllLogs();
        }
      }
      // print brightness scaler
      if (input == 'b') {
        input = Serial.read();
        if (input == 's') {
          dprint(p_user_controls, "Brightness Scalers: ");
          dprintln(p_user_controls, lux_manager.getBrightnessScaler());
          // dprint(p_user_controls, "\t");
          // dprintln(p_user_controls, brightness_scalers[1]);
        }
      }
      /////////////// Changing Values ////////////////////////
      if (input == 's') {
        input = Serial.read();
        if (input == 'g') {
          dprintln(p_user_controls, "what would you like to change the gain to? Please enter in a gain in the format of 1.00.");
          input = Serial.read() - 48;
          dprint(p_user_controls, "A gain of ");
          dprint(p_user_controls, input);
          dprintln(p_user_controls, " has been selected");
        }
      }
    }
  }
}

void updateAudioAnalysis() {
  // update the feature collectors
#if P_FUNCTION_TIMES == true
  function_times = 0;
#endif // P_FUNCTION_TIMES
  if (fft_manager.update()) {
#if P_FUNCTION_TIMES == true
    Serial.print("fft_manager update took ");
    Serial.print(function_times); Serial.println(" micro seconds to update");
#endif // P_FUNCTION_TIMES
  }

#if P_FUNCTION_TIMES == true
  function_times = 0;
#endif // P_FUNCTION_TIMES

  if (feature_collector.update()) {
#if P_FUNCTION_TIMES == true
    Serial.print("feature_collector update took ");
    Serial.print(function_times); Serial.println(" micro seconds to update");
#endif // P_FUNCTION_TIMES
  }
#if P_AUDIO_MEMORY_MAX > 0
  // so the printing is not crazy
  if (last_jumper_read > 99500) {
    Serial.print("audio memory max: ");
    Serial.println(AudioMemoryUsageMax());
  }
#endif
}

void printArtefactInfo() {
  //////////////// ARTEFACT TYPE /////////////////
  Serial.print("Artefact type:\t");
  if (ARTEFACT_TYPE == SPECULATOR) {
    Serial.println("SPECULATOR");
  } else if (ARTEFACT_TYPE == EXPLORATOR) {
    Serial.println("EXPLORATOR");
    Serial.print("Body type  :\t");
    Serial.println(BODY_TYPE);
  } else if (ARTEFACT_TYPE == LEGATUS) {
    Serial.println("LEGATUS");
  } else {
    Serial.println("UNKNOWN!!!!!");
    delay(15000);
  }

  //////////////// Hardware/Software Version /////////////////
  Serial.print("Firmware is version : ");
  Serial.print(SV_MAJOR);
  Serial.print(".");
  Serial.print(SV_MINOR);
  Serial.print(".");
  Serial.println(SV_REVISION);
  Serial.print("Firmware was compiled for a PCB with a hardware revsion of : ");
  Serial.print(HV_MAJOR);
  Serial.print(".");
  Serial.println(HV_MINOR);
  printMinorDivide();

  //////////////// Enclosure Type ///////////////////////////
#if (ARTEFACT_TYPE == SPECULATOR)
  Serial.print("Enclosure type is : ");
  if (ENCLOSURE_TYPE == GROUND_ENCLOSURE) {
    Serial.println("Ground");
  } else if (ENCLOSURE_TYPE == ORB_ENCLOSURE_WITHOUT_HOLE) {
    Serial.println("Orb without hole");
  } else if (ENCLOSURE_TYPE == ORB_ENCLOSURE_WITH_HOLE) {
    Serial.println("Orb with hole");
  } else if (ENCLOSURE_TYPE == NO_ENCLOSURE) {
    Serial.println("no enclosure");
  } else {
    Serial.println("ERROR, enclosure type is not recognized");
    colorWipeAll(255, 0, 0);
    delay(60000);
  }
  printMinorDivide();
#endif // SPECULATOR ARTEFACT TYPE
}

#if ARTEFACT_TYPE == SPECULATOR
void speculatorSetup() {
  // setup up some value tracker stuff
  brightness_tracker.setMinMaxUpdateFactor(BGT_MIN_UPDATE_FACTOR, BGT_MAX_UPDATE_FACTOR);
  hue_tracker.setMinMaxUpdateFactor(HUE_MIN_UPDATE_FACTOR, HUE_MAX_UPDATE_FACTOR);
  saturation_tracker.setMinMaxUpdateFactor(SAT_MIN_UPDATE_FACTOR, SAT_MAX_UPDATE_FACTOR);

  //////////////// User Controls /////////////////////////////
  explainSerialCommands(true);
  // TODO - need to replace with the UIManager
  // setupUserControls();
  // testJumpers(true);
  // readUserControls(true); // to print out the actual final results

  ///////////////////////// Audio //////////////////////////
  setupAudio();

  ///////////////////////// auto_gain //////////////////////////
  // target value, tolerance, min_thresh, max_thresh
  /*
    auto_gain.trackAvgRMS(0.1, 0.2, 0.05, 0.2);
    Serial.println("Setting auto_gain to track RMS Avg");
    auto_gain.setUpdateRate(AUTOGAIN_FREQUENCY);
    Serial.print("Setting auto_gain's update rate to: ");
    Serial.println(AUTOGAIN_FREQUENCY);
    auto_gain.setInitialUpdateRate(AUTOGAIN_START_DELAY);
    Serial.print("Setting auto_gain's start delay to : ");
    Serial.println(AUTOGAIN_START_DELAY);
  */

  ///////////////////////// DL Manager //////////////////////////
  // TODO
  // setupDLManagerCicada();
  // printMinorDivide();
}

void speculatorLoop() {

}

#endif //  ARTEFACT_TYPE == SPECULATOR

void exploratorLoop() {
  if (BODY_TYPE == BELL_BODY) {
    exploratorBellBotLoop();
  } else if (BODY_TYPE == WOODPECKER_BODY) {
    exploratorWoodpeckerLoop();
  }
}

void exploratorWoodpeckerLoop() {
  ///////////////// Actuator Outputs //////////////////
  playback_engine.update(); // will also update all linked mechanisms
  
  ///////////////// Passive Visual Feedback ///////////
  updateFeedbackLEDs();
  
  // the warmer the temperature the more it will actuate? (10 second decrease at 40 degrees and no decrease when at 0 degrees
  // the higher the humidity the less it will actuate? (100 second increase when 100% humidity , 0 second when at 0 %)
  // the brighter it is the more it will actuate (take 5000 lux and subtract the current reading)
  // activity level adds a base of up to five minutes
  ACTUATION_DELAY = (ACTIVITY_LEVEL * ACTIVITY_LEVEL * 5 * 60000) + (weather_manager.getTemperature() * -250) + (weather_manager.getHumidity() * 1000) + (5000 - lux_manager.getGlobalLux());
  // uint16_t t = random(45, 150);
  // TODO
  ACTUATION_DELAY = ACTUATION_DELAY * 0.5;
  if (last_playback_tmr > ACTUATION_DELAY) {
    Serial.print("ACTUATION_DELAY is : ");
    Serial.println(ACTUATION_DELAY);
    if (playback_engine.isActive() == false) {
      playback_engine.playRhythm(rhythm_bank.getRandomRhythm());
      last_playback_tmr = 0;
    } else {
      Serial.println("Skipping rhythm as a rhythm is already playing");
    }
  }
}

void exploratorBellBotLoop() {
#if BODY__TYPE == BELL_BODY
  ///////////////// Actuator Outputs //////////////////
  updateSolenoids(); // turns off all solenoids which have
  playback_engine.update();
  ACTUATION_DELAY = (30000) + ((weather_manager.getTemperature() + weather_manager.getHumidity()) * lux_manager.getGlobalLux());
  if (last_playback_tmr > ACTUATION_DELAY) {
    Serial.print("actuation_delay : ");
    Serial.println(ACTUATION_DELAY);
    Serial.println("playing rhythm through playback_engine");
    playback_engine.playRhythm(rhythm_bank.getRandomRhythm());
    last_playback_tmr = 0;
  }
  ///////////////// Passive Visual Feedback ///////////
  updateFeedbackLEDs();
#endif
}

//////////////////////////////////////////////////////////////////////////
////////////////// setup / main loops ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void exploratorSetup() {

  printMajorDivide("starting exploratorSetup() loop");

#if ARTEFACT_TYPE == EXPLORATOR && BODY_TYPE == WOODPECKER_BODY
  pecker[0].setMotorSpeeds(70, -30);
#endif

  /////////////// User Controls ////////////////////////////////////////////
  uimanager.addBut(BUT1_PIN, BUT1_REVERSE, BUT1_PULLUP, &but_test[0], BUT1_NAME);
  uimanager.addBut(BUT2_PIN, BUT2_REVERSE, BUT2_PULLUP, &but_test[1], BUT2_NAME);
  uimanager.addBut(BUT3_PIN, BUT3_REVERSE, BUT3_PULLUP, &but_test[2], BUT3_NAME);
  uimanager.addBut(BUT4_PIN, BUT4_REVERSE, BUT4_PULLUP, &but_test[3], BUT4_NAME);

  uimanager.addPot(POT1_PIN, POT1_REVERSE, POT1_PLAY, &ACTIVITY_LEVEL, POT1_NAME);
  uimanager.addPot(POT2_PIN, POT2_REVERSE, POT2_PLAY, &STRIKE_LENGTH,  POT2_NAME);

  uimanager.setup();
  uimanager.printAll();

  ////////////////////// Audio
  printMinorDivide();
  Serial.println("Starting the Audio system");
  AudioMemory(AUDIO_MEMORY);
  // TODO make this proper
  uint32_t lpf = 14000;
  uint32_t hpf = 200;
  double q = 0.8;
  amp1.gain(60.0);

  biquad1.setLowpass(0, lpf, q);
  biquad1.setLowpass(1, lpf, q);
  biquad1.setHighpass(2, hpf, q);
  biquad1.setHighpass(3, hpf, q);

  configurePlaybackEngine();

  fft_manager.linkFFT(&input_fft);
  fft_manager.setCalculateCent(true);
  fft_manager.setCalculateFlux(true);

  feature_collector.linkPeak(&peak1, P_PEAK_VALS);
  Serial.println("Finished starting the LED strips");
  printMinorDivide();

  printMinorDivide();
  delay(3000);
  Serial.println("Finished setup Loop");
  colorWipeAll(0, 120, 30, 0.25);
  printMinorDivide();
}

#if (ARTEFACT_TYPE == EXPLORATOR) && TEST_SOLENOIDS == true
void testSolenoids(unsigned int len) {
  elapsedMillis t = 0;
  Serial.print("Testing Solenoids - ");
  while (t < len) {
    for (int i = 0; i < NUM_SOLENOIDS; i++) {
      if (sol_active[i] == true) {
        Serial.print(i);
        digitalWrite(s_pins[i], HIGH); // LOW is on for these transistors as a low opens the gate
        delay(30);
        Serial.print(" ");
        digitalWrite(s_pins[i], LOW);
      }
    }
    Serial.println();
  }
  Serial.println("Finished testing solenoids");
  Serial.println("--------------------------");
};
#endif // test_solenoids, artefact_type explorator

void setOutputs() {
  /////////////// Solenoid Outputs /////////////////////////////////////////
  for (int i = 0; i < NUM_SOLENOIDS; i++) {
    pinMode(s_pins[i], OUTPUT);
    digitalWrite(s_pins[i], LOW);// turns them off
  }

#if (ARTEFACT_TYPE == EXPLORATOR) && TEST_SOLENOIDS == true
  Serial.println("TEST_SOLENOIDS is set to true, system will just test solenoids over and over again forever");
  testSolenoids(1000);// let the system settle
  Serial.println("Finished setting solenoid pins to outputs");
#endif

#if (NUM_MOTORS > 0)
  Serial.println("Starting Motor Set-up");
  motors.flipM1(M1_POLARITY);
  motors.flipM2(M2_POLARITY);
  motors.flipM3(M3_POLARITY);
  motors.enableDrivers();
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.setM3Speed(0);
  delay(500);
  Serial.println("Finished setting up motors");
  Serial.println("-------------------------------------------------------------");
  delay(2500);// let the system settle
#endif
}

void setupLuxManager() {
  printMinorDivide();
  Serial.println("turning off LEDs for Lux Calibration");
  for (int i = 0; i < num_active_led_channels; i++) {
    neos[i].colorWipe(0, 0, 0, 0.0);
  }
  Serial.println("LEDS off");
  delay(100);

  lux_manager.setLuxThresholds(LOW_LUX_THRESHOLD, MID_LUX_THRESHOLD, HIGH_LUX_THRESHOLD, EXTREME_LUX_THRESHOLD);
  lux_manager.setPrintBrightnessScaler(P_BS);
  Serial.print("lux_manager print_brightness_scaler is set to   : \t");
  Serial.println(P_BS);
  lux_manager.setPrintLuxReadings(P_LUX_READINGS);
  Serial.print("lux_manager print_lux_readings is set to        : \t");
  Serial.println(P_LUX_READINGS);
  lux_manager.setPrintGeneralDebug(P_LUX_MANAGER_DEBUG);
  Serial.print("lux_manager print_general_debug is set to       : \t");
  Serial.println(P_LUX_MANAGER_DEBUG);

  delay(500);
#if (ARTEFACT_TYPE == SPECULATOR) && (HV_MAJOR > 2)
  lux_manager.add6030Sensors(2, 25);
  lux_manager.linkNeoGroup(&neos[0]);
#elif (ARTEFACT_TYPE == SPECULATOR) && (HV_MAJOR == 3)
  lux_manager.addSensorTcaIdx("Front", 0);
  lux_manager.addSensorTcaIdx("Rear", 1);
  lux_manager.startTCA7700Sensors(VEML7700_GAIN_1, VEML7700_IT_25MS); // todo add this to config_adv? todo
#elif (ARTEFACT_TYPE == EXPLORATOR)
  Serial.println("Starting LuxManager");
  lux_manager.add7700Sensor((String)"Eye-Stock");
  lux_manager.start7700Sensor(VEML7700_GAIN_1, VEML7700_IT_25MS); // todo add this to config_adv? todo
  Serial.println("Finished starting LuxManager");
#endif // HV_MAJOR
  delay(200);

  if ((lux_manager.getSensorActive(0) || lux_manager.getSensorActive(1)) > 0) {
    Serial.println("Linked the NeoGroups to the LuxManager");
    for (int i = 0; i < num_active_led_channels; i++) {
      lux_manager.linkNeoGroup(&neos[i]);
    }
    delay(200);
  } else {
    Serial.println("ERROR, unable to link NeoGroups to LuxManager");
  }
  lux_manager.setBrightnessScalerMinMax(LUX_BS_MIN, LUX_BS_MAX);
  lux_manager.calibrate(2000, true);
  if (P_LUMIN) {
    lux_manager.print();
  };
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
#if NUM_SOLENOIDS > 0 || NUM_MOTORS > 0
  setOutputs();
#endif
  ///////////////// Serial ///////////////////////////////////
  printDivide();
  Serial.println("Entering the Setup Loop");
  printDivide();
  Serial.println("Serial begun");
  printMinorDivide();
  printArtefactInfo();

  /////////////////// NeoPixels ////////////////////////////////
  printMinorDivide();
  Serial.println("Starting the LED strips");
  for (int i = 0; i < num_active_led_channels; i++) {
    leds[i].begin();
    Serial.print("neogroup ");
    Serial.println(i);
    neos[i].begin();
    neos[i].colorWipe(12, 12, 12, 1.0);

    printMinorDivide();
    Serial.println("LEDS have been initalised");
    Serial.print("There are ");
    Serial.print(LED1_COUNT);
    Serial.println(" LEDs in the first group");
    Serial.print("There are ");
    Serial.print(LED2_COUNT);
    Serial.println(" LEDs in the second group");
    Serial.print("There are ");
    Serial.print(LED3_COUNT);
    Serial.println(" LEDs in the third group");

    Serial.print("NeoGroup p_lux and p_extreme_lux are both set to : \t"); Serial.println(P_LUMIN);
    neos[i].setPrintLux(P_LUMIN);
    Serial.print("NeoGroup p_lux is set to                         : \t"); Serial.println(P_LUMIN);
    neos[i].setPrintExtremeLux(P_LUMIN);
    Serial.print("NeoGroup p_brightness_scaler is set to           : \t"); Serial.println(P_BS);
    neos[i].setPrintBrightnessScaler(P_BS);
    Serial.print("NeoGroup p_leds_on is set to                     : \t"); Serial.println(P_LEDS_ON);
    neos[i].setPrintLedsOn(P_LEDS_ON);
    Serial.print("NeoGroup p_on_ratio is set to                    : \t"); Serial.println(P_LED_ON_RATIO);
    neos[i].setPrintOnRatio(P_LED_ON_RATIO);
    Serial.print("NeoGroup p_color_wipe is set to                  : \t"); Serial.println(P_COLOR_WIPE);
    neos[i].setPrintColorWipe(P_COLOR_WIPE);
    Serial.print("NeoGroup p_onset is set to                       : \t"); Serial.println(P_ONSET);
    neos[i].setPrintOnRatio(P_ONSET);
    Serial.print("NeoGroup p_onset is set to                       : \t"); Serial.println(P_ONSET);
    neos[i].setPrintPackColors(P_PACK_COLORS);
    printMinorDivide();
    Serial.print("NeoGroup SATURATED_COLORS                        : \t"); Serial.println(P_ONSET);
    neos[i].setSaturatedColors(SATURATED_COLORS);

    neos[i].setOnsetColors(ONSET_RED, ONSET_GREEN, ONSET_BLUE);

    neos[i].setFlashBehaviour(FLASH_DOMINATES);
    Serial.print("NeoGroup flash_behaviour is set to                : \t");
    Serial.println(FLASH_DOMINATES);

    neos[i].changeMapping(LED_MAPPING_MODE);
    // neos[i].setSongFeedbackMode(ROUND);
    Serial.print("LED_MAPPING_MODE is set to                        : \t");
    Serial.println(LED_MAPPING_MODE);

#if (ARTEFACT_TYPE == SPECULATOR)
    neos[i].setSongColors(SONG_RED_HIGH, SONG_GREEN_HIGH, SONG_BLUE_HIGH);
    Serial.print("REVERSE_HUE is set to                             : \t");
    Serial.println(REVERSE_HUE);
    Serial.print("REVERSE_SATURATION is set to                      : \t");
    Serial.println(REVERSE_SATURATION);
    Serial.print("REVERSE_BRIGHTNESS is set to                      : \t");
    Serial.println(REVERSE_BRIGHTNESS);
#endif
  }
  Serial.println("Finished starting the LED strips");
  printMinorDivide();

  ///////////////////////// Weather Manager /////////////////////
  // nothing is needed =P
#if WEATHER_MANAGER_PRESENT == true
  printMinorDivide();
  Serial.println("initalising the weather manager");
  weather_manager.init();
  weather_manager.setPrintReadings(P_WEATHER_MANAGER_READINGS);
  Serial.println("finished initalising the weather manager");
  Serial.print("WeatherManager HUMID_EdXTREME_THRESH  : \t");
  Serial.println(HUMID_EXTREME_THRESH);
  Serial.print("WeatherManager TEMP_EXTREME_THRESH   : \t");
  Serial.println(TEMP_EXTREME_THRESH);
  Serial.print("WeatherManager TEMP_HISTERESIS      : \t");
  Serial.println(TEMP_HISTERESIS);
  Serial.print("WEATHER_MANAGER_UPDATE_RATE         : \t");
  Serial.println(WEATHER_MANAGER_UPDATE_RATE);
  weather_manager.update();
  weather_manager.print();
  printMinorDivide();
#endif

  //////////////////////////// Lux Sensors //////////////////////////////
  setupLuxManager();

  ///////////////////////// Artefact Specific Logic ////////////////////////////
#if ARTEFACT_TYPE == SPECULATOR
  speculatorSetup();
#elif ARTEFACT_TYPE == EXPLORATOR
  exploratorSetup();
#elif ARTEFACT_TYPE == LEGATUS
  legatusSetup();
#endif

  /////////////////////////////// Main Loop Delay ////////////////////////////////
  for (int it = 0; it < LED1_COUNT; it++) {
    // we keep readng the jumpers so if we change the jumpers and dont
    // want the boot delay that can happen
    // TODO - replace with the user control manager
    // readJumpers();
    // uimanager.update();
    uint32_t segment = (uint32_t)((double)BOOT_DELAY / (double)LED1_COUNT);
    if (segment > 10) {
      Serial.print("boot delay segment length in seconds is : ");
      Serial.println(segment / 1000, 2);
    }
    neos[0].setPixel(it, 10, 32, 20, 1.0);
    if (digitalRead(BUT1_PIN)) {
      delay(segment);
    }
  }
  neos[0].colorWipe(0, 5, 0, 1.0);
  printMajorDivide("Now starting main() loop");
}

void extremeHumidityShutdown() {
  colorWipeAll(0, 0, 0);
  while (true) {
    Serial.println("HUMIDTY SHUTDOWN INITALISED!!!!!!");
    delay(10000);
  }
  // TODO
}

void extremeTemperatureShutdown() {
  unsigned int times = 0;
  Serial.println("TEMPERATURE SHUTDOWN INITALISED!!!!!!");
  colorWipeAll(255, 0, 0);
  delay(1000);
  colorWipeAll(0, 0, 0);
  while (weather_manager.getTempShutdown() == true) {
    weather_manager.update();
    if (times % 20 == 10) {
      colorWipeAll(255, 0, 0);
    } else if (times % 20 == 11) {
      colorWipeAll(0, 0, 0);
    }
    times++;
    delay(100);
  }
  Serial.println("TEMPERATURE SHUTDOWN LIFTED - resuming normal operation");
}

void loop() {
  //////////////// Testing ///////////////////////////////////
  /*
  if (playback_engine.isActive() == true){
    clip_counter.deactivate();
  } else {
    clip_counter.activate();
    Serial.println(clip_counter.getNumClips());
  }
  */
  
  ///////////////// Ambient Lighting /////////////////////////
  if (lux_manager.getActive() == true) {
    if (lux_manager.update()) {
      // print the updated values
      if (P_LUX_MANAGER_READINGS) {
        lux_manager.print();
      };
    }
  }
  ///////////////// User Controls ////////////////////////////
  uimanager.update();

  ///////////////// WeatherManager ///////////////////////////
#if WEATHER_MANAGER_PRESENT == true
  if (weather_manager.update() && P_WEATHER_MANAGER_READINGS) {
    weather_manager.print();
  }
  if (weather_manager.getHumidityShutdown() == true) {
    extremeHumidityShutdown();
  } else if (weather_manager.getTempShutdown() == true) {
    extremeTemperatureShutdown();
  } else {
#endif // WEATHER_MANAGER_PRESENT
    // if (lux_manager.getExtremeLux() == true) {
    //   Serial.println("WARNING ------------ updateMode() returning due extreme lux conditions, not updating onset or song...");
    // } else {
    //////////////// Audio Analysis ///////////////////////////
    updateAudioAnalysis();
    // Serial.println(millis()/1000);
#if ARTEFACT_TYPE == SPECULATOR
    speculatorLoop();
#elif ARTEFACT_TYPE == EXPLORATOR
    exploratorLoop();
#elif ARTEFACT_TYPE == LEGATUS
    legatusLoop();
#endif
    // updateAutogain();
    // TODO
    // updateDatalog();
    // readUserControls(P_USER_CONTROLS);
    // }

#if WEATHER_MANAGER_PRESENT
  }
#endif // WEATHER_MANAGER_PRESENT
}

void colorWipeAll(uint8_t red, uint8_t green, uint8_t blue, float bright) {
  for (int i = 0; i < num_active_led_channels; i++) {
    neos[i].colorWipe(red, green, blue, bright);
  }
}

void colorWipeAll(uint8_t red, uint8_t green, uint8_t blue) {
  colorWipeAll(red, green, blue, 0.5);
}

void printAudioUsage() {
  // print the audio usage every second or so
  if (last_audio_usage_print > AUDIO_USAGE_POLL_RATE) {
    Serial.print("memory usage: ");
    Serial.print(AudioMemoryUsageMax());
    Serial.print(" out of ");
    Serial.println(AUDIO_MEMORY);
    last_audio_usage_print = 0;
    AudioMemoryUsageMaxReset();
  }
}

void configurePlaybackEngine() {
  // freq, length, onset delay (since last note), velocity
#if BODY_TYPE == BELL_BODY
  rhythm[0].addPitchedNote(500.0,  40, 0,    0.6);
  rhythm[0].addPitchedNote(500.0,  40, 100, 0.8);
  rhythm[0].addPitchedNote(500.0,  30, 150, 1.0);
  rhythm[0].addPitchedNote(1000.0, 30, 400, 1.0);
  rhythm[0].addPitchedNote(1000.0, 20, 1000, 1.0);
  rhythm[0].addPitchedNote(1000.0, 50, 1500, 1.0);

  rhythm[0].addPitchedNote(100.0,  30, 100, 0.3);
  rhythm[0].addPitchedNote(500.0,  40, 600, 0.4);
  rhythm[0].addPitchedNote(1000.0, 30, 150, 0.5);
  rhythm[0].addPitchedNote(100.0,  20, 700, 0.4);
  rhythm[0].addPitchedNote(500.0,  40, 800, 0.3);
  rhythm[0].addPitchedNote(1000.0, 30, 1500, 0.2);

  rhythm[0].addPitchedNote(100.0,  40, 200, 1.0);
  rhythm[0].addPitchedNote(500.0,  30, 70,  1.0);
  rhythm[0].addPitchedNote(1000.0, 40, 90,  1.0);
  rhythm[0].addPitchedNote(100.0,  40, 60,  1.0);
  rhythm[0].addPitchedNote(500.0,  30, 150, 1.0);
  rhythm[0].addPitchedNote(1000.0, 30, 100, 1.0);

  rhythm[1].addPitchedNote(50.0,   50, 0,    1.05);
  rhythm[1].addPitchedNote(150.0,  50, 500,  1.01);
  rhythm[1].addPitchedNote(250.0,  60, 1000, 1.15);
  rhythm[1].addPitchedNote(550.0,  60, 1500, 1.20);
  rhythm[1].addPitchedNote(450.0,  50, 2000, 1.20);
  rhythm[1].addPitchedNote(1550.0, 40, 2500, 1.20);
  rhythm[1].addPitchedNote(1650.0, 30, 3000, 1.20);

  rhythm[2].addPitchedNote(50.0,   40, 0,    1.35);
  rhythm[2].addPitchedNote(150.0,  30, 400,  1.31);
  rhythm[2].addPitchedNote(250.0,  30, 800,  1.35);
  rhythm[2].addPitchedNote(1350.0, 40, 1200, 1.30);
  rhythm[2].addPitchedNote(450.0,  30, 2000, 1.30);
  rhythm[2].addPitchedNote(550.0,  50, 2400, 1.30);
  rhythm[2].addPitchedNote(1650.0, 20, 2600, 1.30);

  rhythm[3].addPitchedNote(150.0,  40, 0,    1.35);
  rhythm[3].addPitchedNote(150.0,  50, 400,  1.31);
  rhythm[3].addPitchedNote(250.0,  60, 600,  1.35);
  rhythm[3].addPitchedNote(1350.0, 40, 700,  1.30);
  rhythm[3].addPitchedNote(450.0,  40, 800,  1.30);
  rhythm[3].addPitchedNote(550.0,  50, 500,  1.30);
  rhythm[3].addPitchedNote(1650.0, 40, 1000, 1.30);

  rhythm_bank.addRhythm(& rhythm[0]);
  rhythm_bank.addRhythm(& rhythm[1]);
  rhythm_bank.addRhythm(& rhythm[2]);
  rhythm_bank.addRhythm(& rhythm[3]);
  * /

  playback_engine.linkBellMechanism(& bells[0]);
  playback_engine.linkBellMechanism(& bells[1]);
  playback_engine.linkBellMechanism(& bells[2]);

  playback_engine.linkNeoGroup(& neos[0]);
  playback_engine.linkNeoGroup(& neos[1]);
  playback_engine.linkNeoGroup(& neos[2]);

#elif BODY_TYPE == WOODPECKER_BODY
  for (int i = 0; i < 10; i++) {
    uint32_t quarter = random(120, 750);
    buildPeckRhythm(i, quarter);
    if (random(0, 100)) {
      rhythm[i].addUnpitchedNote(quarter*4, 0.75);
      buildPeckRhythm(i, quarter);
    }
    rhythm[i].addMotorMove(1, 70, 100);
    rhythm[i].print();
    rhythm_bank.addRhythm(& rhythm[i]);
  }
  playback_engine.linkMechanism(& pecker[0]);
  playback_engine.linkNeoGroup(& neos[1]);
#endif // BODY_TYPE == BELL_BODY
}

void buildPeckRhythm(int idx, uint32_t quarter) {
  uint32_t t = 0;
  uint32_t eigth = quarter / 2;
  uint32_t triplet = quarter / 3;

  // 30% chance to start with a hard strike
  if (random(0, 100) < 30) {
    rhythm[idx].addUnpitchedNote(0, 0.5);
    t = quarter;
  }

  // then 85% chance for a triplet of 3-4
  if (random(0, 100) < 85) {
    for (int i = 0; i < (int)random(3, 4); i++) {
      rhythm[idx].addUnpitchedNote(t, 0.25);
      t = triplet;
    }
  }
  
  // then 55% chance for a triplet of 3-4
  if (random(0, 100) < 55) {
    for (int i = 0; i < (int)random(3, 4); i++) {
      rhythm[idx].addUnpitchedNote(t, 0.25);
      t = triplet;
    }
  }

  // chance for a pause
  if (random(0, 100) < 45) {
    t += triplet;
    if (random(0, 100) < 45) {
      t += triplet;
    }
  }

  // then 55% chance for a triplet of 3-4
  if (random(0, 100) < 55) {
    for (int i = 0; i < (int)random(3, 4); i++) {
      rhythm[idx].addUnpitchedNote(t, 0.25);
      t = triplet;
    }
  }

  // chance for a pause
  if (random(0, 100) < 45) {
    t += triplet;
    if (random(0, 100) < 45) {
      t += eigth;
    }
    if (random(0, 100) < 65) {
      t += eigth;
    }
  }

  // 30% chance to start with a hard strike
  if (random(0, 100) < 30) {
    rhythm[idx].addUnpitchedNote(t, 0.5);
    t = quarter;
  }
}

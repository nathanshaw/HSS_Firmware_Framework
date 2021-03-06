#ifndef __CONFIGURATION_ADV_H__
#define __CONFIGURATION_ADV_H__
#include "PrintUtils.h"
#include "Macros.h"

/////////////////////// Enclosure Type //////////////////////////////
// Currently there are four the Speculator enclosures: 
// GROUND_ENCLOSURE, ORB_ENCLOSURE_WITH_HOLE
// ORB_ENCLOSURE_WITHOUT_HOLE, and NO_ENCLOSURE
// if it is unknown what enclosure will be paired with the PCB
// it is recommended to map ENCLOSURE_TYPE to a physical switch
uint8_t ENCLOSURE_TYPE =          ORB_ENCLOSURE_WITHOUT_HOLE;

// different enclosures result in varying attenuation levels
// ENC_GAIN_ADJUST is automatically calculated based on
// the ARTEFACT_TYPE and ENCLOSURE_TYPE
#if ARTEFACT_TYPE == SPECULATOR
#if ENCLOSURE_TYPE == ORB_ENCLOSURE_WITH_HOLE
double ENC_GAIN_ADJUST =        1.0;
#elif ENCLOSURE_TYPE == ORB_ENCLOSURE_WITHOUT_HOLE
double ENC_GAIN_ADJUST =        2.0;
#elif ENCLOSURE_TYPE == GROUND_ENCLOSURE
double ENC_GAIN_ADJUST =        3.0;
#elif ENCLOSURE_TYPE == NO_ENCLOSURE
double ENC_GAIN_ADJUST =        0.75;
#endif // ENCLOSURE_TYPE
#endif // ARTEFACT_TYPE

//////////////////////////////////////////////////////////////////
/////////////////////// Enclosure Finish /////////////////////////
//////////////////////////////////////////////////////////////////
#define FINISH_NORMAL                  0
#define FINISH_FROSTED                 1

#define ENCLOSURE_FINISH               FINISH_NORMAL

#if ENCLOSURE_FINISH == FINISH_NORMAL
#define LUX_ENCLOSURE_SCALER           0.75
#elif ENCLOSURE_FINISH == FINISH_FROSTED
#define LUX_ENCLOSURE_SCALER           1.0
#endif 

//////////////////////////////////////////////////////////////////////
////////////////////// Lux Sensors ///////////////////////////////////
//////////////////////////////////////////////////////////////////////
#if HV_MAJOR > 1
#define NUM_LUX_SENSORS           2
#endif

#define FRONT_MICROPHONE_INSTALLED      true
#define REAR_MICROPHONE_INSTALLED       true

////////////// TCA Bus Expanders     /////
// I2C_MULTI should be 0 if no TCA I2C bus expander is present on the PCB
// I2C MULTI should be 1 if a TCA I2C bus expander is present
#if HV_MAJOR < 3 && ARTEFACT_TYPE == SPECULATOR
#define I2C_MULTI                 1
// the number of active channels on the TCA (can in theory support 8 sensors, etc.)
#define TCA_CHANNELS              2
#else
#define I2C_MULTI                 0
#define TCA_CHANNELS              0 
#endif

//////////// MICROCONTROLLER PIN OUTS ////
#define LED_PIN                   5

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// User Controls //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// in ms, how often will theUI controls update?
#define UI_POLLING_RATE 60
// how much play will the pots have before returning a value?
// good values should range between 0.002 and 0.005
#define POT_PLAY        0.004

// should the UIManager print when the user control elements are changed?
#define P_UIMANAGER     true
// for the bell bot basically
#if (ARTEFACT_TYPE == EXPLORATOR) &&  (HV_MAJOR == 0)
///////////////////// Buttons //////////////////////////////////////////////////
#define NUM_BUTTONS     0
#define BUT1_PIN        33
#define BUT2_PIN        A13
#define BUT3_PIN        A12
#define BUT4_PIN        17

// should the values received from the buttons be reversed?
#define BUT1_REVERSE    false
#define BUT2_REVERSE    false
#define BUT3_REVERSE    false
#define BUT4_REVERSE    false

#define BUT1_ACTIVE     false
#define BUT2_ACTIVE     false
#define BUT3_ACTIVE     false
#define BUT4_ACTIVE     false

#define BUT1_PULLUP     false
#define BUT2_PULLUP     false
#define BUT3_PULLUP     false
#define BUT4_PULLUP     false

// what is the name of the button? should reflect what it does
#define BUT1_NAME       "N/A"
#define BUT2_NAME       "N/A"
#define BUT3_NAME       "N/A"
#define BUT4_NAME       "N/A"

///////////////////// Pots /////////////////////////////////////////////////////
#define NUM_POTS        1
#define POT1_PIN        A1
#define POT2_PIN        A10

#define POT1_ACTIVE     true
#define POT2_ACTIVE     false

// should the values received from the pots be reversed?
#define POT1_REVERSE    true
#define POT2_REVERSE    true

// how much play will the pots have before returning a value?
// good values should range between 0.002 and 0.005
#define POT1_PLAY       0.004
#define POT2_PLAY       0.004

// what is the name of the button? should reflect what it does
#define POT1_NAME       "POT1"
#define POT2_NAME       "N/A"

////////////////////////////////////////////////////////////////////////////////
// now lets set things up for the woodpecker bot
////////////////////////////////////////////////////////////////////////////////
# elif (ARTEFACT_TYPE == EXPLORATOR) && (HV_MAJOR == 1)

///////////////////// Buttons //////////////////////////////////////////////////
#define NUM_BUTTONS     4
#define BUT1_PIN        33
#define BUT2_PIN        A13
#define BUT3_PIN        A12
#define BUT4_PIN        17

// should the values received from the buttons be reversed?
#define BUT1_REVERSE    false
#define BUT2_REVERSE    false
#define BUT3_REVERSE    false
#define BUT4_REVERSE    false

#define BUT1_ACTIVE     true
#define BUT2_ACTIVE     true
#define BUT3_ACTIVE     true
#define BUT4_ACTIVE     true

#define BUT1_PULLUP     false
#define BUT2_PULLUP     false
#define BUT3_PULLUP     false
#define BUT4_PULLUP     false

// what is the name of the button? should reflect what it does
#define BUT1_NAME       "BUT1"
#define BUT2_NAME       "BUT2"
#define BUT3_NAME       "BUT3"
#define BUT4_NAME       "BUT4"

///////////////////// Pots /////////////////////////////////////////////////////
#define NUM_POTS        2
#define POT1_PIN        A11
#define POT2_PIN        A10

// should the values received from the pots be reversed?
#define POT1_REVERSE    false
#define POT2_REVERSE    false

#define POT1_ACTIVE     true
#define POT2_ACTIVE     true

// how much play will the pots have before returning a value?
// good values should range between 0.002 and 0.005
#define POT1_PLAY       0.004
#define POT2_PLAY       0.004

// what is the name of the button? should reflect what it does
#define POT1_NAME       "ACTIVITY_LEVEL"
#define POT2_NAME       "STRIKE_LENGTH"

#elif (ARTEFACT_TYPE == SPECULATOR) && (HV_MAJOR == 2) && (HV_MINOR == 1)
// num buttons (how many buttons does the hardware support?)
#define NUM_BUTTONS       6

// pins        (what are the pins associated with the buttons?)
#define BUT1_PIN          12
#define BUT2_PIN          11
#define BUT3_PIN          14
#define BUT4_PIN          15
#define BUT5_PIN          16
#define BUT6_PIN          17

// reverse     (should the reading be reversed?)
#define BUT1_REVERSE    false
#define BUT2_REVERSE    false
#define BUT3_REVERSE    false
#define BUT4_REVERSE    false
#define BUT5_REVERSE    false
#define BUT6_REVERSE    false

// active      (is this button populated (available on the enclosure))
#define BUT1_ACTIVE     true
#define BUT2_ACTIVE     true
#define BUT3_ACTIVE     true
#define BUT4_ACTIVE     true
#define BUT5_ACTIVE     true
#define BUT6_ACTIVE     true

// pullup      (should the hardware pull-up be used)
#define BUT1_PULLUP     false
#define BUT2_PULLUP     false
#define BUT3_PULLUP     false
#define BUT4_PULLUP     false
#define BUT5_PULLUP     false
#define BUT6_PULLUP     false

// TODO name these properly
// name        (what is the name for the buttons)
#define BUT1_NAME       "BUT1"
#define BUT2_NAME       "BUT2"
#define BUT3_NAME       "BUT3"
#define BUT4_NAME       "BUT4"
#define BUT5_NAME       "BUT5"
#define BUT6_NAME       "BUT6"

// num pots    (how many pots does the hardware support?)
#define NUM_POTS        0
// pins        (what are the A pins associated with the pots?)
// reverse     (should the reading be reversed?)
// active      (is the pot populated (available on the enclosure)) 
// play        (what is the amount of play allowed? 0.002 - 0.005 is usally good
// name        (what is the name for the pot?)

#elif (ARTEFACT_TYPE == SPECULATOR) && (HV_MAJOR == 3) && (HV_MINOR == 0)
// num buttons (how many buttons does the hardware support?)
#define NUM_BUTTONS       10

// pins        (what are the pins associated with the buttons?)
#define BUT1_PIN          2
#define BUT2_PIN          3
#define BUT3_PIN          4
#define BUT4_PIN          6
#define BUT5_PIN          7
#define BUT6_PIN          8
#define BUT7_PIN          10
#define BUT8_PIN          11
#define BUT9_PIN          12
#define BUT10_PIN         14

// reverse     (should the reading be reversed?)
#define BUT1_REVERSE    false
#define BUT2_REVERSE    false
#define BUT3_REVERSE    false
#define BUT4_REVERSE    false
#define BUT5_REVERSE    false
#define BUT6_REVERSE    false
#define BUT7_REVERSE    false
#define BUT8_REVERSE    false
#define BUT9_REVERSE    false
#define BUT10_REVERSE   false

// active      (is this button populated (available on the enclosure))
#define BUT1_ACTIVE     true
#define BUT2_ACTIVE     true
#define BUT3_ACTIVE     true
#define BUT4_ACTIVE     true
#define BUT5_ACTIVE     true
#define BUT6_ACTIVE     true
#define BUT7_ACTIVE     true
#define BUT8_ACTIVE     true
#define BUT9_ACTIVE     true
#define BUT10_ACTIVE    true

// pullup      (should the hardware pull-up be used)
#define BUT1_PULLUP     false
#define BUT2_PULLUP     false
#define BUT3_PULLUP     false
#define BUT4_PULLUP     false
#define BUT5_PULLUP     false
#define BUT6_PULLUP     false
#define BUT7_PULLUP     false
#define BUT8_PULLUP     false
#define BUT9_PULLUP     false
#define BUT10_PULLUP    false

// TODO name these properly
// name        (what is the name for the buttons)
#define BUT1_NAME       "BUT1"
#define BUT2_NAME       "BUT2"
#define BUT3_NAME       "BUT3"
#define BUT4_NAME       "BUT4"
#define BUT5_NAME       "BUT5"
#define BUT6_NAME       "BUT6"
#define BUT7_NAME       "BUT7"
#define BUT8_NAME       "BUT8"
#define BUT9_NAME       "BUT9"
#define BUT10_NAME      "BUT10"

// num pots    (how many pots does the hardware support?)
#define NUM_POTS        4
// pins        (what are the A pins associated with the pots?)
#define POT1_PIN          2
#define POT2_PIN          3
#define POT3_PIN          4
#define POT4_PIN          6

// reverse     (should the reading be reversed?)
#define POT1_REVERSE    false
#define POT2_REVERSE    false
#define POT3_REVERSE    false
#define POT4_REVERSE    false

// active      (is this button populated (available on the enclosure))
#define POT1_ACTIVE     true
#define POT2_ACTIVE     true
#define POT3_ACTIVE     true
#define POT4_ACTIVE     true

// play        (what is the amount of play allowed? 0.002 - 0.005 is usally good
#define POT1_PLAY       0.004
#define POT2_PLAY       0.004
#define POT3_PLAY       0.004
#define POT4_PLAY       0.004

// name        (what is the name for the pot?)
#define POT1_NAME       "POT1"
#define POT2_NAME       "POT2"
#define POT3_NAME       "POT3"
#define POT4_NAME       "POT4"

#endif // ARTEFACT_TYPE and HV_MAJOR

// TODO - need  to move some of this to the EEPROM storage, and add a flag in the standard configuratui file to  either read that information or to write it
// how long does the microphone test routine last for in the feature collector testMicrophone() function
#define MICROPHONE_TEST_DURATION  1500
#define LUX_TEST_DURATION         1000

////////////////////////////////////////////////////////////////////////////
///////////////////////// Jumper Settings //////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// turn on/off reading jumpers in setup (if off take the default "true" values for jumper bools
#define JUMPERS_POPULATED               1

#endif // __HARDWARE_CONFIGURATION_H__

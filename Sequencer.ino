//
// Sequencer
//
// Smart sequencing for audio equipment at GFC
// Initially developed with [embedXcode](https://embedXcode.weebly.com)
//
// Author     Max Nedorezov
//
// Date     2/9/19 3:27 PM
// Version    1.0
//
// Copyright  © Max Nedorezov, 2019
//

#include "Arduino.h"
#include "Timer/Timer.h"

/*
 * RAW        - 5-12V power
 * GND        - Obvious
 * Pin 2  IN  - Remote control (GND when on, floating otherwise)
 * Pin 3  IN  - Previous Arduino status
 * Pin 4  OUT - Current status based on power out and previous arduino matching
 * Pin 5  OUT - Power control (relay control)
 * Pin 6  OUT - Status LED
 * Pin 7  IN  - Status of first power bank of ASD-120 2.0 - HIGH means bank is off
 * Pin 8  IN  - Status of last power bank of ASD-120 2.0 - HIGH means bank is off
 * Pin 9  OUT - Special OUT that matches state of ASD-120 2.0 having all of its
 *              power banks either on or off.
 * Pin 12 OUT - GND for relay control
 *
 * First Arduino is the one that should turn on first and turn off last.
 * Previous Arduino is the one that turns on second and turns off second to last.
 *
 * Principle of operation:
 * All arduino's wait for remote signal to specify power control state.
 * Arduino sets a timer for the power control to turn ON/OFF based on remote state.
 * While all arduino's are turning ON/OFF blink status LED.
 * When all arduino's match state, status LED specifies current power control state.
 *
 * ASD-120 has its own sequencing built-in where it sequences its 6 power banks.
 * In order to properly pass on the signal of whether the ASD-120 is completely
 * on or off, the first and last power bank need to be monitored until they match
 * each other. If they do not match then it is either not on or off completely.
 * Due to this, the status of the Arduino needs to be passed appropriately to
 * the next one in the chain.
 *
 * Connections:
 * Pin 2 <- Remote control
 * Pin 3 <- Pin 4 of previous Arduino (if last, then own pin 5 or the special
            pin 9 for ASD-120)
 * Pin 4 -> Pin 3 of next Arduino (if first, then no connection)
 * Pin 5 -> Power relay control (if last, also into own pin 3)
 * Pin 6 -> Status LED connected to first arduino, otherwise no connection
 */

// Unit 1 (mixer):
// ON_DELAY: 0
// OFF_DELAY: 30

// Unit 2 (stagebox):
// ON_DELAY: 0
// OFF_DELAY: 30

// Unit 3 (amplifiers):
// ON_DELAY: 45
// OFF_DELAY: 0

// Time in milliseconds to turn on after remote changes to on
#define ON_DELAY 0 * 1000UL

// Time in milliseconds to turn off after remote changes to off
#define OFF_DELAY 30 * 1000UL

// Pins used for different functions
#define REMOTE_IN 2
#define PREVIOUS_IN 3
#define CURRENT_OUT 4
#define POWER_OUT 5
#define STATUS_OUT 6
#define FIRST_BANK_IN 7
#define LAST_BANK_IN 8
#define BANKS_MATCH_OUT 9
#define GROUND_OUT 12

// State of remote
volatile bool remote_state = false;

// State of previous Arduino
volatile bool previous_state = false;

// Current power control state
bool current_power_state = false;

// Toggle the current power control
void toggle_on();
void toggle_off();

// ISR for the remote changing state
void remote_isr();

// ISR for the previous arduino changing state
void previous_isr();

// Timer used for toggling the state
Timer timer;
int8_t on_timer = -1;
int8_t off_timer = -1;
int8_t status_led_timer = -1;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // Reflects power control state
    pinMode(GROUND_OUT, OUTPUT);
    pinMode(REMOTE_IN, INPUT_PULLUP); // Need pullup otherwise left floating
    pinMode(PREVIOUS_IN, INPUT);
    pinMode(CURRENT_OUT, OUTPUT);
    pinMode(POWER_OUT, OUTPUT);
    pinMode(STATUS_OUT, OUTPUT);
    pinMode(FIRST_BANK_IN, INPUT_PULLUP);
    pinMode(LAST_BANK_IN, INPUT_PULLUP);
    pinMode(BANKS_MATCH_OUT, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(REMOTE_IN), remote_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PREVIOUS_IN), previous_isr, CHANGE);

    // Set everything initially off
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(CURRENT_OUT, LOW);
    digitalWrite(POWER_OUT, LOW);
    digitalWrite(STATUS_OUT, LOW);
    digitalWrite(BANKS_MATCH_OUT, LOW);
    digitalWrite(GROUND_OUT, LOW);

    Serial.begin(115200);
    Serial.println("Start sequencing!");
}

void loop()
{
    timer.update();

    // The reason for using a separate OUTPUT pin for the ASD-120 instead of
    // just manually changing the previous_state variable is because I want to
    // use the same exact code without changes for the other sequencer Arduinos.

    // Both banks match HIGH (this means the banks in the ASD-120 are all off)
    if (digitalRead(FIRST_BANK_IN) && digitalRead(LAST_BANK_IN)) {
        digitalWrite(BANKS_MATCH_OUT, LOW);
        // Serial.println("ASD-120 banks all OFF");
    }

    // Both banks match LOW (this means the banks in the ASD-120 are all on)
    else if (!digitalRead(FIRST_BANK_IN) && !digitalRead(LAST_BANK_IN)) {
      digitalWrite(BANKS_MATCH_OUT, HIGH);
    //   Serial.println("ASD-120 banks all ON");
    }

    // Current output only changes when the current arduino and previous arduino
    // are done changing state based on the set remote control.
    if (remote_state == current_power_state && remote_state == previous_state) {
        timer.stop(status_led_timer);
        digitalWrite(CURRENT_OUT, remote_state);
        digitalWrite(STATUS_OUT, remote_state);
    }
}

void toggle_on()
{
    current_power_state = true;
    digitalWrite(POWER_OUT, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Toggle ON");
}

void toggle_off()
{
    current_power_state = false;
    digitalWrite(POWER_OUT, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Toggle OFF");
}

void remote_isr()
{
    // Remote change to GND means turn on
    remote_state = !digitalRead(REMOTE_IN);

    timer.stop(off_timer);
    timer.stop(on_timer);
    timer.stop(status_led_timer);

    if (remote_state) {
        on_timer = timer.after(ON_DELAY, toggle_on);
    } else {
        off_timer = timer.after(OFF_DELAY, toggle_off);
    }

    status_led_timer = timer.oscillate(STATUS_OUT, 250, remote_state);
}

void previous_isr()
{
    previous_state = digitalRead(PREVIOUS_IN);
}

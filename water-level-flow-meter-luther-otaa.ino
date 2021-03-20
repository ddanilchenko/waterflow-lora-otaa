/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include "Arduino.h"
#include <EEPROM.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h> //https://github.com/ElectronicCats/CayenneLPP
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x59, 0x2B, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xEF, 0x67, 0x16, 0x5C, 0xB7, 0xCE, 0x35, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x60, 0x2E, 0x4E, 0xC0, 0x16, 0xEB, 0x40, 0x39, 0x03, 0xCE, 0xD6, 0x4B, 0xA1, 0xBA, 0xEB, 0xDD };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

//Luther !!!
volatile int count = 0;
const unsigned int confirmedStep = 5;

const  lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9, 
    .dio = {2, 3, 4},
};

#define SENSOR_POWER_PIN 8

#define HALL_SENSOR_PIN_1 7
#define HALL_SENSOR_PIN_2 6
#define HALL_SENSOR_PIN_3 5

#define PROBE0_PIN A0
#define PROBE1_PIN A1
#define PROBE2_PIN A2
#define PROBE3_PIN A3

#define NUMBER_OF_HALL_SENSORS 4

#define EEPROM_RESET_CMD 0x42

const int hallSensorPin[NUMBER_OF_HALL_SENSORS] = {HALL_SENSOR_PIN_1, HALL_SENSOR_PIN_2, HALL_SENSOR_PIN_3, NOT_A_PIN};

#define SLEEP_TIME_MICROS 6e6

int delayMillis = SLEEP_TIME_MICROS / 1000;
unsigned long lastExecutionTime = 0;

const int EEPROM_START_ADDR = 0;

// count how many pulses!
volatile uint16_t pulses[NUMBER_OF_HALL_SENSORS];

// track the state of the pulse pin
volatile uint8_t lastflowpinstate[NUMBER_OF_HALL_SENSORS];
// you can try to keep time of how long it is between pulses
volatile uint32_t lastflowratetimer[NUMBER_OF_HALL_SENSORS];
// and use that to calculate a flow rate
volatile float flowrate[NUMBER_OF_HALL_SENSORS];

// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    int sensorPin = hallSensorPin[i];
    if (sensorPin == NOT_A_PIN) {
      continue;
    }
    
    uint8_t x = digitalRead(sensorPin);

    if (x == lastflowpinstate[i]) {
      lastflowratetimer[i]++;
      continue; // nothing changed!
    }

    if (x == HIGH) {
      //low to high transition!
      pulses[i]++;
      EEPROM.put(EEPROM_START_ADDR + i * sizeof(pulses[i]), pulses[i]);
    }

    lastflowpinstate[i] = x;
    //TODO calculate proper flow rate here!!!
    flowrate[i] = 1000.0;
    flowrate[i] /= lastflowratetimer[i];  // in hertz
    lastflowratetimer[i] = 0;
  }
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}



void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            if (LMIC.dataLen == 1 && LMIC.frame[LMIC.dataBeg] == EEPROM_RESET_CMD) {
              resetEEPROM();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      readSensorDataAndSend();
      Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void resetEEPROM() {
  Serial.println(F("resetting EEPROM..."));
  for(int i = 0 ; i < NUMBER_OF_HALL_SENSORS; ++i) {
    EEPROM.put(EEPROM_START_ADDR + i * sizeof(pulses[i]), 0);
  }
  Serial.println(F("done resetting eeprom"));
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));


    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
    LMIC_setLinkCheckMode(0);   // Disable link check validation
    LMIC_setAdrMode(false);     // Disable ADR
    LMIC.dn2Dr = DR_SF9; 

    initSensor();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void initHallSensors() {
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    int sensorPin = hallSensorPin[i];
    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);
    EEPROM.get(EEPROM_START_ADDR + i * sizeof(pulses[i]), pulses[i]);
    lastflowratetimer[i] = flowrate[i] = 0;
    lastflowpinstate[i] = digitalRead(sensorPin);
  }
}

void initSensor() {
  //Serial.println(F("InitSensor BEGIN"));
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(PROBE0_PIN, INPUT);
  pinMode(PROBE1_PIN, INPUT);
  pinMode(PROBE2_PIN, INPUT);
  pinMode(PROBE3_PIN, INPUT);

  initHallSensors();

  useInterrupt(true);

  sensorOff();
  Serial.println(F("InitSensor END"));
}

void sensorOn() {
  digitalWrite(SENSOR_POWER_PIN, LOW);
}

void sensorOff() {
  digitalWrite(SENSOR_POWER_PIN, HIGH); 
}

CayenneLPP lppBuffer(42);
void readSensorDataAndSend() {
  //Serial.println(F("readSensorDataAndSend BEGIN"));
  sensorOn();
  delay(200);
  int l0 = digitalRead(PROBE0_PIN);
  int l1 = digitalRead(PROBE1_PIN);
  int l2 = digitalRead(PROBE2_PIN);
  int l3 = digitalRead(PROBE3_PIN);
  sensorOff();

  int level = (l3 << 3) | (l2 << 2) | (l1 << 1) | l0;
  
  Serial.print(F("Lvl:"));
  Serial.print(level);
  
  Serial.print(F(", pulses:"));
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    Serial.print(pulses[i]);
    Serial.print(" ");
  }
  
  Serial.println();
  Serial.println(F("start sending data"));
  lppBuffer.reset();
  lppBuffer.addDigitalInput(1, level);

  
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    lppBuffer.addLuminosity(i + 2, pulses[i]);
  }
  Serial.println(F("done building buffer"));
  int confirmed = count++ == confirmedStep;
  LMIC_setTxData2(1, lppBuffer.getBuffer(), lppBuffer.getSize(), confirmed);
  if (confirmed) {
    count = 0;
  }
  Serial.println(F("done sending data"));
}

void loop() {
    os_runloop_once();
}

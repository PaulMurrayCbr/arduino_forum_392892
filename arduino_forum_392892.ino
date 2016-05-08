// #define DEBUG

#ifdef DEBUG
#define log(a) Serial.print(a)
#define logln(a) Serial.println(a)
#else
#define log(a)
#define logln(a)
#endif

///////////// STATE MACHINES /////////////

enum PolarityState {
  NOT_DETERMINED = 0,
  FORWARD = 1,
  REVERSE = 2,
  ERROR_CHECKING_POLARITY = 4
} polarityState = NOT_DETERMINED;

enum CycleState {
  NOT_STARTED = 0,
  CHARGING = 1,
  CHARGING_TEST_VOLTAGE = 2,
  DISCHARGING = 3,
  DISCHARGING_TEST_VOLTAGE = 4
} cycleState = NOT_STARTED;

unsigned long cycleStateStartMs = 0;

#ifdef DEBUG
// these are the DEBUG/TEST values

// charge for 2 seconds, then check voltage
const unsigned long chargeTimeMs = 5L * 1000L;
// turn off relay .5 seconds to check the voltage
const unsigned long voltageCheckMs = 500L;
// discharge for 1 seconds, then check voltage
const unsigned long dischargeTimeMs = 3L * 1000L;
#else 


// charge for 5 minutes, then check voltage
const unsigned long chargeTimeMs = 5L * 60L * 1000L;

// turn off relay 5 seconds to check the voltage
const unsigned long voltageCheckMs = 5L * 1000L;

// discharge for 2 minutes, then check voltage
const unsigned long dischargeTimeMs = 2L * 60L * 1000L;
#endif


const int lowVThreshhold = 40; // .2v / 5v * 1023
const int highVThreshhold = 450; // 2.2v / 5v * 1023

///////////// COMPONENTS /////////////

struct BlinkyLight {
  const byte pin;
  unsigned long *blinkMs = NULL;
  int i = 0;
  unsigned long mostRecentMs;

  BlinkyLight(const byte pin):
    pin(pin)
  {
  }

  void setup() {
    pinMode(pin, OUTPUT);
  }

  void loop() {
    if (!blinkMs || !*blinkMs) return;

    if (millis() - mostRecentMs >=  blinkMs[i]) {
      i++;
      if (blinkMs[i] == 0) i = 0;

      digitalWrite(pin, !digitalRead(pin));
      mostRecentMs = millis();
    }
  }

  void setBlink(unsigned long *blink) {
    if (!blink || !*blink) {
      blinkMs = NULL;
      digitalWrite(pin, LOW);
    }
    else {
      blinkMs = blink;
      i = 0;
      digitalWrite(pin, HIGH);
      mostRecentMs = millis();
    }
  }

  void on() {
    blinkMs = NULL;
    digitalWrite(pin, HIGH);
  }

  void off() {
    blinkMs = NULL;
    digitalWrite(pin, LOW);
  }
};

extern BlinkyLight blinky; // forward declaration

struct RelayCluster {
  const byte detectVoltageAPin;
  const byte chargePin;
  unsigned long *chargeBlink;
  const byte dischargePin;
  unsigned long *dischargeBlink;

  RelayCluster(
    const byte detectVoltageAPin,
    const byte chargePin,
    unsigned long *chargeBlink,
    const byte dischargePin,
    unsigned long *dischargeBlink
  ):
    detectVoltageAPin(detectVoltageAPin),
    chargePin(chargePin),
    chargeBlink(chargeBlink),
    dischargePin(dischargePin),
    dischargeBlink(dischargeBlink)
  {
  }

  void setup() {
    pinMode(chargePin, OUTPUT);
    pinMode(dischargePin, OUTPUT);
    off();
  }

  void off() {
    digitalWrite(chargePin, LOW);
    digitalWrite(dischargePin, LOW);
    blinky.off();
  }

  void charge() {
    off();
    digitalWrite(chargePin, HIGH);
    blinky.setBlink(chargeBlink);
  }

  void discharge() {
    off();
    digitalWrite(dischargePin, HIGH);
    blinky.setBlink(dischargeBlink);
  }

  int readVoltage() {
    return analogRead(detectVoltageAPin);
  }
};


///////////// PINOUT /////////////

unsigned long dischargeBlink[] = {1000, 1000, 0};
unsigned long forwardChargeBlink[] = {250, 250, 250 * 3, 250 * 7, 0};
unsigned long reverseChargeBlink[] = {250 * 3, 250, 250, 250 * 7, 0};
unsigned long errorBlink[] = {250, 250, 0};

RelayCluster forward(A0, 10, forwardChargeBlink, 11, dischargeBlink);
RelayCluster reverse(A1, 12, reverseChargeBlink, 11, dischargeBlink);
RelayCluster *cluster; // should use a refernce, but it's being a nuisance
BlinkyLight blinky(13);

void setup() {
#ifdef DEBUG
  Serial.begin(57600);
  while (!Serial);
#endif

  forward.setup();
  reverse.setup();
  blinky.setup();

  blinky.on();
}

void loop() {
  blinky.loop();
  polarityState_loop();
  cycleState_loop();
}

void polarityState_loop() {
  if (polarityState != NOT_DETERMINED) return;

  boolean forwardOk = forward.readVoltage() > lowVThreshhold;
  boolean reverseOk = reverse.readVoltage() > lowVThreshhold;

  if (forwardOk && reverseOk) {
    polarityState = ERROR_CHECKING_POLARITY;
    blinky.setBlink(errorBlink);
    // the sketch is now locked in this state and will need to be reset
  }
  else if (forwardOk) {
    polarityState = FORWARD;
  }
  else if (reverseOk) {
    polarityState = REVERSE;
  }

}


void cycleState_loop() {
  switch (cycleState) {
    case NOT_STARTED:
      if (polarityState != FORWARD && polarityState != REVERSE) break;

      logln("NOT_STARTED");

      // start the charge cycle appropriate for the polarity
      if (polarityState == FORWARD) {
        cluster = &forward;
      }
      else {
        cluster = &reverse;
      }

      logln("-> CHARGING");

      cluster->charge();
      cycleState = CHARGING;
      cycleStateStartMs = millis();
      break;

    case CHARGING:
      if (millis() - cycleStateStartMs < chargeTimeMs) break;

      logln("CHARGING");

      // transition to test voltage

      logln("-> CHARGING_TEST_VOLTAGE");
      cluster->off();
      blinky.on();
      cycleState = CHARGING_TEST_VOLTAGE;
      cycleStateStartMs = millis();
      break;

    case CHARGING_TEST_VOLTAGE:
      if (millis() - cycleStateStartMs < voltageCheckMs) break;

      logln("CHARGING_TEST_VOLTAGE");

      // transition to discharge or to continue charging

      if (cluster->readVoltage() < highVThreshhold) {
        logln("-> CHARGING");
        cluster->charge();
        cycleState = CHARGING;
        cycleStateStartMs = millis();
      }
      else {
        logln("-> DISCHARGING");
        cluster->discharge();
        cycleState = DISCHARGING;
        cycleStateStartMs = millis();
      }
      break;

    case DISCHARGING:
      if (millis() - cycleStateStartMs < dischargeTimeMs) break;

      logln("DISCHARGING");

      // transition to test voltage

      logln("-> DISCHARGING_TEST_VOLTAGE");
      cluster->off();
      blinky.on();
      cycleState = DISCHARGING_TEST_VOLTAGE;
      cycleStateStartMs = millis();
      break;

    case DISCHARGING_TEST_VOLTAGE:
      if (millis() - cycleStateStartMs < voltageCheckMs) break;

      logln("DISCHARGING_TEST_VOLTAGE");
      // transition to contuniung discharge or to flipping the polarity and restarting cycle

      if (cluster->readVoltage() >= lowVThreshhold) {
        logln("-> DISCHARGING");
        cluster->discharge();
        cycleState = DISCHARGING;
        cycleStateStartMs = millis();
      }
      else {
        logln("-> NOT_STARTED");
        blinky.off();
        cycleState = NOT_STARTED;
        polarityState = polarityState == REVERSE ? FORWARD : REVERSE;
      }
      break;
  }
}




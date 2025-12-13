//CPE Final Project
//SOLO Group
//Brandon Lafferty

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>

// Component and Pin Definitions
// DHT11
#define DHTPIN   22
#define DHTTYPE  DHT11

// ADC channel for water sensor (ADC0 = PF0)
#define WATER_ADC_CHANNEL 0

// Fan on D8 = PH5
#define FAN_DDR   DDRH
#define FAN_PORT  PORTH
#define FAN_BIT   PH5

// System Buttons:
// Start: D2 = PE4
#define START_PINREG   PINE
#define START_BIT      PE4
#define START_DDR      DDRE

// Stop: D3 = PE5
#define STOP_PINREG    PINE
#define STOP_BIT       PE5
#define STOP_DDR       DDRE

// Reset: D4 = PG5
#define RESET_PINREG   PING
#define RESET_BIT      PG5
#define RESET_DDR      DDRG

// Vent Buttons:
// Left: D5 = PE3
#define VENT_LEFT_PINREG   PINE
#define VENT_LEFT_BIT      PE3
#define VENT_LEFT_DDR      DDRE
// Right: D6 = PH3
#define VENT_RIGHT_PINREG  PINH
#define VENT_RIGHT_BIT     PH3
#define VENT_RIGHT_DDR     DDRH

// LEDs:
// Yellow D40 = PG1
#define LED_DISABLED_DDR   DDRG
#define LED_DISABLED_PORT  PORTG
#define LED_DISABLED_BIT   PG1

// Green D41 = PG0
#define LED_IDLE_DDR       DDRG
#define LED_IDLE_PORT      PORTG
#define LED_IDLE_BIT       PG0

// Red D42 = PL7
#define LED_ERROR_DDR      DDRL
#define LED_ERROR_PORT     PORTL
#define LED_ERROR_BIT      PL7

// Blue D43 = PL6
#define LED_RUNNING_DDR    DDRL
#define LED_RUNNING_PORT   PORTL
#define LED_RUNNING_BIT    PL6

// LCD
const int LCD_RS = 30;
const int LCD_E  = 31;
const int LCD_D4 = 32;
const int LCD_D5 = 33;
const int LCD_D6 = 34;
const int LCD_D7 = 35;

// Stepper Motor
const int STEPS_PER_REV = 2048;
const int STEPPER_IN1 = 50;
const int STEPPER_IN2 = 51;
const int STEPPER_IN3 = 52;
const int STEPPER_IN4 = 53;

// Constants
const float TEMP_THRESHOLD_C = 24.3;            
const int   WATER_THRESHOLD  = 300;              
const unsigned long SENSOR_UPDATE_MS = 60000UL;    
const int   VENT_STEP_SIZE   = 10;               
const unsigned long VENT_STEP_INTERVAL_MS = 150;

//  Helper macros 
#define SET_BIT(reg, bit)    ((reg) |= (1 << (bit)))
#define CLR_BIT(reg, bit)    ((reg) &= ~(1 << (bit)))
#define READ_BIT(pinreg, bit) ((pinreg) & (1 << (bit)))

//  Global states
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR_STATE
};

volatile bool startRequested = false;
State currentState = DISABLED;

float lastTempC = 0.0f;
float lastHumidity = 0.0f;
unsigned long lastSensorUpdate = 0;

int ventPositionSteps = 0;
unsigned long lastVentStepTime = 0;

// Library objects 
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHTPIN, DHTTYPE);
RTC_DS3231 rtc;
Stepper ventStepper(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

// UART0 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void uart0_init(uint32_t baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart0_putc(char c) {
  while (!(UCSR0A & (1 << UDRE0))) {
  }
  UDR0 = c;
}

void uart0_print(const char *s) {
  while (*s) {
    uart0_putc(*s++);
  }
}

void uart0_println(const char *s) {
  uart0_print(s);
  uart0_print("\r\n");
}

// ADC (water level)
void adc_init() {
  ADMUX = (1 << REFS0);          
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) {
  }
  return ADC;
}

bool isWaterLow() {
  uint16_t value = adc_read(WATER_ADC_CHANNEL);
  return value < WATER_THRESHOLD;
}

// Fan, LEDs, buttons
void fanOn() {
  SET_BIT(FAN_PORT, FAN_BIT);
}

void fanOff() {
  CLR_BIT(FAN_PORT, FAN_BIT);
}

void updateLEDs() {
  CLR_BIT(LED_DISABLED_PORT, LED_DISABLED_BIT);
  CLR_BIT(LED_IDLE_PORT,     LED_IDLE_BIT);
  CLR_BIT(LED_ERROR_PORT,    LED_ERROR_BIT);
  CLR_BIT(LED_RUNNING_PORT,  LED_RUNNING_BIT);

  switch (currentState) {
    case DISABLED:
      SET_BIT(LED_DISABLED_PORT, LED_DISABLED_BIT);
      break;
    case IDLE:
      SET_BIT(LED_IDLE_PORT, LED_IDLE_BIT);
      break;
    case RUNNING:
      SET_BIT(LED_RUNNING_PORT, LED_RUNNING_BIT);
      break;
    case ERROR_STATE:
      SET_BIT(LED_ERROR_PORT, LED_ERROR_BIT);
      break;
  }
}

bool startButtonPressed() {
  return READ_BIT(START_PINREG, START_BIT);
}
bool stopButtonPressed() {
  return READ_BIT(STOP_PINREG, STOP_BIT);
}
bool resetButtonPressed() {
  return READ_BIT(RESET_PINREG, RESET_BIT);
}
bool ventLeftPressed() {
  return READ_BIT(VENT_LEFT_PINREG, VENT_LEFT_BIT);
}
bool ventRightPressed() {
  return READ_BIT(VENT_RIGHT_PINREG, VENT_RIGHT_BIT);
}

// Event Logging
void logEvent(const char *msg) {
  DateTime now = rtc.now();
  char line[96];
  snprintf(line, sizeof(line),
           "%04u/%02u/%02u %02u:%02u:%02u - %s",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second(),
           msg);
  uart0_println(line);
}

void logVentPosition() {
  char buf[64];
  snprintf(buf, sizeof(buf), "Vent position steps: %d", ventPositionSteps);
  logEvent(buf);
}

// LCD helpers

void showStatusLine() {
  lcd.setCursor(0, 0);
  switch (currentState) {
    case DISABLED:   lcd.print("State:DISABLED "); break;
    case IDLE:       lcd.print("State:IDLE     "); break;
    case RUNNING:    lcd.print("State:RUNNING  "); break;
    case ERROR_STATE:lcd.print("State:ERROR    "); break;
  }
}

void showTempHumLine() {
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++) lcd.print(' ');
  lcd.setCursor(0, 1);

  lcd.print("T:");
  lcd.print(lastTempC, 1);
  lcd.print("C H:");
  lcd.print(lastHumidity, 0);
  lcd.print("%");
}

void showErrorLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water level low");
  lcd.setCursor(0, 1);
  lcd.print("Check reservoir ");
}

// State control
void setState(State newState, const char *reason) {
  currentState = newState;

  switch (currentState) {
    case DISABLED:
      fanOff();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("System DISABLED");
      lcd.setCursor(0, 1);
      lcd.print("Press START btn ");
      break;

    case IDLE:
      fanOff();
      lcd.clear();
      showStatusLine();
      showTempHumLine();
      break;

    case RUNNING:
      fanOn();
      lcd.clear();
      showStatusLine();
      showTempHumLine();
      break;

    case ERROR_STATE:
      fanOff();
      showErrorLCD();
      break;
  }

  updateLEDs();

  char buf[80];
  snprintf(buf, sizeof(buf), "State changed to %d (%s)", currentState, reason);
  logEvent(buf);
}

//ISR for START button
void START_BUTTON_ISR() {
  startRequested = true;
}

// Vent Control
void UPDATE_VENT_CONTROL() {
  if (currentState == DISABLED) return;

  unsigned long now = millis();
  if (now - lastVentStepTime < VENT_STEP_INTERVAL_MS) return;

  int steps = 0;
  bool left = ventLeftPressed();
  bool right = ventRightPressed();

  if (left && !right) {
    steps = -VENT_STEP_SIZE;
  } else if (right && !left) {
    steps = VENT_STEP_SIZE;
  }

  if (steps != 0) {
    ventStepper.step(steps);
    ventPositionSteps += steps;
    lastVentStepTime = now;
    logVentPosition();
  }
}

//Sensor update
void READ_SENSORS_AND_UPDATE_LCD() {
  if (currentState == DISABLED) 
    return;

  unsigned long now = millis();

  if (now - lastSensorUpdate < SENSOR_UPDATE_MS) 
    return;

  lastSensorUpdate = now;
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    uart0_println("DHT read error");
    return;  
  }

  lastHumidity = h;
  lastTempC = t;

  if (currentState == ERROR_STATE) {
    lcd.setCursor(0, 1);
  } else {
    lcd.clear();
    showStatusLine();
    lcd.setCursor(0, 1);
  }
  showTempHumLine();

  //Log the reading with RTC time
char logbuf[80];
int t10 = (int)(lastTempC * 10);
int h10 = (int)(lastHumidity * 10);

snprintf(logbuf, sizeof(logbuf),
         "Sensor: T=%d.%dC H=%d.%d%%",
         t10 / 10, abs(t10 % 10),
         h10 / 10, abs(h10 % 10));

logEvent(logbuf);
}

void setup() {
  SET_BIT(FAN_DDR, FAN_BIT);

  // Buttons as inputs
  CLR_BIT(START_DDR, START_BIT);
  CLR_BIT(STOP_DDR,  STOP_BIT);
  CLR_BIT(RESET_DDR, RESET_BIT);
  CLR_BIT(VENT_LEFT_DDR,  VENT_LEFT_BIT);
  CLR_BIT(VENT_RIGHT_DDR, VENT_RIGHT_BIT);

  // LEDs
  SET_BIT(LED_DISABLED_DDR, LED_DISABLED_BIT);
  SET_BIT(LED_IDLE_DDR,     LED_IDLE_BIT);
  SET_BIT(LED_ERROR_DDR,    LED_ERROR_BIT);
  SET_BIT(LED_RUNNING_DDR,  LED_RUNNING_BIT);

  // ADC and UART
  adc_init();
  uart0_init(9600);
  uart0_println("Booting cooler...");

  // Libraries
  lcd.begin(16, 2);   
  dht.begin();

  if (!rtc.begin()) {
    uart0_println("RTC not found!");
  } else if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    logEvent("RTC adjusted to compile time");
  }

  ventStepper.setSpeed(10); 

// LED test
SET_BIT(LED_DISABLED_PORT, LED_DISABLED_BIT);
SET_BIT(LED_IDLE_PORT,     LED_IDLE_BIT);
SET_BIT(LED_ERROR_PORT,    LED_ERROR_BIT);
SET_BIT(LED_RUNNING_PORT,  LED_RUNNING_BIT);
for (volatile uint32_t i = 0; i < 200000; ++i) { }
CLR_BIT(LED_DISABLED_PORT, LED_DISABLED_BIT);
CLR_BIT(LED_IDLE_PORT,    LED_IDLE_BIT);
CLR_BIT(LED_ERROR_PORT,   LED_ERROR_BIT);
CLR_BIT(LED_RUNNING_PORT,  LED_RUNNING_BIT);

attachInterrupt(digitalPinToInterrupt(2), START_BUTTON_ISR, RISING);
setState(DISABLED, "Power-up");
}

void loop() {
  if (startRequested) {
    cli();
    bool doStart = startRequested;
    startRequested = false;
    sei();

    if (doStart && currentState == DISABLED) {
      setState(IDLE, "Start button");
    }
  }

  if (currentState != DISABLED && stopButtonPressed()) {
    setState(DISABLED, "Stop button");
  }

  if (currentState == ERROR_STATE && resetButtonPressed()) {
    if (!isWaterLow()) {
      setState(IDLE, "Reset, water OK");
    } else {
      logEvent("Reset pressed but water still low");
    }
  }

  READ_SENSORS_AND_UPDATE_LCD();
  UPDATE_VENT_CONTROL();

  switch (currentState) {
    case IDLE:
      if (isWaterLow()) {
        setState(ERROR_STATE, "Water low in IDLE");
      } else if (lastTempC >= TEMP_THRESHOLD_C) {
        setState(RUNNING, "Temp above threshold");
      }
      break;

    case RUNNING:
      if (isWaterLow()) {
        setState(ERROR_STATE, "Water low in RUNNING");
      } else if (lastTempC < TEMP_THRESHOLD_C) {
        setState(IDLE, "Temp below threshold");
      }
      break;

    case DISABLED:
    case ERROR_STATE:
      break;
  }
}

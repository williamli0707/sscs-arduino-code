#include <LiquidCrystal.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <EEPROM.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
MPU6050 accelgyro;

const int LCD_POWER_PIN = 6, ACC_POWER_PIN = 7, BTN_PIN = 10;
const int NUM_SAMPLES = 20, IDLE_TIMEOUT = 10000, BTN_HOLD = 5000;
const int NUM_SLIDES = 3, NUM_FACTS = 3;
const float STRIDE_LENGTH = 2.5; //feet; walking
const float SAFE_VOLTAGE = 4.8; //determined experimentally, as the getBandgap() function does not just output the combined voltages of the batteries
float STEP_CUTOFF = 11;
bool lcd_power_state = 1, acc_power_state = 0, flag = 0, button_pressed = 0, walking = 0;
int16_t ax, ay, az, gx, gy, gz;
long step_count = 0, step_count_total = 0; 
float miles_total = 0, miles = 0;
long button_last_pressed = 0, button_first_pressed = 0;
long slide_last_changed;
int slide = 0, last_slide = -1;

void toggleLCDPower() {
  digitalWrite(LCD_POWER_PIN, lcd_power_state ? LOW : HIGH);
  lcd_power_state = !lcd_power_state;
  last_slide = -1;
  // lcd.clear();
  if(!walking) {
    
  }
}
void toggleACCPower() {
  digitalWrite(ACC_POWER_PIN, acc_power_state ? LOW : HIGH);
  acc_power_state = !acc_power_state;
}

void startWalk() {
  initialize();
  step_count = 0;
  miles_total = 0;
  walking = 1;
}

long EEPROMReadlong(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

void endWalk() {
  walking = 0;
  step_count_total += step_count;
  miles = ((int) ((step_count * STRIDE_LENGTH / 5280) * 10))/10.0;
  miles_total += miles;
  EEPROMWritelong(0, step_count_total);
  EEPROMWritelong(4, miles_total);
  lcd.clear();
  lcd.print("You walked ");
  lcd.print(step_count);
  lcd.setCursor(0, 1);
  lcd.print("steps and ");
  lcd.print(miles);
  lcd.print("mi!");
  step_count = miles = 0;
  delay(3000);
}

void initialize() {
  lcd.clear();
  lcd.print("Initializing....");
  lcd.setCursor(0, 1);
  lcd.print("Hold still!");
  unsigned long start = millis();
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.CalibrateAccel(6);
  accelgyro.CalibrateGyro(6);
  lcd.clear();
  lcd.print("Started!");
}  // Initialize

void setup() {
  Serial.begin(9600);
  //read from EEPROM
  // step_count_total = EEPROM.get(0);
  // miles_total = EEPROM.get(6);
  // EEPROM.get(0, step_count_total);
  // EEPROM.get(6, miles_total);
  step_count_total = EEPROMReadlong(0);
  miles_total = EEPROMReadlong(4);

  Serial.println("step count total: ");
  Serial.println(step_count_total);

  Serial.println("mile total: ");
  Serial.println(miles_total);

  pinMode(LCD_POWER_PIN, OUTPUT);
  pinMode(ACC_POWER_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  digitalWrite(LCD_POWER_PIN, HIGH);

  // lcd.createChar(0, bottle);
  lcd.begin(16, 2);
  lcd.print("Started!");
}

void loop() {
  if(getBandgap() < SAFE_VOLTAGE) {
    if(lcd_power_state) toggleLCDPower();
    if(acc_power_state) toggleACCPower();
    return;
  }

  long time = millis();
  if(digitalRead(BTN_PIN) == HIGH) {
    if(!button_pressed) button_first_pressed = time;
    button_last_pressed = time;
    button_pressed = 1;
    if(!lcd_power_state) toggleLCDPower();

    if(time - button_first_pressed >= BTN_HOLD) {
      if(walking) endWalk();
      else startWalk();
    }
  }
  else {
    if(button_pressed) { //add to slide count if it was just let go
      slide++;
      slide %= NUM_SLIDES;
    }
    button_pressed = 0;
  }
  
  
  if(lcd_power_state && time - button_last_pressed >= IDLE_TIMEOUT) {
    toggleLCDPower();
    slide = 0;
    slide_last_changed = -500;
  }
  if(walking) {
    float sumaccel = 0;
    for(int i = 0;i < NUM_SAMPLES;i++) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      sumaccel += sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2)) / 16384 * 9.81;
      delay(5);
    }
    sumaccel /= NUM_SAMPLES;

    if(sumaccel >= STEP_CUTOFF) {
      step_count += flag == 0;
      flag = 1;
      Serial.println("printing");
      lcd.clear();
      lcd.print("Steps: ");
      lcd.print(step_count);
    }
    else flag = 0;
    // Serial.println(sumaccel);
  }
  else {
    // if(time - slide_last_changed < 500) return;
    if(last_slide == slide) return;
    lcd.clear();
    if(slide == 0){
      lcd.print("You've walked ");
      lcd.setCursor(0, 1);
      lcd.print(miles_total);
      lcd.print(" mi total");
    }
    else if(slide == 1) {
      lcd.print("You've walked ");
      lcd.setCursor(0, 1);
      lcd.print(step_count_total);
      lcd.print(" steps total");
    }
    else if(slide == 2) {
      int rand = (int) random(NUM_FACTS);
      if(rand == 0) {
        int animals = round(miles_total * 0.0225); //taken from doc for animals saved per mile
        if(animals  >= 1) {
          lcd.print("You've saved ");
          lcd.print(animals);
          lcd.setCursor(0, 1);
          lcd.print("sea animal(s)!");
        }
        else rand++;
      }
      if(rand == 1) {
        int bottles = round(miles_total) * 5;
        lcd.print("You saved ");
        lcd.print(bottles);
        lcd.setCursor(0, 1);
        lcd.print("plastic bottles!");
      }
      if(rand == 2) {
        float money = ((int) ((miles_total * 1/6) * 100)) / 100.0;
        lcd.print("You've saved");
        lcd.setCursor(0, 1);
        lcd.print("$");
        lcd.print(money);
        lcd.print(" on gas!");
      }
    }
    last_slide = slide;
    slide_last_changed = time;
  }
}

int getBandgap(void) { //Source: https://forum.arduino.cc/t/stop-arduino-when-battery-voltage-is-low/640377

  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // For mega boards
    const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
                                                  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
                                                  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  #else
    // For 168/328 boards
    const long InternalReferenceVoltage = 1056L;  // Adjust this value to your boards specific internal BG voltage x1000
                                                  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
                                                  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  #endif
    delay(50);  // Let mux settle a little to get a more stable A/D conversion
                // Start a conversion
    ADCSRA |= _BV(ADSC);
    // Wait for it to complete
    while (((ADCSRA & (1 << ADSC)) != 0))
      ;
    // Scale the value
    int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;  // calculates for straight line value
    return results;
}
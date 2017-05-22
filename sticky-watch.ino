/*
 * ISD1700 Manager sketch
 * Author: Minori Yamashita
 * Contact: ympbyc@gmail.com
 * 
 * Thanks! to following library authors:
 * ISD1700 lib by neuron_upheaval https://forum.arduino.cc/index.php?topic=52509.0
 * Timer lib by Simon Monk http://playground.arduino.cc/Code/Timer
 * OneButton lib by Matthias Hertel http://www.mathertel.de/Arduino/OneButtonLibrary.aspx
 */

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <EEPROM.h>
#include <ISD1700.h>
#include <OneButton.h>
#include <Timer.h>
#include <MsTimer2.h>

#define SS 10
#define FIRST_ADDR 0x010
#define LAST_ADDR 0x3cf //3cf for 120, 2df for 90
#define APC 0xE1 //000011100001
#define SLOT_NUM 5

#define MAIN_BTN 3 //3
#define SUB_BTN  2 //2

#define LEDI 4 //1 for D1, 14 for A0
#define LEDN 5
#define LEDFREQ 100
#define LEDREP 5

#define REC_TIMEUP ((120 * 1000) / SLOT_NUM)
#define AVR_PWR_DWN_TIME 3000

/*int digitalPinToInterrupt (int d) {
  if (d == 2) return 0;
  if (d == 3) return 1;
  return -1;
}*/

int interrupt_main_btn = digitalPinToInterrupt(MAIN_BTN);
int interrupt_sub_btn = digitalPinToInterrupt(SUB_BTN);

ISD1700 chip(SS); // Initialize chipcorder with
                 // SS at Arduino's digital pin 10

OneButton btn(SUB_BTN, true);
OneButton btn2(MAIN_BTN, true);

Timer tim;
int tim_id;
int c_w_t_id;
int r_tu_t_id;
int avr_pd_t_id;

uint16_t apc = 0;

uint16_t slot_addrs[SLOT_NUM];
uint16_t end_addrs[SLOT_NUM];
int cur_slot = 0;
int slot_occup[SLOT_NUM];

bool recording = false;

void setup() {
  int i;
  analogReference(INTERNAL);
  Serial.begin(115200);
  /* lower power consumption */
  DDRD &= B00000011;       // set Arduino pins 2 to 7 as inputs
  DDRB = B00000000;        // set pins 8 to 13 as inputs
  PORTD |= B11111100;      // enable pullups on pins 2 to 7
  PORTB |= B11111111;      // enable pullups on pins 8 to 13
  
  // re-set pin-mode for SPI after above setting
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SS, OUTPUT);

  for (i=LEDI; i<LEDI+LEDN; i++)
    pinMode(i, OUTPUT);
  pinMode(MAIN_BTN, INPUT_PULLUP);
  pinMode(SUB_BTN, INPUT_PULLUP);

  btn.setClickTicks(300);
  btn.setPressTicks(400);
  btn2.setClickTicks(300);
  btn2.setPressTicks(1500); //press longer for erase
  btn.attachClick(playback);
  btn.attachLongPressStart(rec);
  btn.attachLongPressStop(rec_end);
  btn2.attachClick(forward);
  btn2.attachLongPressStart(erase_one);
  digitalWrite(SS, HIGH); // just to ensure
  while (!chip_wakeup())
    delay(10);
  if (chip.PU()) {
    Serial.print(chip.CMD_ERR());
    Serial.print(chip.RDY());
    Serial.println("Chip woke up!");
  }
  apc |= APC;
  chip.wr_apc2(apc);
  check_chip_err();
    
  /*chip.g_erase();
  for (i=0; i<SLOT_NUM; i++)
    EEPROM[i] = 0;
  check_chip_err();*/

  sync_eeprom();
  
  MsTimer2::set(4, led_update);
  MsTimer2::start();

  //setup memory management
  for (i=0; i<SLOT_NUM; i++) {
    slot_addrs[i] = floor((LAST_ADDR - FIRST_ADDR) / SLOT_NUM) * i + FIRST_ADDR + i;
    end_addrs[i] = floor((LAST_ADDR - FIRST_ADDR) / SLOT_NUM) * (i+1) + FIRST_ADDR + i;
    Serial.println(slot_addrs[i], HEX);
  }

  chip.pd();
  //avr_sleep();
}

void sync_eeprom () {
  int i;
  for (i=0; i<SLOT_NUM; i++)
    slot_occup[i] = EEPROM[i];  //sync eeprom status with array
}

bool chip_wakeup () {
  // Power Up
  tim.stop(avr_pd_t_id); //clear pwrdwn timeout
  Serial.println("ISD1700 wakeup");
  chip.rd_status();
  if (chip.CMD_ERR()) { Serial.println("rd_status CMD_ERR"); }
  if (! chip.PU()) { Serial.println("instructing pu");  chip.pu();}
  delay(10);
  chip.stop(); //stop whatever it's doing
  chip.clr_int();
  if (chip.RDY()) return true;
  chip.reset();
  delay(100);
  if (chip.RDY()) return true;
  led_notwakeup_pattern();
  return false;
}

bool chip_cleanup () {
  tim.stop(c_w_t_id);
  Serial.print("Status---> ");
  Serial.print(chip.CMD_ERR()? "CMD_ERR ": " ");
  Serial.print(chip.PU()? "PU ": " ");
  Serial.print(chip.RDY()? "RDY ": "Not_RDY");
  Serial.println();
  c_w_t_id = tim.every(1000, attempt_pd);
}

void attempt_pd () {
  chip.rd_status();
  if (chip.INT() || chip.EOM()) {
    tim.stop(c_w_t_id);
    //chip.clr_int();
    chip.pd();
    Serial.println(chip.CMD_ERR() ? "pd CMD_ERR" : "ISD1700 powerdown");
    avr_pd_t_id = tim.after(AVR_PWR_DWN_TIME, avr_sleep);
  }
}

void playback () {
  Serial.println("button action: playback");
  if ( ! slot_occup[cur_slot]) return;
  if ( ! chip_wakeup()) return;
  chip.set_play(slot_addrs[cur_slot], end_addrs[cur_slot]);
  if (check_chip_err()) return;
  register_led_oscillation(cur_slot, LEDFREQ);
  tim_id = tim.after(5000, cancel_led_oscillation);
  chip_cleanup();
}

void forward () {
  Serial.println("button action: forward");
  if ( ! chip_wakeup()) return;
  chip.stop();
  //chip.fwd();
  cur_slot = next_occupied_slot(cur_slot);
  if (cur_slot == 0 && ! slot_occup[cur_slot]) return; //no recording
  chip.set_play(slot_addrs[cur_slot], end_addrs[cur_slot]);
  if (check_chip_err()) return;
  register_led_oscillation(cur_slot, LEDFREQ);
  tim_id = tim.after(5000, cancel_led_oscillation);
  chip_cleanup();
}

void write_eeprom (int i, byte x) {
  EEPROM.write(i, x);
  slot_occup[i] = x;
  tim.stop(tim_id);
  cancel_led_oscillation();
}

void rec () {
  Serial.println("button action: rec");
  int next_slot;
  if ( ! chip_wakeup()) return;
  next_slot = next_available_slot(cur_slot);
  if (next_slot < 0) return;
  recording = true;
  chip.set_rec(slot_addrs[next_slot], end_addrs[next_slot]);
  write_eeprom(next_slot, 1);
  register_led_oscillation(next_slot, LEDFREQ);
  cur_slot = next_slot;
  r_tu_t_id = tim.after(REC_TIMEUP, rec_end); //ensure rec_end gets called
}

void rec_end () {
  if (! recording) return;
  recording = false;
  Serial.println("rec end");
  cancel_led_oscillation();
  tim.stop(r_tu_t_id);
  delay(10);
  if (check_chip_err()) return;
  chip.stop();
  chip_cleanup();
}

void erase_one () {
  if ( ! chip_wakeup()) return;
  chip.set_erase(slot_addrs[cur_slot], end_addrs[cur_slot]);
  delay(10);
  //if (check_chip_err()) return;
  write_eeprom(cur_slot, 0);
  cur_slot = next_occupied_slot(cur_slot);
  chip_cleanup();
}

void erase_all () {
  int i;
  if ( ! chip_wakeup()) return;
  chip.g_erase();
  delay(10);
  if (check_chip_err()) return;
  for (i=0; i<SLOT_NUM; i++)
    write_eeprom(i, 0);
  cur_slot = 0;
  chip_cleanup();
}

//interrupt function
int rom_i = 0;
int led_i_max = LEDI+(LEDN-1);
volatile int osc_i = -1;
volatile int osc_st = 0;
void led_update () {
  //led display update LOW = ON
  if (rom_i > SLOT_NUM-1) {
    rom_i = 0;
    digitalWrite(led_i_max, HIGH);
  } else {
    digitalWrite(LEDI+rom_i - 1, HIGH);
  }
  
  //osc
  if (rom_i == osc_i) {
    digitalWrite(LEDI+rom_i, !osc_st);
    osc_st = !osc_st;
  }
  //slot occupation indication
  else if (slot_occup[rom_i])
    digitalWrite(LEDI+rom_i, LOW);
  else
    digitalWrite(LEDI+rom_i, HIGH);
  rom_i++;
}

void register_led_oscillation (int rom_i, int freq) {
  tim.stop(tim_id);
  osc_i = rom_i;
}

void cancel_led_oscillation () {
  osc_i = -1;
}

bool check_chip_err () {
  if (! chip.CMD_ERR()) return false;
  error_pattern();
  return true;
}

void led_off () {
  int i;
  MsTimer2::stop();
  for (i=0; i<SLOT_NUM; i++)
    digitalWrite(LEDI+i, HIGH);
}

void error_pattern () {
  int i;
  for (i=0; i < SLOT_NUM; i++)
    slot_occup[i] = 1; //enable all leds
  MsTimer2::stop();
  cancel_led_oscillation();
  MsTimer2::set(300, led_update);
  MsTimer2::start();
  delay(1000);
  sync_eeprom(); //restore slot occupation status
  MsTimer2::stop();
  MsTimer2::set(4, led_update);
  MsTimer2::start();
}

void led_notwakeup_pattern () {
  error_pattern();
}

int next_available_slot (int cur) {
  for (int i=0; i<SLOT_NUM; i++) {
    if ( ! slot_occup[i]) return i;
  }
  return -1;
}

int next_occupied_slot (int cur) {
  int nxt = (cur + 1) % SLOT_NUM;
  for (int i=nxt; i<SLOT_NUM+nxt; i++) {
    if (slot_occup[i % SLOT_NUM]) return i % SLOT_NUM;
  }
  return 0;
}

void pinInterrupt () {
  sleep_disable(); //Wait for an interrupt, disable, and continue
  detachInterrupt(interrupt_main_btn);
  detachInterrupt(interrupt_sub_btn);
}

void avr_sleep () {
  Serial.println("avr_sleep reached");
  if (digitalRead(MAIN_BTN) == LOW || digitalRead(SUB_BTN) == LOW) return;
  sleep_enable();  //Set sleep enable (SE) bit
  attachInterrupt(interrupt_main_btn, pinInterrupt, LOW);
  attachInterrupt(interrupt_sub_btn,  pinInterrupt, LOW);
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  led_off();
  Serial.println("Arduino PWR_DOWN");
  delay(10);
  sleep_mode();    //Put the device to sleep
  
  sleep_disable(); //Wait for an interrupt, disable, and continue
  Serial.println("Arduino Wakeup");
  sync_eeprom();
  MsTimer2::set(4, led_update);
  MsTimer2::start();
}

/* 
 * Secret Arduino Voltmeter – Measure Battery Voltage 
 * https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 */
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//monitor vcc for LiPo protection
void pd_if_low_battery () {
  if (readVcc() < 2900) {
    chip.pd();
    delay(10);
    avr_sleep();
  }
}

void loop() {
  pd_if_low_battery(); 
  btn.tick();
  btn2.tick();
  tim.update();
  delay(10);
}


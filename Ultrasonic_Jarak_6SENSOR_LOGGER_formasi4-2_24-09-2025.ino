// ===== JSN-SR04T x4, ATmega328PB, NON-BLOCKING =====
// Timer1: 1 µs tick (8 MHz / 8) untuk ukur lebar ECHO
// PCINT  : deteksi rising/falling di ECHO (PD0, PB2, PD1, PB7)
// Timer2 : CTC 1 ms untuk scheduler tembak bergiliran + timeout + GAP
// LCD 16x2 + LED kedip makin cepat saat objek makin dekat

#define F_CPU 8000000UL
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include <SPI.h>
// #include <SdFat.h>
#include <SD.h>
#include <EEPROM.h>

// ===== LCD pins =============================================
const int rs=4, en=5, d4=6, d5=7, d6=8, d7=9;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// ===== RTC & SD ===========================================================
const uint8_t SD_CS = 10;
const char* LOG_PATH = "LOGGER.CSV";
const char* LOG_HEADER = "No,Jarak,Counter,Waktu Total";
bool sdReady = false;
//SdFat sd;

// ====== KONFIG LOGGER DANGER (5 detik) ======
const uint32_t LOG_PERIOD_MS = 5000;   // interval log saat danger
static bool     dangerActive = false;  // sedang dalam episode danger?
static uint16_t dangerCounter = 0;     // 1,2,3,... (tiap 5 detik)
static uint32_t dangerNextDueMs = 0;   // deadline log berikutnya

// ===== Mapping sensor =====
const uint8_t TRIG = 2; //PD2
const uint8_t TRIG2 = 17; //PC3
const uint8_t ECHO[8] = { 15, 1, 16, 18, 0, 19, 14, 17 }; //PC1 , PD1, PC2, PC4, PD0, PC5, PC0, PC3

// ====== KONFIG SENSOR ======
const uint8_t N_SENSORS = 6; // kamu baru pasang 6 dulu

// 0 = pakai TRIG, 1 = pakai TRIG2 (atur sesuai kabelmu)
const uint8_t TRIG_SEL[8]   = {1,0,0, 1,1,1, 0,0};

// Lebar pulsa TRIG per sensor (us). JSN-SR04T/AJ-SR04M biasanya OK 10–20us,
// tapi ada batch yg "senang" 100us+. Silakan sesuaikan.
const uint8_t PULSE_US[8]   = {110,110,110, 100,100,100, 100,100};

// GAP antar tembak per sensor (ms). JSN-SR04T waterproof lebih stabil di 90–120ms.
const uint16_t GAP_MS_PER[8]= {100,100,100, 40,40,40, 40,40};

// Blanking awal (cm) untuk mengabaikan echo terlalu dekat (derau/ringing)
const uint16_t BLANK_NEAR_CM = 5; // ≈ 25 cm


//===== Tombol=====
const unsigned long DEBOUNCE_MS = 25;  // 20–30 ms cukup
const uint8_t button1 = 20;

// ===== LED indikator =====
const int  buzzer = 3; // BUZZER 3 / LAMPU 19
//const int  lampu = 19; // BUZZER 3 / LAMPU 19

#define SHORT_PRESS_MAX 300
#define LONG_PRESS_MIN 500

//=====button 1==========================================================
unsigned long buttonPressTime1 = 0;
bool buttonPressed1 = false;
bool lastButtonState1 = false;
bool newShortPress1 = false;
bool newLongPress1 = false;

//==========MENU================================================================
const uint8_t JUMLAH_MENU = 4;
const char m0[] PROGMEM = "CHANGE PASS";
const char m1[] PROGMEM = "SET PARAMETER";
const char m2[] PROGMEM = "DISPLAY MODE";
const char m3[] PROGMEM = "EXIT";
const char* const MENU_TEXT[] PROGMEM = { m0, m1, m2, m3 };

bool baruMasukMenu = true;     // mirip baruMasukInputPassword
static uint8_t menuIndex = 0;  // 0..JUMLAH_MENU-1

//=======IDLE===================
unsigned long lastInteractionTime = 0;
const unsigned long IDLE_TIMEOUT = 30000;  // 30 detik untuk mati otomatis

// ====== KONFIG PASSWORD ======
char PASS_EXPECTED[7] = "000000";   // ganti sesuai password kamu
bool baruMasukInputPassword = true;

// ====== VARIABEL STATE INPUT PASSWORD ======
int8_t   pw_digits[6]= {-1,-1,-1,-1,-1,-1};          // -1 = kosong, 0..9 angka
uint8_t  pw_index = 0;          // indeks digit yang sedang diedit (0..5)
bool     pw_blinkOn = true;     // status kedip untuk digit aktif
unsigned long pw_nextBlinkMs = 0;
const unsigned long PW_BLINK_PERIOD = 400; // ms, kecepatan kedip

// Alamat EEPROM (ubah kalau kamu sudah pakai alamat ini di tempat lain)
const int EE_ADDR_MAGIC = 900;
const int EE_ADDR_PASS  = 901;
const byte EE_MAGIC     = 0xA5;

// ----- EEPROM untuk threshold -----
const int EE_ADDR_START = 910;  // 2 byte (LSB, MSB)
const int EE_ADDR_SOLID = 912;  // 2 byte (LSB, MSB)

//============EEPROM MODE=============================
const int EE_ADDR_MODE = 914;   // alamat EEPROM untuk mode
uint8_t DISPLAY_MODE = 0;

// ================== EEPROM: LOG COUNTER ==================
const int EEPROM_ADDR_LOGNO = 200;   // 4 byte
uint32_t logCounter = 0;

// ====== VARIABEL STATE UBAH PASSWORD ======
bool  baruMasukUbahPass = true;
int8_t cpw_digits[6];                 // digit yang sedang diedit
uint8_t cpw_index = 0;
bool  cpw_blinkOn = true;
unsigned long cpw_nextBlinkMs = 0;
const unsigned long CPW_BLINK_PERIOD = 400;

//======= ATUR PARAMETER ====================================================================
const uint8_t JUMLAH_MENU_PARAM = 4;
const char m0p[] PROGMEM = "SET WARNING";
const char m1p[] PROGMEM = "SET DANGER";
const char m2p[] PROGMEM = "SET TO DEFAULT";
const char m3p[] PROGMEM = "EXIT TO MENU";
const char* const MENU_TEXTP[] PROGMEM = { m0p, m1p, m2p, m3p };

bool baruMasukMenuP = true;     // mirip baruMasukInputPassword
static uint8_t menuIndexP = 0;  // 0..JUMLAH_MENU-1

// ===== Custom characters (punyamu, dibiarkan) =====
byte image03[8] = {0b01000, 0b11100, 0b11110, 0b11110, 0b00000, 0b00000, 0b00000, 0b00000};
byte image02[8] = {0b00000, 0b00111, 0b01111, 0b11111, 0b11100, 0b11110, 0b11110, 0b11110};
byte image01[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00001};
byte image18[8] = {0b11110, 0b11111, 0b11101, 0b11110, 0b11101, 0b11100, 0b10100, 0b10100};
byte image17[8] = {0b00001, 0b00000, 0b00000, 0b00000, 0b00001, 0b00111, 0b00100, 0b01100};

// ===== Param pembacaan & anti-crosstalk =====
const uint32_t TOUT_US = 30000UL;  // timeout echo ~30 ms (≈ 5 m)
const uint16_t GAP_MS  = 50;      // jeda antar sensor JSN-SR04T waterproof

// ===== Parameter BUZZER/LED =====
const bool LED_ACTIVE_LOW     = true;   // ubah ke false jika modul aktif-HIGH
const bool BUZZER_ACTIVE_LOW     = false;   // ubah ke false jika modul aktif-HIGH
int  START_DIST_CM      = 400;    // mulai bunyi pada 3 m
int  SOLID_DIST_CM      = 50;     // <= 40 cm: nyala terus
const int START_MAX = 500;  // batas atas START
const int SOLID_MIN = 30;   // batas bawah SOLID
const int STEP_CM   = 10;   // kelipatan edit
const unsigned ON_MS          = 50;     // 50 ms hidup
const unsigned OFF_AT_STARTMS = 950;    // 950 ms mati di 3 m (50+950=1000 ms)


enum BlinkPhase { PH_OFF, PH_ON };
BlinkPhase phase = PH_OFF;
unsigned long phaseStartMs = 0;

// inline void setLampu(bool on){
//   digitalWrite(lampu, LED_ACTIVE_LOW ? (on ? LOW : HIGH): (on ? HIGH : LOW)); //apakah led aktif low?? jika ya aktifkan yang (on ? LOW : HIGH) jika tidak yang satunya, apakah on true? jika ya write low jika gk ya high
// }

inline void setBuzzer(bool on){
  digitalWrite(buzzer, BUZZER_ACTIVE_LOW ? (on ? LOW : HIGH): (on ? HIGH : LOW)); //apakah led aktif low?? jika ya aktifkan yang (on ? LOW : HIGH) jika tidak yang satunya, apakah on true? jika ya write low jika gk ya high
}


// ===== State pembacaan (ISR) =====
volatile uint32_t g_ovf = 0;        // Timer1 overflow counter (extend > 65535)
volatile uint8_t  lastPINB, lastPIND, lastPINC;
;

//struct  kumpulan dari beberapa variabel dengan beragam tipe data yang dibungkus dalam satu variabel.

struct EchoChan {
  volatile uint8_t  armed;      // 1: sedang menunggu echo untuk kanal ini
  volatile uint8_t  stage;      // 0: tunggu rising, 1: tunggu falling
  volatile uint16_t t_start;    // TCNT1 saat rising
  volatile uint32_t ovf_start;  // g_ovf saat rising
  volatile uint32_t tof_us;     // hasil durasi us
  volatile uint8_t  ready;      // 1: tof_us valid

  volatile uint16_t t_fire;     // TCNT1 saat tembak
  volatile uint32_t ovf_fire;   // g_ovf saat tembak
  volatile uint32_t blank_us;   // jendela blank awal (us)
};
volatile EchoChan ch[8];  //ch[0]-ch[3] berisi struct echo channel

volatile int16_t distance_cm[8] = {-1, -1, -1, -1, -1, -1, -1, -1};

// ===== Waktu sistem 1 ms (Timer2) =====
volatile uint32_t g_ms = 0;
ISR(TIMER2_COMPA_vect) { g_ms++; } //mode ctc per 1ms

// ===== Timer1 overflow extend =====
ISR(TIMER1_OVF_vect) { g_ovf++; }  //mode ketika timer 1 overflow

// ===== PCINT PORTD (PD0=ECHO1, PD1=ECHO3) =====
ISR(PCINT2_vect) {
  uint8_t now = PIND;
  uint8_t changed = now ^ lastPIND;
  lastPIND = now;

  // PD0 -> ch[4]
  if (ch[4].armed && (changed & (1 << 0))) {
    uint8_t level = (now >> 0) & 1;
    uint16_t t = TCNT1;
    if (level && ch[4].stage == 0) { 
      uint32_t since = (g_ovf - ch[4].ovf_fire)*65536UL + (uint32_t)t - ch[4].t_fire;
      if (since < ch[4].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[4].t_start=t; ch[4].ovf_start=g_ovf; ch[4].stage=1; }
    else if (!level && ch[4].stage == 1) {
      uint32_t ticks = (g_ovf - ch[4].ovf_start)*65536UL + (uint32_t)t - ch[4].t_start;
      ch[4].tof_us = ticks; ch[4].ready=1; ch[4].stage=0; ch[4].armed=0;
    }
  }
  // PD1 -> ch[1]
  if (ch[1].armed && (changed & (1 << 1))) {
    uint8_t level = (now >> 1) & 1;
    uint16_t t = TCNT1;
    if (level && ch[1].stage == 0) { 
      uint32_t since = (g_ovf - ch[1].ovf_fire)*65536UL + (uint32_t)t - ch[1].t_fire;
      if (since < ch[1].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[1].t_start=t; ch[1].ovf_start=g_ovf; ch[1].stage=1; }
    else if (!level && ch[1].stage == 1) {
      uint32_t ticks = (g_ovf - ch[1].ovf_start)*65536UL + (uint32_t)t - ch[1].t_start;
      ch[1].tof_us = ticks; ch[1].ready=1; ch[1].stage=0; ch[1].armed=0;
    }
  }
}



ISR(PCINT1_vect) {
  uint8_t now = PINC;
  uint8_t changed = now ^ lastPINC;
  lastPINC = now;

  // PC1 -> ch[0]
  if (ch[0].armed && (changed & (1 << 1))) {
    uint8_t level = (now >> 1) & 1;
    uint16_t t = TCNT1;
    if (level && ch[0].stage == 0) { 
      uint32_t since = (g_ovf - ch[0].ovf_fire)*65536UL + (uint32_t)t - ch[0].t_fire;
      if (since < ch[0].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[0].t_start=t; ch[0].ovf_start=g_ovf; ch[0].stage=1; }
    else if (!level && ch[0].stage == 1) {
      uint32_t ticks = (g_ovf - ch[0].ovf_start)*65536UL + (uint32_t)t - ch[0].t_start;
      ch[0].tof_us = ticks; ch[0].ready=1; ch[0].stage=0; ch[0].armed=0;
    }
  }


  // PC2 -> ch[2]
  if (ch[2].armed && (changed & (1 << 2))) {
    uint8_t level = (now >> 2) & 1;
    uint16_t t = TCNT1;
    if (level && ch[2].stage == 0) { 
      uint32_t since = (g_ovf - ch[2].ovf_fire)*65536UL + (uint32_t)t - ch[2].t_fire;
      if (since < ch[2].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[2].t_start=t; ch[2].ovf_start=g_ovf; ch[2].stage=1; }
    else if (!level && ch[2].stage == 1) {
      uint32_t ticks = (g_ovf - ch[2].ovf_start)*65536UL + (uint32_t)t - ch[2].t_start;
      ch[2].tof_us = ticks; ch[2].ready=1; ch[2].stage=0; ch[2].armed=0;
    }
  }


  // PC4 -> ch[3]
  if (ch[3].armed && (changed & (1 << 4))) {
    uint8_t level = (now >> 4) & 1;
    uint16_t t = TCNT1;
    if (level && ch[3].stage == 0) { 
      uint32_t since = (g_ovf - ch[3].ovf_fire)*65536UL + (uint32_t)t - ch[3].t_fire;
      if (since < ch[3].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[3].t_start=t; ch[3].ovf_start=g_ovf; ch[3].stage=1; }
    else if (!level && ch[3].stage == 1) {
      uint32_t ticks = (g_ovf - ch[3].ovf_start)*65536UL + (uint32_t)t - ch[3].t_start;
      ch[3].tof_us = ticks; ch[3].ready=1; ch[3].stage=0; ch[3].armed=0;
    }
  }


  // PC5 -> ch[5]
  if (ch[5].armed && (changed & (1 << 5))) {
    uint8_t level = (now >> 5) & 1;
    uint16_t t = TCNT1;
    if (level && ch[5].stage == 0) { 
      uint32_t since = (g_ovf - ch[5].ovf_fire)*65536UL + (uint32_t)t - ch[5].t_fire;
      if (since < ch[5].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[5].t_start=t; ch[5].ovf_start=g_ovf; ch[5].stage=1; }
    else if (!level && ch[5].stage == 1) {
      uint32_t ticks = (g_ovf - ch[5].ovf_start)*65536UL + (uint32_t)t - ch[5].t_start;
      ch[5].tof_us = ticks; ch[5].ready=1; ch[5].stage=0; ch[5].armed=0;
    }
  }


  // PC0 -> ch[6]
  if (ch[6].armed && (changed & (1 << 0))) {
    uint8_t level = (now >> 0) & 1;
    uint16_t t = TCNT1;
    if (level && ch[6].stage == 0) { 
      uint32_t since = (g_ovf - ch[6].ovf_fire)*65536UL + (uint32_t)t - ch[6].t_fire;
      if (since < ch[6].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[6].t_start=t; ch[6].ovf_start=g_ovf; ch[6].stage=1; }
    else if (!level && ch[6].stage == 1) {
      uint32_t ticks = (g_ovf - ch[6].ovf_start)*65536UL + (uint32_t)t - ch[6].t_start;
      ch[6].tof_us = ticks; ch[6].ready=1; ch[6].stage=0; ch[6].armed=0;
    }
  }


  // PC3 -> ch[7]
  if (ch[7].armed && (changed & (1 << 3))) {
    uint8_t level = (now >> 3) & 1;
    uint16_t t = TCNT1;
    if (level && ch[7].stage == 0) { 
      uint32_t since = (g_ovf - ch[7].ovf_fire)*65536UL + (uint32_t)t - ch[7].t_fire;
      if (since < ch[7].blank_us) return;  // terlalu cepat → crosstalk/derau
      ch[7].t_start=t; ch[7].ovf_start=g_ovf; ch[7].stage=1; }
    else if (!level && ch[7].stage == 1) {
      uint32_t ticks = (g_ovf - ch[7].ovf_start)*65536UL + (uint32_t)t - ch[7].t_start;
      ch[7].tof_us = ticks; ch[7].ready=1; ch[7].stage=0; ch[7].armed=0;
    }
  }
}


// ===== Util: pulse TRIG 10–12 µs =====
static inline void trigPulse(uint8_t pin) {
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(30);
  digitalWrite(pin, LOW);
}

static inline void trigPulse2(uint8_t pin) {
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(110);
  digitalWrite(pin, LOW);
}

static inline void trigPulseUs(uint8_t pin, uint8_t width_us){
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(width_us);
  digitalWrite(pin, LOW);
}


// ===== Timers & PCINT init =====
void timer1_begin_1us() {
  // Timer1 normal mode, prescaler /8 => 1 µs per tick @ 8 MHz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TIFR1  = (1 << TOV1);      // clear flags
  TIMSK1 = (1 << TOIE1);     // enable overflow
  TCCR1B |= (1 << CS11);     // start /8
}

void timer2_begin_1ms() {
  // CTC 1 ms: OCR2A=124 @ presc 64  (8e6/(64*(124+1))=1000 Hz)
  TCCR2A = 0; TCCR2B = 0;
  TCNT2  = 0;
  OCR2A  = 124;
  TCCR2A |= (1 << WGM21);    // CTC
  TIMSK2 |= (1 << OCIE2A);   // ISR COMPA
  TCCR2B |= (1 << CS22);     // presc 64 (CS22..20 = 100)
}

void pcint_begin() {
  lastPIND = PIND;
  lastPINB = PINB;
  lastPINC = PINC;

  // PORTD: PD0(bit0), PD1(bit1)
  PCICR  |= (1 << PCIE2);       //aktifkan portd
  PCMSK2 |= (1 << 0) | (1 << 1); //pilih pin yang diawasi PD0,PD1  echo

  // // PORTB: PB2(bit2), PB7(bit7)
  // PCICR  |= (1 << PCIE0);       //aktifkan portb
  // PCMSK0 |= (1 << 2) | (1 << 7);  //PB2,PB7  echo bit 2 dan 7 diberi nilai 1

    // Aktifkan PORTC (PC0/PC1) -> vektor PCINT1_vect
  PCICR  |= (1 << PCIE1);        // enable Pin Change Interrupt untuk PORTC
  PCMSK1 |= (1 << 1) | (1 << 2) | (1 << 4) | (1 << 5); // PC0(bit0), PC1(bit1) - PC5
}

// ===== Scheduler round-robin =====
enum { S_IDLE, S_WAIT_ECHO, S_GAP_WAIT };
uint8_t  s_idx   = 0;         // index sensor aktif sekarang (0..3)
uint8_t  s_state = S_IDLE;    // status state machine scheduler
uint32_t t_fire_us = 0;       // waktu saat TRIG ditembak
uint32_t t_next_ms = 0;       // kapan boleh trigger sensor berikutnya

int pwpw = 0;
int cpwcpw = 0;
int passw = 0;

// ===== Intro non-blocking =============================================================
enum State { 
INTRO, 
UKUR,
RESET,
INPUT_PASSWORD,
VERIFY_PASSWORD,
MENU,
MENU_PASSWORD_CHANGE,
SETTING,
BATAS_ATAS,
BATAS_BAWAH,
DEFAULTS,
MODE,
BACA_CSV,
EXIT
 };
State state = INTRO;
uint32_t introStartMs = 0;

// ===== Setup ==========================================================================
void setup() {
  //Serial.begin(1000000);
  //  pinMode(SD_CS, OUTPUT);
    pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  // CS modul SD
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Mulai SPI pelan & mode standar SD (MODE0, MSB first)
  // SPI.begin();
  // SPI.setDataMode(SPI_MODE0);
  // SPI.setBitOrder(MSBFIRST);
  // // 8 MHz / 16 = ~500 kHz  (bisa dinaikkan nanti jika sudah stabil)
  // SPI.setClockDivider(SPI_CLOCK_DIV16);

  //   // 80 dummy clock dengan CS HIGH (beberapa kartu butuh)
  // digitalWrite(SD_CS, HIGH);
  // for (uint8_t i = 0; i < 10; i++) SPI.transfer(0xFF);

  pinMode(TRIG, OUTPUT); digitalWrite(TRIG, LOW);
  pinMode(TRIG2, OUTPUT); digitalWrite(TRIG2, LOW);
  for (uint8_t i=0; i<N_SENSORS; i++) pinMode(ECHO[i], INPUT);


  pinMode(button1, INPUT_PULLUP);

  // pinMode(lampu, OUTPUT);
  pinMode(buzzer, OUTPUT);
  // setLampu(false);
  setBuzzer(false);

  pass_load();
  thresh_load();
  mode_load();

  lcd.begin(16, 2);

  delay(100);

  // sdReady = sd.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_HZ(100000)));
  sdReady = SD.begin(SD_CS);
  delay(100);
  if (!sdReady) {
    lcd.setCursor(0,0); lcd.print(F("SD FAIL"));
    lcd.setCursor(0,1); lcd.print(F("LOG OFF"));
    delay(800);
    lcd.clear();
  } 
  if(sdReady) {
    lcd.setCursor(0,0); lcd.print(F("SD OK"));
    lcd.setCursor(0,1); lcd.print(F("LOG ON"));
    delay(800);
    lcd.clear();
  }
  
  logCounter = recountLogNoFromFile();
  saveLogCounter();
  loadLogCounter();

  showIntroOnce();
  introStartMs = millis();
  phaseStartMs = millis(); // init timer kedip
  timer1_begin_1us();
  timer2_begin_1ms();
  pcint_begin();
  sei();
}

// ===== Loop utama (non-blocking) =====
void loop() {
  // 1) Scheduler sensor (jalan terus)
  handleButton1();
  buzzer_service();
  
  // 2) Intro 3 detik non-blocking
  //================= STATE INTRO =========================================================================================
  if (state == INTRO) {
    if (millis() - introStartMs >= 3000UL) {
      lcd.clear();
      state = UKUR;
    }
  //====================STATE UKUR======================================================================================
  } else if (state == UKUR){
      scheduler_step();
      unsigned long now = millis();
      // 3) Hitung d_min untuk kontrol LED
      int cm1 = distance_cm[0];
      int cm2 = distance_cm[1];
      int cm3 = distance_cm[2];
      int cm4 = distance_cm[3];
      int cm5 = distance_cm[4];
      int cm6 = distance_cm[5];
      int cm7 = distance_cm[6];
      int cm8 = distance_cm[7];

      if (cm1>500){cm1= -1;}
      if (cm2>500){cm2= -1;}
      if (cm3>500){cm3= -1;}
      if (cm4>500){cm4= -1;}
      if (cm5>500){cm5= -1;}
      if (cm6>500){cm6= -1;}
      if (cm7>500){cm7= -1;}
      if (cm8>500){cm8= -1;}

      // === aturan: jika semua "--", lampu OFF ===
      bool anyInvalid = (cm1<0 && cm3<0 && cm2<0 && cm4<0 && cm5<0 && cm6<0 && cm7<0 && cm8<0);

      int dmin = minValid8(cm1, cm2, cm3, cm4,cm5,cm6,cm7,cm8);
      // updateWarningLamp(dmin, anyInvalid, now);

      // === LOGGER 5 detik saat DANGER ===
      bool isDanger = (dmin >= 0 && dmin <= SOLID_DIST_CM);

      if (isDanger) {
        // Mulai episode baru
        if (!dangerActive) {
          dangerActive   = true;
          dangerCounter  = 0;
          dangerNextDueMs = millis() + LOG_PERIOD_MS;  // tunggu 5 detik pertama
        } else {
          // Cek jatuh tempo pencatatan (tiap 5 detik)
          unsigned long nowMs = millis();
          if ((long)(nowMs - dangerNextDueMs) >= 0) {
            dangerCounter++;  // 1,2,3,...
            uint16_t waktuTotal = dangerCounter * (LOG_PERIOD_MS / 1000); // 5,10,15,...

            // Jarak yang dicatat = dmin saat momen pencatatan
            if (writeDangerLog((uint16_t)dmin, dangerCounter, waktuTotal)) {
              logCounter++;
              saveLogCounter();
              lcd.setCursor(0,0);
              lcd.print(F("LOG #"));
            }
            // Jadwalkan tenggat berikutnya (anti-drift, absolut step)
            dangerNextDueMs += LOG_PERIOD_MS;
          }
        }
      } else {
        // Episode selesai / dibatalkan: reset
        dangerActive   = false;
        dangerCounter  = 0;
        dangerNextDueMs = 0;
        }


      // 5) Refresh LCD (throttle biar halus, 100 ms)
      static uint32_t lastLcd = 0;
      if (now - lastLcd >= 60) {
      lastLcd = now;

      if (DISPLAY_MODE == 0){
        printCell(0, 0, F(" "), cm1);
        printCell(4, 0, F(" "), cm2);
        printCell(8, 0, F(" "), cm3);
        printCell(12, 0, F(" "), cm4);
        printCell(0, 1, F(" "), cm5);
        printCell(4, 1, F(" "), cm6);
        printCell(8, 1, F(" "), cm7);
        printCell(12, 1, F(" "), cm8);
      } else {
        // ===== MODE SIMPLE =====
        // Status KONDISI
        const __FlashStringHelper* status;
        if (dmin < 0) {
        status = F("   BLIND SPOT   ");
        } else if (dmin <= SOLID_DIST_CM) {
          status = F("   DANGER !!!   ");
        } else if (dmin <= START_DIST_CM && dmin>=SOLID_DIST_CM) {
          status = F("    WARNING!    ");
        } else {
          status = F("      SAFE      ");
        } 
        lcd.setCursor(0,0); //lcd.print(F("KONDISI: "));
        lcd.print(status);  // pastikan terpad

        // Jarak minimal (pakai meter 1 desimal, atau cm kalau kamu mau)
        //lcd.setCursor(0,1); lcd.print(F("Jarak: "));
        lcd.setCursor(0,1);
        if (dmin < 0) {
          lcd.print(F("      ----      "));
        } else {
        // cetak meter 1 desimal tanpa sprintf
        //uint16_t cm = (uint16_t)dmin;
        // uint16_t whole = cm / 100;
        // uint8_t  tenth = (cm % 100) / 10;
        // lcd.print(whole); lcd.print('.');
        // lcd.print(tenth);
        // lcd.print(F(" m   "));
        // printCell(1, 1, F("Jarak : "), dmin);
        // lcd.print(F("CM"));
        lcd.print(F("Jarak: "));
        if (dmin < 10)      { lcd.print(dmin); lcd.print(F("   cm  ")); }
        else if (dmin < 100){ lcd.print(dmin); lcd.print(F("  cm  "));  }
        else                { lcd.print(dmin); lcd.print(F(" cm  "));   }

        }

      }

    }
    if (newLongPress1){
      baruMasukInputPassword = true;
      state = INPUT_PASSWORD;
      newLongPress1 = false;
    }

    // else if (newLongPress2 && newLongPress3 == true){
    //   state = RESET;
    //   newLongPress2 = false;
    //   newLongPress3 = false;
      
    // }

  }else if (state == RESET){
      unsigned long now = millis();
      static uint32_t resetHoldUntil = 0;      // tahan layar "RESET" 1.5s

      for (uint8_t i=0;i<6;i++) PASS_EXPECTED[i] = '0';
        PASS_EXPECTED[6] = '\0';
        pass_save();
        if (resetHoldUntil == 0){
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(F("  RESET PASS   "));
        lcd.setCursor(0,1); lcd.print(F("    SUCCESS     "));
        resetHoldUntil = now + 1500;}

        if ((long)(now - resetHoldUntil) >= 0) {
          resetHoldUntil = 0;
          pass_load();
          lcd.clear(); // setelah selesai, layar akan diisi ulang oleh refresh biasa
          passw = 0;
          state = UKUR;
        }
//====================STATE INPUT PASSWORD=======================================
  } else if (state == INPUT_PASSWORD){
      if (baruMasukInputPassword) {
        lastInteractionTime = millis();      // set waktu awal
        baruMasukInputPassword = false;      // agar tidak diulang tiap loop
        lcd.clear();
        pw_reset();
        pw_draw(true);
      }
      unsigned long now = millis();

      // ===== KEDIP NON-BLOCKING =====
      if (now >= pw_nextBlinkMs) {
      pw_blinkOn = !pw_blinkOn;
      pw_nextBlinkMs = now + PW_BLINK_PERIOD;
      pw_draw(false); // redraw baris digit saja
      }

      //ketika button up ditekan
      if (newShortPress1){
        // kalau masih kosong, mulai dari 0
          pw_digits[pw_index] = (pw_digits[pw_index] + 1) % 10;
          pw_blinkOn = true; 
          pw_nextBlinkMs = now + PW_BLINK_PERIOD;
          pw_draw(false);
          lastInteractionTime = now;
          newShortPress1 = false;
      
      }else if (newLongPress1){
        if (pw_digits[pw_index] < 0) pw_digits[pw_index] = 0; // auto-commit
        pw_index = (pw_index + 1) % 6;        // selalu wrap
        pwpw++;
        pw_blinkOn = true;                    // langsung tampil ON
        pw_nextBlinkMs = now + PW_BLINK_PERIOD;
        pw_draw(false);                       // redraw tiap pindah
        lastInteractionTime = now;
        newLongPress1 = false;
      
      }if (pwpw > 5){
        state = VERIFY_PASSWORD;
        pwpw = 0;
      }
        // idle timeout balik ke layar utama
      if (now - lastInteractionTime >= 8000) {
        lcd.clear();
        state = UKUR;
      }
      if (passw > 5){
        state = RESET;
      }
  //===========STATE VERIFY PASSWORD=========================
  }else if (state == VERIFY_PASSWORD) {
  static uint32_t holdUntil = 0;  // one-shot penahan pesan 1 detik

  // cek sudah lengkap 6 digit?
  //bool complete = true;
  char entered[7];
  for (uint8_t i=0;i<6;i++){
    // if (pw_digits[i] < 0){ complete = false; break; }
    entered[i] = char('0' + pw_digits[i]);
  }
  entered[6] = '\0';

    if (strcmp(entered, PASS_EXPECTED) == 0) {
      if (holdUntil == 0) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(" ACCESS GRANTED ");
      lcd.setCursor(0,1); lcd.print("   WELCOME:)    ");
      if (holdUntil == 0) holdUntil = millis() + 2000;}
      if ((long)(millis() - holdUntil) >= 0) {
        holdUntil = 0;
        lcd.clear();
        baruMasukMenu = true;
        passw = 0;
        state = MENU;          // lanjut
      }
    } else {
      if (holdUntil == 0) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("   WRONG PASS   ");
      lcd.setCursor(0,1); lcd.print("    RETRY!!!    ");
      if (holdUntil == 0) holdUntil = millis() + 2000;}
      if ((long)(millis() - holdUntil) >= 0) {
        holdUntil = 0;
        baruMasukInputPassword = true;
        passw++;
        state = INPUT_PASSWORD;
      }
    }
  //}
//==========MENU====================================================================
} else if (state == MENU) {
  unsigned long now = millis();

  if (baruMasukMenu) {
    menuIndex = 0;                 // mulai dari item pertama
    menu_draw(true);
    baruMasukMenu = false;
    lastInteractionTime = now;
  }
  // tombol: next/prev/select/back
  if (newShortPress1) {            // NEXT (button1 short)
    menuIndex = (menuIndex + 1) % JUMLAH_MENU;
    menu_draw(false);
    newShortPress1 = false;
    lastInteractionTime = now;
  }

  if (newLongPress1) {             // SELECT (button1 long)
    newLongPress1 = false;
    lastInteractionTime = now;

    if (menuIndex == 0) {
      lcd.clear();
      baruMasukUbahPass = true;
      state = MENU_PASSWORD_CHANGE;
    } else if (menuIndex == 1) {
      lcd.clear();
      baruMasukMenuP = true;
      state = SETTING;   // atau MENU_CT_SETUP jika sudah ada
    } else if (menuIndex == 2) {
      lcd.clear();
      state = MODE;               // “Keluar” kembali ke layar ukur
    } else if (menuIndex == 3) {
      lcd.clear();
      state = UKUR;               // “Keluar” kembali ke layar ukur
    }
  }

  // idle timeout balik ke layar utama
  if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    lcd.clear();
    state = UKUR;
  }
//=======================UBAH PASSWORD==============================================================
}else if (state == MENU_PASSWORD_CHANGE) {
  unsigned long now = millis();
  static bool     cpwHold = false; //
  static uint32_t cpwHoldUntil = 0; //

  if (baruMasukUbahPass) {
    lcd.clear();
    cpw_reset_from_current();
    cpw_draw(true);
    baruMasukUbahPass = false;
    cpwHold = false; //
    lastInteractionTime = now;
  }

  // saat hold aktif, tunggu 1 detik lalu balik ke MENU
  if (cpwHold) {
    if ((long)(millis() - cpwHoldUntil) >= 0) {
      cpwHold = false;
      lcd.clear();
      baruMasukMenu = true;
      state = MENU;
      baruMasukUbahPass = true;
    }
    return;   // penting: cegah cpw_draw() menimpa tulisan feedback
  }

  // Blink non-blocking
  if (now >= cpw_nextBlinkMs) {
    cpw_blinkOn = !cpw_blinkOn;
    cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
  }

  // Kontrol tombol (samakan mapping dengan INPUT_PASSWORD):
  // button2 short: +1 digit
  if (newShortPress1) {
    if (cpw_digits[cpw_index] < 0) cpw_digits[cpw_index] = 0;
    cpw_digits[cpw_index] = (cpw_digits[cpw_index] + 1) % 10;
    cpw_blinkOn = true; cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
    newShortPress1 = false;
    lastInteractionTime = now;
  }

  // button1 short: pindah kolom
  if (newLongPress1) {
    if (cpw_digits[cpw_index] < 0) cpw_digits[cpw_index] = 0; // auto-commit
    cpw_index = (cpw_index + 1) % 6;
    cpwcpw++;
    cpw_blinkOn = true; cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
    newLongPress1 = false;
    lastInteractionTime = now;
  }
  // button1 long: SIMPAN
  if (cpwcpw > 5) {
    cpwcpw = 0;
    bool same = true;
    for (uint8_t i=0; i<6; i++) {
      if (PASS_EXPECTED[i] != char('0' + cpw_digits[i])) { same = false; break; }
    }
       lcd.clear();
      if (!same) {
      // commit ke RAM + EEPROM
      for (uint8_t i=0;i<6;i++) PASS_EXPECTED[i] = char('0' + cpw_digits[i]);
      PASS_EXPECTED[6] = '\0';
      pass_save();

      //if (holdUntil == 0) {
       // lcd.clear();
        lcd.setCursor(0,0); lcd.print(" PASSWORD SAVED ");
        lcd.setCursor(0,1); lcd.print("  SUCCESSFULLY  ");
       // holdUntil = millis() + 1000;
      //}
      // if ((long)(millis() - holdUntil) >= 0) {
      //   lcd.clear();
      //   baruMasukMenu = true;
      //   state = MENU;                 // kembali ke menu
      //   baruMasukUbahPass = true;     // biar redraw saat masuk lagi
      //   holdUntil = 0;
      // }
      //  lastInteractionTime = now;
      } else {
        // Nilai sama: tetap beri feedback, tapi TANPA menulis EEPROM
          // if (holdUntil == 0) {
            // lcd.clear();
            lcd.setCursor(0,0); lcd.print("NOTHING  CHANGED");
            //lcd.setCursor(0,1); lcd.print("   PERUBAHAN    ");
            // holdUntil = millis() + 1000;
          // }
          // if ((long)(millis() - holdUntil) >= 0) {
          //   lcd.clear();
          //   baruMasukMenu = true;
          //   state = MENU;                 // kembali ke menu
          //   baruMasukUbahPass = true;     // biar redraw saat masuk lagi
          //   holdUntil = 0;
          // }
        } // Aktifkan hold 1 detik -> otomatis kembali ke MENU
    cpwHold = true;
    cpwHoldUntil = millis() + 1000;

    lastInteractionTime = now;
  }

  // --- Idle timeout ---
  if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    state = UKUR;
  }
  //================SETTING=========================================================
} else if (state == SETTING){
    unsigned long now = millis();

  if (baruMasukMenuP) {
    menuIndexP = 0;                 // mulai dari item pertama
    menu_draw_param(true);
    baruMasukMenuP = false;
    lastInteractionTime = now;
  }
  // tombol: next/prev/select/back
   if (newShortPress1) {            // NEXT (button1 short)
    menuIndexP = (menuIndexP + 1) % JUMLAH_MENU_PARAM;
    menu_draw_param(false);
    newShortPress1 = false;
    lastInteractionTime = now;
  }

  if (newLongPress1) {             // SELECT (button1 long)
    newLongPress1 = false;
    lastInteractionTime = now;

    if (menuIndexP == 0) {
      lcd.clear();
      //baruMasukUbahPass = true;
      state = BATAS_ATAS;
    } else if (menuIndexP == 1) {
      lcd.clear();
      state = BATAS_BAWAH;   // atau MENU_CT_SETUP jika sudah ada
    } else if(menuIndexP == 2){
      lcd.clear();
      state = DEFAULTS;
    }else if (menuIndexP == 3) {
      lcd.clear();
      baruMasukMenu = true;
      state = MENU;               // “Keluar” kembali ke setting
    }
  }

  // idle timeout balik ke layar utama
  if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    lcd.clear();
    state = UKUR;
  }

//============================BATAS ATAS===========================================
} else if (state == BATAS_ATAS){
  static bool first = true;
  static uint32_t holdUntil = 0;  // untuk pesan "tersimpan"
  unsigned long now = millis();

  // Jika sedang menahan pesan, jangan gambar/terima tombol
  if (holdUntil) {
    if ((long)(millis() - holdUntil) >= 0) {
      holdUntil = 0;
      first = true;
      state = SETTING;
      baruMasukMenuP = true;
      lcd.clear();
    }
    return;
  }

  if (first) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Set WARNING");
    lcd.setCursor(0,1); lcd.print("Val: ");
    first = false;
  }

  // Tampilkan nilai
  lcdPrintCmAsMetersAt(5, 1, START_DIST_CM);

  // +10 (button2 short), -10 (button2 long)
  if (newShortPress1) {
    START_DIST_CM += STEP_CM;
    if (START_DIST_CM > START_MAX) START_DIST_CM = SOLID_DIST_CM;
    if (START_DIST_CM < SOLID_DIST_CM) START_DIST_CM = SOLID_DIST_CM; // jaga hubungan
    newShortPress1 = false;
    lastInteractionTime = now;
  }
  // if (newShortPress3) {
  //   START_DIST_CM -= STEP_CM;
  //   if (START_DIST_CM < SOLID_DIST_CM) START_DIST_CM = SOLID_DIST_CM; // tidak boleh < SOLID
  //   newShortPress3 = false;
  //   lastInteractionTime = now;
  // }

  // Simpan (button1 long) -> tahan pesan 1 detik
  if (newLongPress1) {
    newLongPress1 = false;
    thresh_save();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(" WARNING  SAVED ");
    lcd.setCursor(0,1); lcd.print("  SUCCESSFULLY  ");
    holdUntil = millis() + 1500;
    lastInteractionTime = now;
  }

  // // Back cepat (opsional) dengan button3 short
  // if (newShortPress3) {
  //   newShortPress3 = false;
  //   state = MENU;
  //   baruMasukMenu = true;
  //   lcd.clear();
  // }

  // Idle timeout
  if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    state = UKUR;
    lcd.clear();
    first = true;
  }

//============================BATAS BAWAH=============================================================================
}else if (state == BATAS_BAWAH){
  static bool first = true;
  static uint32_t holdUntil = 0;
  unsigned long now = millis();

  if (holdUntil) {
    if ((long)(millis() - holdUntil) >= 0) {
      holdUntil = 0;
      first = true;
      state = SETTING;
      baruMasukMenuP = true;
      lcd.clear();
    }
    return;
  }

  if (first) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Set DANGER  ");
    lcd.setCursor(0,1); lcd.print("Val:            ");
    first = false;
  }

  lcdPrintCmAsMetersAt(5, 1, SOLID_DIST_CM);

  // +10 (button2 short), -10 (button2 long)
  if (newShortPress1) {
    SOLID_DIST_CM += STEP_CM;
    if (SOLID_DIST_CM > START_DIST_CM) SOLID_DIST_CM = SOLID_MIN; // tak boleh > START
    newShortPress1 = false;
    lastInteractionTime = now;
  }
  // if (newShortPress3) {
  //   SOLID_DIST_CM -= STEP_CM;
  //   if (SOLID_DIST_CM < SOLID_MIN) SOLID_DIST_CM = SOLID_MIN;         // batas bawah 25
  //   newShortPress3 = false;
  //   lastInteractionTime = now;
  // }

  // Simpan (button1 long)
  if (newLongPress1) {
    newLongPress1 = false;
    thresh_save();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("  DANGER SAVED  ");
    lcd.setCursor(0,1); lcd.print("  SUCCESSFULLY  ");
    holdUntil = millis() + 1500;
    lastInteractionTime = now;
  }

  // if (newShortPress3) {
  //   newShortPress3 = false;
  //   state = MENU;
  //   baruMasukMenu = true;
  //   lcd.clear();
  // }

  if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    state = UKUR;
    lcd.clear();
    first = true;
  }
//=============================DEFAULT========================================================
}else if (state == DEFAULTS){
    unsigned long now = millis();
    static uint32_t resetHoldUntil = 0;      // tahan layar "RESET" 1.5s
    int warning = 400;
    int danger = 50;
    eepromWriteU16(EE_ADDR_START, (uint16_t)warning);
    eepromWriteU16(EE_ADDR_SOLID, (uint16_t)danger);
    START_DIST_CM = warning;
    SOLID_DIST_CM = danger;
    
    if (resetHoldUntil == 0){
      lcd.clear();
      lcd.setCursor(0,0); lcd.print(F(" SET TO DEFAULT "));
      lcd.setCursor(0,1); lcd.print(F("    SUCCESS     "));
      resetHoldUntil = now + 1500;}

      if ((long)(now - resetHoldUntil) >= 0) {
        resetHoldUntil = 0;
        lcd.clear(); // setelah selesai, layar akan diisi ulang oleh refresh biasa
        state = SETTING;
        baruMasukMenuP = true;
        }
}else if (state == MODE){
    static bool first = true;
    static uint32_t holdUntil = 0;
    unsigned long now = millis();

      if (holdUntil) {
    if ((long)(now - holdUntil) >= 0) {
      holdUntil = 0;
      lcd.clear();
      state = MENU;
      baruMasukMenu = true;
      first = true;
    }
    return;
  }   
    if (first) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(F("DISPLAY MODE  "));
    first = false;
  }
    // tampilkan pilihan saat ini
  lcd.setCursor(0,1); lcd.print(F("> "));
  lcd.setCursor(2,1);
  if (DISPLAY_MODE == 0) {
    lcd.print(F("NORMAL          "));
  } else {
    lcd.print(F("SIMPLE         "));
  }

  if (newShortPress1) {
    newShortPress1 = false;
    DISPLAY_MODE = (DISPLAY_MODE + 1) % 2;
    lastInteractionTime = now;
  // }else if (newShortPress3) {
  //   newShortPress2 = false;
  //   DISPLAY_MODE = (DISPLAY_MODE + 1) % 2;
  //   lastInteractionTime = now;
  }else if (newLongPress1) {
    newLongPress1 = false;
    mode_save();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(F("   SAVED MODE   "));
    lcd.setCursor(0,1);
    if (DISPLAY_MODE == 0) lcd.print(F("     NORMAL     "));
    else                   lcd.print(F("     SIMPLE     "));
    holdUntil = now + 1000;
    lastInteractionTime = now;
  }
   if (now - lastInteractionTime >= IDLE_TIMEOUT) {
    state = UKUR;
    lcd.clear();
    first = true;
  }
}






}


//============================================END LOOP=======================================================================================================================



//=================GAMBAR INTRO CUKURUKUK DISPLAY==========================
void showIntroOnce() {
  lcd.clear();
  lcd.createChar(0, image03);
  lcd.createChar(1, image02);
  lcd.createChar(2, image01);
  lcd.createChar(3, image18);
  lcd.createChar(4, image17);

  lcd.setCursor(9, 0); lcd.write((uint8_t)0);
  lcd.setCursor(8, 0); lcd.write((uint8_t)1);
  lcd.setCursor(7, 0); lcd.write((uint8_t)2);
  lcd.setCursor(8, 1); lcd.write((uint8_t)3);
  lcd.setCursor(7, 1); lcd.write((uint8_t)4);
}

//=================DISPLAY SENSOR ======================
void printCell(int col, int row, const __FlashStringHelper* label, int val) {
  lcd.setCursor(col, row);
  lcd.print(label);
  if (val < 0) {
    lcd.print(F("--  "));
  } else {
    // pad 3–4 kolom supaya tidak “bekas” digit lama
    if (val < 10)      { lcd.print(val); lcd.print(F("   ")); }
    else if (val < 100){ lcd.print(val); lcd.print(F("  "));  }
    else               { lcd.print(val); lcd.print(F(" "));   }
  }
}

//============== WARNING BUZZER BLINK=============================
void updateWarningLamp(int dmin, bool anyInvalid, unsigned long now){
  // 1) Jika semua sensor "--" => lampu/buzzer OFF
  if (anyInvalid) { /*setLampu(false);*/ setBuzzer(false); phase = PH_OFF; phaseStartMs = now; return; }

  //2) Jika semua > 300 cm atau tak ada baca valid => OFF
  if (dmin < 0 || dmin > START_DIST_CM) {
    /*setLampu(false);*/ setBuzzer(false); phase = PH_OFF; phaseStartMs = now; return;
  }

  // 3) Jika <= 40 cm => ON solid
  if (dmin <= SOLID_DIST_CM) { /*setLampu(true);*/ setBuzzer(true); return; }

  // 4) 40..300 cm => kedip: ON 50ms, OFF menurun dari 950ms -> 0ms
  unsigned long offTarget = offMsFromDistance(dmin);

  if (phase == PH_ON) {
    if (now - phaseStartMs >= ON_MS) {
      //setLampu(false);
      setBuzzer(false);
      phase = PH_OFF;
      phaseStartMs = now;
    }
  } else { // PH_OFF
    if (now - phaseStartMs >= offTarget) {
      //setLampu(true);
      setBuzzer(true);
      phase = PH_ON;
      phaseStartMs = now;
    }
  }
}

//======= HITUNG LAMA OFF BERDASAR JARAK (LINIER 300->40 cm : 950->0 ms) ===============================
static inline unsigned long offMsFromDistance(int dcm){
  if (START_DIST_CM <= SOLID_DIST_CM) return 0;  // treat as solid ON
  if (dcm >= START_DIST_CM) return OFF_AT_STARTMS; //jarak aktual lebih besar dari 300cm maka off 950ms
  if (dcm <= SOLID_DIST_CM) return 0; //jika jarak lebih kecil dari 40cm maka full on atau off=0
  // linear interpolation: off = (d-40)/(300-40) * 950
  return (unsigned long)((long)(dcm - SOLID_DIST_CM) * OFF_AT_STARTMS / (START_DIST_CM - SOLID_DIST_CM));
}

// ===== MENCARI ANGKA MINIMAL/ PALING KECIL VALID ======================================
static inline int minValid8(int a, int b, int c, int d,int e,int f,int g,int h) {
  const int INF = 10000;
  int m = INF;
  if (a > 0 && a < 500) m = min(m, a);
  if (b > 0 && b < 500) m = min(m, b);
  if (c > 0 && c < 500) m = min(m, c);
  if (d > 0 && d < 500) m = min(m, d);
  if (e > 0 && e < 500) m = min(m, e);
  if (f > 0 && f < 500) m = min(m, f);
  if (g > 0 && g < 500) m = min(m, g);
  if (h > 0 && h < 500) m = min(m, h);
  return (m == INF) ? -1 : m;
}

//==========SCHEDULER UNTUK PENGGANTI PULSEIN=====================================
void scheduler_step() {
  switch (s_state) {
    case S_IDLE:
      if ((int32_t)(g_ms - t_next_ms) >= 0) {
        ch[s_idx].armed = 1;
        ch[s_idx].stage = 0;
        ch[s_idx].ready = 0;

        // siapkan blanking per sensor
        ch[s_idx].blank_us = (uint32_t)BLANK_NEAR_CM * 58UL;

        // catat waktu tembak (untuk hitung blanking di ISR)
        ch[s_idx].ovf_fire = g_ovf;
        ch[s_idx].t_fire   = TCNT1;

        // pilih TRIG pin & lebar pulsa
        uint8_t trigPin = (TRIG_SEL[s_idx] == 0) ? TRIG : TRIG2;
        trigPulseUs(trigPin, PULSE_US[s_idx]);

        t_fire_us = micros();
        s_state = S_WAIT_ECHO;
      }
    break;


    case S_WAIT_ECHO:
      if (ch[s_idx].ready) {
        int16_t raw = (int16_t)(ch[s_idx].tof_us / 58UL);
        distance_cm[s_idx] = (raw>0 && raw<=500) ? raw : -1;
        t_next_ms = g_ms + GAP_MS_PER[s_idx];
        s_state = S_GAP_WAIT;
      } else if ((uint32_t)(micros() - t_fire_us) > TOUT_US) {
          ch[s_idx].armed = 0; ch[s_idx].stage = 0;
          distance_cm[s_idx] = -1;
          t_next_ms = g_ms + GAP_MS_PER[s_idx];
          s_state = S_GAP_WAIT;
        }
      break;

    case S_GAP_WAIT:
      if ((int32_t)(g_ms - t_next_ms) >= 0) {
        s_idx = (s_idx + 1) % N_SENSORS;   // <<— dulu %8, sekarang ikuti jumlah aktif
        s_state = S_IDLE;
      }
      break;
  }
}

// ====== UTIL INPUT PW ==================================================================
void pw_reset(){
  for (uint8_t i=0;i<6;i++) pw_digits[i] = 0;
  pw_index = 0;
  pw_blinkOn = true;
  pw_nextBlinkMs = millis() + PW_BLINK_PERIOD;
}

void pw_draw(bool redrawHeader){
  if (redrawHeader){
    lcd.setCursor(0,0);
    lcd.print("   INPUT PASS");    // padding spasi biar bersih
  }
  const uint8_t startCol = 5;         // center: 16-6 = 10, /2 = 5
  for (uint8_t i=0;i<6;i++){
    char ch;
    int8_t d = pw_digits[i];
    bool isCursor = (i == pw_index);
    if (isCursor && !pw_blinkOn) ch = ' ';        // “hilang” saat off -> efek kedip
    else                         ch = (d < 0 ? '-' : char('0' + d)); // placeholder '-'
    lcd.setCursor(startCol + i, 1);
    lcd.write(ch);
  }
}

//===============MENU =====================================================

// helper print string dari PROGMEM ke LCD, auto-pad 16 kolom
void lcdPrintPadded_P(uint8_t col, uint8_t row, const char* p) {
  char buf[17];                           // max 16 char
  strncpy_P(buf, p, 16); buf[16] = '\0';
  lcd.setCursor(col, row); lcd.print(buf);
  // pad sisa agar bekas huruf lama hilang
  uint8_t len = strlen(buf);
  for (uint8_t i=len; i<16-col; i++) lcd.print(' ');
}

// gambar layar menu (judul + item aktif dgn panah)
void menu_draw(bool clearAll) {
  if (clearAll) lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MENU ");                 // "Menu 1/3"
  lcd.print(menuIndex+1);
  lcd.print("/");
  lcd.print(JUMLAH_MENU);
  uint8_t pad = 16 - (5 + (menuIndex+1>=10) + (JUMLAH_MENU>=10?2:1));
  while (pad--) lcd.print(' ');        // bereskan sisa kolom

  lcd.setCursor(0,1); lcd.print("> ");
  const char* p = (const char*)pgm_read_word(&MENU_TEXT[menuIndex]);
  lcdPrintPadded_P(2,1,p);
}

// gambar layar menu (judul + item aktif dgn panah)
void menu_draw_param(bool clearAll) {
  if (clearAll) lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SET PARAM ");                 // "Menu 1/3"
  lcd.print(menuIndexP+1);
  lcd.print("/");
  lcd.print(JUMLAH_MENU_PARAM);
  uint8_t pad = 16 - (5 + (menuIndexP+1>=10) + (JUMLAH_MENU_PARAM>=10?2:1));
  while (pad--) lcd.print(' ');        // bereskan sisa kolom

  lcd.setCursor(0,1); lcd.print("> ");
  const char* p = (const char*)pgm_read_word(&MENU_TEXTP[menuIndexP]);
  lcdPrintPadded_P(2,1,p);
}


//============LOAD SAVE PASSSWORD EEPROM=====================================================
void pass_load() {
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC) {
    // Inisialisasi pertama kali
    for (uint8_t i=0;i<6;i++) EEPROM.update(EE_ADDR_PASS+i, PASS_EXPECTED[i]);
    EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
  }
  for (uint8_t i=0;i<6;i++) PASS_EXPECTED[i] = EEPROM.read(EE_ADDR_PASS+i);
  PASS_EXPECTED[6] = '\0';
}

void pass_save() {
  for (uint8_t i=0;i<6;i++) {
    char c = PASS_EXPECTED[i];
    if (c < '0' || c > '9') c = '0';
    EEPROM.update(EE_ADDR_PASS+i, c);
  }
  EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
}

//==============FUNGSI STATE UBAH PASSWORD===========================================
void cpw_reset_from_current() {
  for (uint8_t i=0;i<6;i++) cpw_digits[i] = PASS_EXPECTED[i] - '0';
  cpw_index = 0;
  cpw_blinkOn = true;
  cpw_nextBlinkMs = millis() + CPW_BLINK_PERIOD;
}

void cpw_draw(bool redrawHeader) {
  if (redrawHeader) {
    lcd.setCursor(0,0);
    lcd.print("CHANGE  PASSWORD");   // 16 kolom, pad spasi
  }
  const uint8_t startCol = 5;        // sama seperti input
  for (uint8_t i=0;i<6;i++){
    char ch;
    int8_t d = cpw_digits[i];
    bool isCursor = (i == cpw_index);
    if (isCursor && !cpw_blinkOn) ch = ' ';
    else                          ch = (d < 0 ? '-' : char('0' + d));
    lcd.setCursor(startCol + i, 1);
    lcd.write(ch);
  }
}

//==================EEPROM THRESHOLD PARAMETER SETTING==============================
// magic byte kamu sudah ada (EE_ADDR_MAGIC/EE_MAGIC). Panggil pass_load() lebih dulu.
static inline void eepromWriteU16(int addr, uint16_t v){
  EEPROM.update(addr,     (uint8_t)(v & 0xFF));
  EEPROM.update(addr + 1, (uint8_t)(v >> 8));
}
static inline uint16_t eepromReadU16(int addr){
  uint16_t lo = EEPROM.read(addr);
  uint16_t hi = EEPROM.read(addr+1);
  return (hi << 8) | lo;
}

static inline void clamp_start() {
  if (START_DIST_CM > START_MAX) START_DIST_CM = START_MAX;
  if (START_DIST_CM < SOLID_DIST_CM) START_DIST_CM = SOLID_DIST_CM;
}
static inline void clamp_solid() {
  if (SOLID_DIST_CM < SOLID_MIN) SOLID_DIST_CM = SOLID_MIN;
  if (SOLID_DIST_CM > START_DIST_CM) SOLID_DIST_CM = START_DIST_CM;
}

void thresh_load() {
  int s = (int)eepromReadU16(EE_ADDR_START); // bisa jadi -1 kalau EEPROM = 0xFFFF
  int t = (int)eepromReadU16(EE_ADDR_SOLID);

  // valid kalau: 30 <= t <= s <= 500
  bool sane = (t >= SOLID_MIN) && (s <= START_MAX) && (t <= s);

  if (!sane) {
    // fallback default saat pertama kali / data rusak
    s = 400;
    t = 50;
    eepromWriteU16(EE_ADDR_START, (uint16_t)s);
    eepromWriteU16(EE_ADDR_SOLID, (uint16_t)t);
  }

  START_DIST_CM = s;
  SOLID_DIST_CM = t;
} 

void thresh_save() {
  clamp_start();
  clamp_solid();
  eepromWriteU16(EE_ADDR_START, (uint16_t)START_DIST_CM);
  eepromWriteU16(EE_ADDR_SOLID, (uint16_t)SOLID_DIST_CM);
}

//================EEPROM MODE========================================================
// load & save
void mode_load() {
  uint8_t v = EEPROM.read(EE_ADDR_MODE);
  if (v > 1) v = 0;              // fallback
  DISPLAY_MODE = v;
}
void mode_save() {
  EEPROM.update(EE_ADDR_MODE, DISPLAY_MODE);
}

// ====== EEPROM LOG COUNTER ==============================================
void loadLogCounter() {
  EEPROM.get(EEPROM_ADDR_LOGNO, logCounter);
  if (logCounter == 0xFFFFFFFFUL || logCounter > 100000000UL) logCounter = 0;
}
void saveLogCounter() {
  EEPROM.put(EEPROM_ADDR_LOGNO, logCounter);
}


// ===================== Flag permintaan bunyi dari handler ========================================================================================
volatile bool btnBeepRequest = false;
// One-shot 50 ms tanpa delay

static inline void buzzer_service(){
  static bool on = false;
  static unsigned long offAt = 0;

  // trigger baru?
  if (btnBeepRequest){
    btnBeepRequest = false;  // konsumsi
    //setLampu(true);
    setBuzzer(true);
    on = true;
    offAt = millis() + 50;   // 50 ms
  }

  // matikan setelah 50 ms
  if (on && (long)(millis() - offAt) >= 0){
    //setLampu(false);
    setBuzzer(false);
    on = false;
  }
}

//==================KONVERSI CM ke M==========================================================================
// Tampil cm sebagai meter dengan 1 digit desimal, mis. 50 -> "0.5 m"
static inline void lcdPrintCmAsMetersAt(uint8_t col, uint8_t row, uint16_t cm){
  uint16_t whole = cm / 100;       // bagian meter
  uint8_t  tenth = (cm % 100) / 10; // 0..9 (0.1 m)
  lcd.setCursor(col, row);
  lcd.print(whole);
  lcd.print('.');
  lcd.print(tenth);
  lcd.print(F(" M   "));           // pad agar bekas angka lama hilang
}

//=====================Write Log=========================================================================================
void ensureLogHeader(File &f) {
  if (f.size() == 0) {
    f.println(F("sep=,")); // Excel pakai koma
    f.println(F("No,Jarak,Counter,Waktu Total"));
    logCounter = 0; saveLogCounter();   // mulai dari 0/1 lagi
  }
}

bool writeDangerLog(uint16_t jarakCm, uint16_t counter, uint16_t waktuTotalSec){
  if (!sdReady) return false;

  File f = SD.open(LOG_PATH, FILE_WRITE);
  //if (!f.open(LOG_PATH, O_WRITE | O_CREAT | O_APPEND)) return false;

  ensureLogHeader(f);

  uint32_t no = logCounter + 1;

  f.print(no);                 f.print(',');
  f.print(jarakCm);            f.print(F(" cm,"));   // sesuai contohmu “30 cm”
  f.print(counter);            f.print(',');
  f.print(waktuTotalSec);      f.println(F(" detik"));

  //f.sync();
  f.close();
  return true;
}

uint32_t recountLogNoFromFile() {
  File f = SD.open(LOG_PATH, FILE_READ);
  //if (!f.open(LOG_PATH, O_READ)) return 0;
  char line[120]; //Buffer untuk menampung satu baris teks yang dibaca dari file.
  uint8_t len=0; 
  uint32_t lastNo=0; //Menyimpan nomor urut terakhir yang berhasil dibaca.
  int16_t ch; //Variabel untuk menampung hasil pembacaan satu karakter dari file, Tipe int16_t supaya bisa menampung nilai -1 (EOF).
  while ((ch = f.read()) >= 0) {// Loop membaca file **satu karakter** per‑iterasi sampai EOF.
    char c = (char)ch; // konversi ke tipe char
    if (c=='\r') continue; //Abaikan karakter carriage‑return '\r', lalu kembali ke awal while untuk baca karakter berikutnya
    if (c!='\n' && len<sizeof(line)-1) { //Jika karakter bukan newline '\n' dan masih ada ruang di buffer
      line[len++]=c; //tambahkan ke array line dan naikkan len.
      continue; } //kembali ke awal while untuk baca karakter berikutnya
    
    //Di sini berarti kita baru saja menemui '\n' **atau** buffer sudah penuh.
    line[len]=0; // tutup string dengan \0
    len=0; // reset panjang untuk baris berikutnya
    if(!line[0]) continue; //Jika baris kosong (string pertama = '\0'), lewati saja.
    if(line[0]=='s'||line[0]=='S'||line[0]=='N'||line[0]=='n') continue; // Abaikan baris yang dimulai dengan huruf s/S/N/n. separator dan no
    // parse kolom pertama (No)
    uint32_t no=0;  //Siapkan variabel untuk menampung nomor yang berada di kolom pertama.
    char* p=line; // pointer ke awal string
    while(*p>='0' && *p<='9'){ //Selama karakter masih antara '0'‑'9', ubah menjadi nilai numerik.
      no = no*10 + (*p-'0'); // contoh: "12" → 1*10+2 = 12
      p++; 
    }
    if (no) lastNo = no; //Jika memang ada nomor (no > 0), simpan sebagai nomor terakhir.
  }
  f.close(); //Tutup file untuk membebaskan resource kartu SD.
  return lastNo; //mengembalikan nilai lastno ke pemanggil
}




//==================================HANDLE BUTTON================================================================================================

void handleButton1() {
  unsigned long now = millis();
  bool raw = (digitalRead(button1) == LOW);  // wiring: tombol ke GND, pin INPUT_PULLUP
  // Reset sinyal output (hanya 1x per panggilan)
  newShortPress1 = false;
  newLongPress1  = false;
  // ===== Debounce raw -> stable =====
  static bool lastRaw = false;
  static unsigned long lastEdgeAt = 0;
  static bool lastStable = false;          // state stabil terakhir (false = lepas, true = ditekan)
  static unsigned long pressAt = 0;
  static bool longTriggered = false;       // kunci agar long-press hanya 1x

  if (raw != lastRaw) {                    // ada perubahan raw -> mulai debounce window
    lastRaw = raw;
    lastEdgeAt = now;
  }
  if (now - lastEdgeAt < DEBOUNCE_MS) {
    return;                                // masih dalam window debounce, abaikan
  }

  bool current = lastRaw;                  // sudah stabil

  // ===== Transisi state stabil =====
  if (current != lastStable) {
    lastStable = current;
    if (current) {
      // Baru DITEKAN
      pressAt = now;
      longTriggered = false;
    } else {
      // Baru DILEPAS
      unsigned long dur = now - pressAt;
      // short-press hanya kalau tidak sempat long-press
      if (!longTriggered && dur < SHORT_PRESS_MAX) {
        newShortPress1 = true;             // <<— 1 klik = 1 trigger
        btnBeepRequest = true;
      }
    }
    return;                                // selesai proses transisi
  }

  // ===== Tombol sedang DITEKAN (stabil) =====
  if (current) {
    unsigned long dur = now - pressAt;
    if (!longTriggered && dur >= LONG_PRESS_MIN) {
      newLongPress1 = true;                // picu SEKALI
      longTriggered = true;
      btnBeepRequest = true;
    }
  }
}

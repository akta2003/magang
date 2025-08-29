// ===== JSN-SR04T x4, ATmega328PB, NON-BLOCKING =====
// Timer1: 1 µs tick (8 MHz / 8) untuk ukur lebar ECHO
// PCINT  : deteksi rising/falling di ECHO (PD0, PB2, PD1, PB7)
// Timer2 : CTC 1 ms untuk scheduler tembak bergiliran + timeout + GAP
// LCD 16x2 + LED kedip makin cepat saat objek makin dekat

#define F_CPU 8000000UL
#include <Arduino.h>
#include <avr/pgmspace.h>  
#include <LiquidCrystal.h>
#include <EEPROM.h>

// ===== LCD pins =============================================
const int rs=4, en=5, d4=6, d5=7, d6=8, d7=9;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// ===== Mapping sensor =====
const uint8_t TRIG[4] = {16, 15, 14, 20};
const uint8_t ECHO[4] = { 0, 10,  1, 21};

//===== Tombol=====
const unsigned long DEBOUNCE_MS = 25;  // 20–30 ms cukup
const uint8_t button1 = 11;
const uint8_t button2 = 12;
const uint8_t button3 = 18;

// ===== LED indikator =====
const int  lampu = 3; // BUZZER 3 / LAMPU 19

#define SHORT_PRESS_MAX 500
#define LONG_PRESS_MIN 1000

//=====button 1==========================================================
unsigned long buttonPressTime1 = 0;
bool buttonPressed1 = false;
bool lastButtonState1 = false;
bool newShortPress1 = false;
bool newLongPress1 = false;
//=======button 2========
unsigned long buttonPressTime2 = 0;
bool buttonPressed2 = false;
bool lastButtonState2 = false;
bool newShortPress2 = false;
bool newLongPress2 = false;
//========button 3=======
unsigned long buttonPressTime3 = 0;
bool buttonPressed3 = false;
bool lastButtonState3 = false;
bool newShortPress3 = false;
bool newLongPress3 = false;
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
const uint16_t GAP_MS  = 60;      // jeda antar sensor JSN-SR04T waterproof

// ===== Parameter BUZZER/LED =====
const bool LED_ACTIVE_LOW     = false;   // ubah ke false jika modul aktif-HIGH
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

inline void setLampu(bool on){
  digitalWrite(lampu, LED_ACTIVE_LOW ? (on ? LOW : HIGH): (on ? HIGH : LOW)); //apakah led aktif low?? jika ya aktifkan yang (on ? LOW : HIGH) jika tidak yang satunya, apakah on true? jika ya write low jika gk ya high
} 


// ===== State pembacaan (ISR) =====
volatile uint32_t g_ovf = 0;        // Timer1 overflow counter (extend > 65535)
volatile uint8_t  lastPINB, lastPIND;

//struct  kumpulan dari beberapa variabel dengan beragam tipe data yang dibungkus dalam satu variabel.

struct EchoChan {
  volatile uint8_t  armed;      // 1: sedang menunggu echo untuk kanal ini
  volatile uint8_t  stage;      // 0: tunggu rising, 1: tunggu falling
  volatile uint16_t t_start;    // TCNT1 saat rising
  volatile uint32_t ovf_start;  // g_ovf saat rising
  volatile uint32_t tof_us;     // hasil durasi us
  volatile uint8_t  ready;      // 1: tof_us valid
};
volatile EchoChan ch[4];  //ch[0]-ch[3] berisi struct echo channel

volatile int16_t distance_cm[4] = {-1, -1, -1, -1};

// ===== Waktu sistem 1 ms (Timer2) =====
volatile uint32_t g_ms = 0;
ISR(TIMER2_COMPA_vect) { g_ms++; } //mode ctc per 1ms

// ===== Timer1 overflow extend =====
ISR(TIMER1_OVF_vect) { g_ovf++; }  //mode ketika timer 1 overflow

// ===== PCINT PORTD (PD0=ECHO1, PD1=ECHO3) =====
ISR(PCINT2_vect) { //ketika ada perubahan low ke high atau sebaliknya pada PORTD (PD0–PD7) yang di enable.
  uint8_t now = PIND;  //baca status portd sekarang
  uint8_t changed = now ^ lastPIND; //XOR dengan status sebelumnya → bit yang berubah jadi 1, intinya cek apakah ada perubahan
  lastPIND = now; /// simpan kondisi terbaru untuk perbandingan berikutnya

  // Sensor 0: PD0 (bit 0)
  if (ch[0].armed && (changed & (1 << 0))) { //jika ch[0].armed true dan apakah pin PD0 berubah di changed, jika iya
    uint8_t level = (now >> 0) & 1; //ambil level PD0 (0 atau 1)
    uint16_t t = TCNT1; //baca nilai timer1 sekarang
    if (level && ch[0].stage == 0) {              // rising / naik dari LOW→HIGH
      ch[0].t_start = t;                          // simpan waktu start
      ch[0].ovf_start = g_ovf;                    // simpan overflow counter saat itu
      ch[0].stage = 1;                            // pindah stage: sekarang tunggu falling
    } else if (!level && ch[0].stage == 1) {      // falling / turun dari HIGH→LOW
      uint32_t ticks = (g_ovf - ch[0].ovf_start) * 65536UL + (uint32_t)t - ch[0].t_start;
      ch[0].tof_us = ticks;                       // hasil durasi dalam ticks
      ch[0].ready = 1;                            // tandai hasil sudah siap dibaca
      ch[0].stage = 0;                            // reset ke tahap awal
      ch[0].armed = 0;                            // matikan sensor ini, tunggu trigger berikutnya
    }
  }

  // Sensor 2: PD1 (bit 1)
  if (ch[2].armed && (changed & (1 << 1))) {
    uint8_t level = (now >> 1) & 1;
    uint16_t t = TCNT1;
    if (level && ch[2].stage == 0) {
      ch[2].t_start = t; 
      ch[2].ovf_start = g_ovf; 
      ch[2].stage = 1;
    } else if (!level && ch[2].stage == 1) {
      uint32_t ticks = (g_ovf - ch[2].ovf_start) * 65536UL + (uint32_t)t - ch[2].t_start;
      ch[2].tof_us = ticks; 
      ch[2].ready = 1; 
      ch[2].stage = 0; 
      ch[2].armed = 0;
    }
  }
}

// ===== PCINT PORTB (PB2=ECHO2, PB7=ECHO4) =====
ISR(PCINT0_vect) {  //pot b ada perubahan
  uint8_t now = PINB;
  uint8_t changed = now ^ lastPINB;
  lastPINB = now;

  // Sensor 1: PB2 (bit 2)
  if (ch[1].armed && (changed & (1 << 2))) {
    uint8_t level = (now >> 2) & 1;
    uint16_t t = TCNT1;
    if (level && ch[1].stage == 0) {
      ch[1].t_start = t; 
      ch[1].ovf_start = g_ovf; 
      ch[1].stage = 1;
    } else if (!level && ch[1].stage == 1) {
      uint32_t ticks = (g_ovf - ch[1].ovf_start) * 65536UL + (uint32_t)t - ch[1].t_start;
      ch[1].tof_us = ticks; 
      ch[1].ready = 1; 
      ch[1].stage = 0; 
      ch[1].armed = 0;
    }
  }

  // Sensor 3: PB7 (bit 7)
  if (ch[3].armed && (changed & (1 << 7))) {
    uint8_t level = (now >> 7) & 1;
    uint16_t t = TCNT1;
    if (level && ch[3].stage == 0) {
      ch[3].t_start = t; 
      ch[3].ovf_start = g_ovf; 
      ch[3].stage = 1;
    } else if (!level && ch[3].stage == 1) {
      uint32_t ticks = (g_ovf - ch[3].ovf_start) * 65536UL + (uint32_t)t - ch[3].t_start;
      ch[3].tof_us = ticks; 
      ch[3].ready = 1; 
      ch[3].stage = 0; 
      ch[3].armed = 0;
    }
  }
}

// ===== Util: pulse TRIG 10–12 µs =====
static inline void trigPulse(uint8_t pin) {
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(20);
  digitalWrite(pin, LOW);
}

static inline void trigPulse2(uint8_t pin) {
  digitalWrite(pin, LOW);  delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(110);
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

  // PORTD: PD0(bit0), PD1(bit1)
  PCICR  |= (1 << PCIE2);       //aktifkan portd
  PCMSK2 |= (1 << 0) | (1 << 1); //pilih pin yang diawasi PD0,PD1  echo

  // PORTB: PB2(bit2), PB7(bit7)
  PCICR  |= (1 << PCIE0);       //aktifkan portb
  PCMSK0 |= (1 << 2) | (1 << 7);  //PB2,PB7  echo bit 2 dan 7 diberi nilai 1
}

// ===== Scheduler round-robin =====
enum { S_IDLE, S_WAIT_ECHO, S_GAP_WAIT };
uint8_t  s_idx   = 0;         // index sensor aktif sekarang (0..3)
uint8_t  s_state = S_IDLE;    // status state machine scheduler
uint32_t t_fire_us = 0;       // waktu saat TRIG ditembak
uint32_t t_next_ms = 0;       // kapan boleh trigger sensor berikutnya

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
EXIT
 };
State state = INTRO;
uint32_t introStartMs = 0;

// ===== Setup ==========================================================================
void setup() {
  // TRIG out, ECHO in
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
  }
  pinMode(ECHO[0], INPUT); // PD0
  pinMode(ECHO[1], INPUT); // PB2
  pinMode(ECHO[2], INPUT); // PD1
  pinMode(ECHO[3], INPUT); // PB7

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  pinMode(lampu, OUTPUT);
  setLampu(false);

  pass_load();
  thresh_load();
  mode_load();

  lcd.begin(16, 2);
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
  handleButton2();
  handleButton3();
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

      if (cm1>500){cm1= -1;}
      if (cm2>500){cm2= -1;}
      if (cm3>500){cm3= -1;}
      if (cm4>500){cm4= -1;}

      // === aturan: jika semua "--", lampu OFF ===
      bool anyInvalid = (cm1<0 && cm3<0 && cm2<0 && cm4<0);

      int dmin = minValid4(cm1, cm2, cm3, cm4);
      updateWarningLamp(dmin, anyInvalid, now);

      // 5) Refresh LCD (throttle biar halus, 100 ms)
      static uint32_t lastLcd = 0;
      if (now - lastLcd >= 60) {
      lastLcd = now;

      if (DISPLAY_MODE == 0){
        printCell(0, 0, F("S1:"), cm1);
        printCell(8, 0, F("S2:"), cm2);
        printCell(0, 1, F("S3:"), cm3);
        printCell(8, 1, F("S4:"), cm4);
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
        if (dmin < 0) {
          lcd.print(F("      ----      "));
        } else {
        // cetak meter 1 desimal tanpa sprintf
        uint16_t cm = (uint16_t)dmin;
        // uint16_t whole = cm / 100;
        // uint8_t  tenth = (cm % 100) / 10;
        // lcd.print(whole); lcd.print('.');
        // lcd.print(tenth);
        // lcd.print(F(" m   "));
        printCell(1, 1, F("Jarak : "), dmin);
        lcd.print(F("CM"));

        }

      }

    }
    if (newLongPress1){
      baruMasukInputPassword = true;
      state = INPUT_PASSWORD;
      newLongPress1 = false;
    }

    else if (newLongPress2 && newLongPress3 == true){
      state = RESET;
      newLongPress2 = false;
      newLongPress3 = false;
      
    }

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
      if (newShortPress2){
        // kalau masih kosong, mulai dari 0
          pw_digits[pw_index] = (pw_digits[pw_index] + 1) % 10;
          pw_blinkOn = true; 
          pw_nextBlinkMs = now + PW_BLINK_PERIOD;
          pw_draw(false);
          lastInteractionTime = now;
          newShortPress2 = false;
      
      }else if (newShortPress3){
        // kalau masih kosong, mulai dari 0
          pw_digits[pw_index] = (pw_digits[pw_index] + 9) % 10;
          pw_blinkOn = true; 
          pw_nextBlinkMs = now + PW_BLINK_PERIOD;
          pw_draw(false);
          lastInteractionTime = now;
          newShortPress3 = false;
      
      }else if (newShortPress1){
        if (pw_digits[pw_index] < 0) pw_digits[pw_index] = 0; // auto-commit
        pw_index = (pw_index + 1) % 6;        // selalu wrap
        pw_blinkOn = true;                    // langsung tampil ON
        pw_nextBlinkMs = now + PW_BLINK_PERIOD;
        pw_draw(false);                       // redraw tiap pindah
        lastInteractionTime = now;
        newShortPress1 = false;
      
      }else if (newLongPress1){
        state = VERIFY_PASSWORD;
        newLongPress1 = false;
      }
        // idle timeout balik ke layar utama
      if (now - lastInteractionTime >= 8000) {
      lcd.clear();
      state = UKUR;
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

  // if (!complete) {
  //   if (holdUntil == 0) {
  //   lcd.clear();
  //   lcd.setCursor(0,0); lcd.print("LENGKAPI 6 DIGIT");
  //   if (holdUntil == 0) holdUntil = millis() + 2000;}
  //   if ((long)(millis() - holdUntil) >= 0) {
  //     holdUntil = 0;
  //     baruMasukInputPassword = true;
  //     state = INPUT_PASSWORD;
  //   }
  // } else {
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
  if (newShortPress2) {            // NEXT (button1 short)
    menuIndex = (menuIndex + 1) % JUMLAH_MENU;
    menu_draw(false);
    newShortPress2 = false;
    lastInteractionTime = now;
  }
  if (newShortPress3) {            // PREV (button2 short) – opsional
    menuIndex = (menuIndex + JUMLAH_MENU - 1) % JUMLAH_MENU;
    menu_draw(false);
    newShortPress3 = false;
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
  if (newShortPress2) {
    if (cpw_digits[cpw_index] < 0) cpw_digits[cpw_index] = 0;
    cpw_digits[cpw_index] = (cpw_digits[cpw_index] + 1) % 10;
    cpw_blinkOn = true; cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
    newShortPress2 = false;
    lastInteractionTime = now;
  }
  // button2 long: -1 digit
  if (newShortPress3) {
    if (cpw_digits[cpw_index] < 0) cpw_digits[cpw_index] = 0;
    cpw_digits[cpw_index] = (cpw_digits[cpw_index] + 9) % 10;
    cpw_blinkOn = true; cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
    newShortPress3 = false;
    lastInteractionTime = now;
  }
  // button1 short: pindah kolom
  if (newShortPress1) {
    if (cpw_digits[cpw_index] < 0) cpw_digits[cpw_index] = 0; // auto-commit
    cpw_index = (cpw_index + 1) % 6;
    cpw_blinkOn = true; cpw_nextBlinkMs = now + CPW_BLINK_PERIOD;
    cpw_draw(false);
    newShortPress1 = false;
    lastInteractionTime = now;
  }
  // button1 long: SIMPAN
  if (newLongPress1) {
    newLongPress1 = false;
    // Pastikan semua digit terisi
    //static uint32_t holdUntil = 0;

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

    // Buang event sisa agar tak kebawa
    newShortPress1 = newShortPress2 = newShortPress3 = false;
    newLongPress2  = false;

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
  if (newShortPress2) {            // NEXT (button1 short)
    menuIndexP = (menuIndexP + 1) % JUMLAH_MENU_PARAM;
    menu_draw_param(false);
    newShortPress2 = false;
    lastInteractionTime = now;
  }
  if (newShortPress3) {            // PREV (button2 short) – opsional
    menuIndexP = (menuIndexP + JUMLAH_MENU_PARAM - 1) % JUMLAH_MENU_PARAM;
    menu_draw_param(false);
    newShortPress3 = false;
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
  if (newShortPress2) {
    START_DIST_CM += STEP_CM;
    if (START_DIST_CM > START_MAX) START_DIST_CM = START_MAX;
    if (START_DIST_CM < SOLID_DIST_CM) START_DIST_CM = SOLID_DIST_CM; // jaga hubungan
    newShortPress2 = false;
    lastInteractionTime = now;
  }
  if (newShortPress3) {
    START_DIST_CM -= STEP_CM;
    if (START_DIST_CM < SOLID_DIST_CM) START_DIST_CM = SOLID_DIST_CM; // tidak boleh < SOLID
    newShortPress3 = false;
    lastInteractionTime = now;
  }

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
  if (newShortPress2) {
    SOLID_DIST_CM += STEP_CM;
    if (SOLID_DIST_CM > START_DIST_CM) SOLID_DIST_CM = START_DIST_CM; // tak boleh > START
    newShortPress2 = false;
    lastInteractionTime = now;
  }
  if (newShortPress3) {
    SOLID_DIST_CM -= STEP_CM;
    if (SOLID_DIST_CM < SOLID_MIN) SOLID_DIST_CM = SOLID_MIN;         // batas bawah 25
    newShortPress3 = false;
    lastInteractionTime = now;
  }

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

  if (newShortPress2) {
    newShortPress2 = false;
    DISPLAY_MODE = (DISPLAY_MODE + 1) % 2;
    lastInteractionTime = now;
  }else if (newShortPress3) {
    newShortPress2 = false;
    DISPLAY_MODE = (DISPLAY_MODE + 1) % 2;
    lastInteractionTime = now;
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
  // 1) Jika ada sensor "--" => lampu OFF
  if (anyInvalid) { setLampu(false); phase = PH_OFF; phaseStartMs = now; return; }

  // 2) Jika semua > 300 cm atau tak ada baca valid => OFF
  // if (dmin == -1 || dmin > START_DIST_CM) {
  //   setLampu(false); phase = PH_OFF; phaseStartMs = now; return;
  // }

  // 3) Jika <= 40 cm => ON solid
  if (dmin <= SOLID_DIST_CM) { setLampu(true); return; }

  // 4) 40..300 cm => kedip: ON 50ms, OFF menurun dari 950ms -> 0ms
  unsigned long offTarget = offMsFromDistance(dmin);

  if (phase == PH_ON) {
    if (now - phaseStartMs >= ON_MS) {
      setLampu(false);
      phase = PH_OFF;
      phaseStartMs = now;
    }
  } else { // PH_OFF
    if (now - phaseStartMs >= offTarget) {
      setLampu(true);
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
static inline int minValid4(int a, int b, int c, int d) {
  const int INF = 10000;
  int m = INF;
  if (a > 0 && a < 500) m = min(m, a);
  if (b > 0 && b < 500) m = min(m, b);
  if (c > 0 && c < 500) m = min(m, c);
  if (d > 0 && d < 500) m = min(m, d);
  return (m == INF) ? -1 : m;
}

//==========SCHEDULER UNTUK PENGGANTI PULSEIN=====================================
void scheduler_step() {
  switch (s_state) {
    case S_IDLE:
      if ((int32_t)(g_ms - t_next_ms) >= 0) {  //jika nilai timer2 g_ms - t_next_ms lebih dari sama dengan 0, jika iya
        ch[s_idx].armed = 1; //ch[0].armed = 1 == siap nangkap echo untuk port ch[0]
        ch[s_idx].stage = 0; //ch[0].stage = 0 == tunggu rising
        ch[s_idx].ready = 0; //ch[0].ready = 0 == tof_us belum dibaca valid
        if (s_idx == 0){
          trigPulse(TRIG[s_idx]); //Kirim pulsa trigger → sensor mulai bekerja.
        }
         if (s_idx == 1){
          trigPulse2(TRIG[s_idx]); //Kirim pulsa trigger → sensor mulai bekerja.
        }
         if (s_idx == 2){
          trigPulse2(TRIG[s_idx]); //Kirim pulsa trigger → sensor mulai bekerja.
        }
         if (s_idx == 3){
          trigPulse(TRIG[s_idx]); //Kirim pulsa trigger → sensor mulai bekerja.
        }
        t_fire_us = micros();   //catat waktu tembak
        s_state = S_WAIT_ECHO;  //pindah state wait_echo
      }
      break;

    case S_WAIT_ECHO:
      if (ch[s_idx].ready) { //ketika echo sudah terbaca valid == ready == 1
        distance_cm[s_idx] = (int16_t)(ch[s_idx].tof_us / 58UL); //menghitung jarak cm, gk pake 0.0343/2 karena lambat. jadi kontanta dibalik (dt=tof*0.0343/2 jadi dt=tof*0.01715 lalu dibalik jadi dt = tof/1/0.0175 = 58)
        t_next_ms = g_ms + GAP_MS; //boleh trigger lagi ketika waktu sudah melewati gapnya(jeda)
        s_state = S_GAP_WAIT; //pindah state ke gap wait
      } else if ((uint32_t)(micros() - t_fire_us) > TOUT_US) { //ketika waktu sudah melebihi timeout tapi ready masih == 0, jika iya
        ch[s_idx].armed = 0; //set armed balik ke 0
        ch[s_idx].stage = 0; //posisi balikkan ke 0
        distance_cm[s_idx] = -1; //return nilai -1
        t_next_ms = g_ms + GAP_MS; //set delay per pembacaan
        s_state = S_GAP_WAIT; //pindah state
      }
      break;

    case S_GAP_WAIT:
      if ((int32_t)(g_ms - t_next_ms) >= 0) {  //nunggu delay gap ms terpenuhi, jika iya
        s_idx = (s_idx + 1) & 0x03; // 0..3 //lanjut ke ch[1] dan terus hingga 3 lalu balik 0
        s_state = S_IDLE; //balik ke state idle (mengulang)
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


// ===================== Flag permintaan bunyi dari handler ========================================================================================
volatile bool btnBeepRequest = false;
// One-shot 50 ms tanpa delay

static inline void buzzer_service(){
  static bool on = false;
  static unsigned long offAt = 0;

  // trigger baru?
  if (btnBeepRequest){
    btnBeepRequest = false;  // konsumsi
    setLampu(true);
    on = true;
    offAt = millis() + 50;   // 50 ms
  }

  // matikan setelah 50 ms
  if (on && (long)(millis() - offAt) >= 0){
    setLampu(false);
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

void handleButton2() {
  unsigned long now = millis();
  bool raw = (digitalRead(button2) == LOW);  // wiring: tombol ke GND, pin INPUT_PULLUP
  // Reset sinyal output (hanya 1x per panggilan)
  newShortPress2 = false;
  newLongPress2  = false;
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
        newShortPress2 = true;             // <<— 1 klik = 1 trigger
        btnBeepRequest = true;
      }
    }
    return;                                // selesai proses transisi
  }

  // ===== Tombol sedang DITEKAN (stabil) =====
  if (current) {
    unsigned long dur = now - pressAt;
    if (!longTriggered && dur >= LONG_PRESS_MIN) {
      newLongPress2 = true;                // picu SEKALI
      longTriggered = true;
      btnBeepRequest = true;
    }
  }
}

void handleButton3() {
  unsigned long now = millis();
  bool raw = (digitalRead(button3) == LOW);  // wiring: tombol ke GND, pin INPUT_PULLUP
  // Reset sinyal output (hanya 1x per panggilan)
  newShortPress3 = false;
  newLongPress3  = false;
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
        newShortPress3 = true;             // <<— 1 klik = 1 trigger
        btnBeepRequest = true;
      }
    }
    return;                                // selesai proses transisi
  }

  // ===== Tombol sedang DITEKAN (stabil) =====
  if (current) {
    unsigned long dur = now - pressAt;
    if (!longTriggered && dur >= LONG_PRESS_MIN) {
      newLongPress3 = true;                // picu SEKALI
      longTriggered = true;
      btnBeepRequest = true;
    }
  }
}
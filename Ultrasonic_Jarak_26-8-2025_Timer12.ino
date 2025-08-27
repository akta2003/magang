// ===== JSN-SR04T x4, ATmega328PB, NON-BLOCKING =====
// Timer1: 1 µs tick (8 MHz / 8) untuk ukur lebar ECHO
// PCINT  : deteksi rising/falling di ECHO (PD0, PB2, PD1, PB7)
// Timer2 : CTC 1 ms untuk scheduler tembak bergiliran + timeout + GAP
// LCD 16x2 + LED kedip makin cepat saat objek makin dekat

#define F_CPU 8000000UL
#include <Arduino.h>
#include <LiquidCrystal.h>

// ===== LCD pins =====
const int rs=4, en=5, d4=6, d5=7, d6=8, d7=9;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// ===== Mapping sensor =====
// const int TRIG1 = 16;     // PC2 / A2
// const int ECHO1 = 0;      // PD0
// const int TRIG2 = 15;     // (PB6/XTAL)
// const int ECHO2 = 10;     // PB2
// const int TRIG3 = 14;     // (PB7/XTAL)
// const int ECHO3 = 1;      // PD1
// const int TRIG4 = 20;     // tinggal masukkan pin
// const int ECHO4 = 21;     // PD1

const uint8_t TRIG[4] = {16, 15, 14, 20};
const uint8_t ECHO[4] = { 0, 10,  1, 21};

// ===== LED indikator =====
// const int  lampu = 19;                 // PC5
const int  lampu = 3;                 // PC5

// ===== Custom characters (punyamu, dibiarkan) =====
byte image03[8] = {0b01000, 0b11100, 0b11110, 0b11110, 0b00000, 0b00000, 0b00000, 0b00000};
byte image02[8] = {0b00000, 0b00111, 0b01111, 0b11111, 0b11100, 0b11110, 0b11110, 0b11110};
byte image01[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00001};
byte image18[8] = {0b11110, 0b11111, 0b11101, 0b11110, 0b11101, 0b11100, 0b10100, 0b10100};
byte image17[8] = {0b00001, 0b00000, 0b00000, 0b00000, 0b00001, 0b00111, 0b00100, 0b01100};

// ===== Param pembacaan & anti-crosstalk =====
const uint32_t TOUT_US = 30000UL;  // timeout echo ~30 ms (≈ 5 m)
const uint16_t GAP_MS  = 60;      // jeda antar sensor JSN-SR04T waterproof

// ===== Param kedip LED =====
// const int WARN_MAX_CM = 300;                 // mulai kedip jika <= 100 cm
// const int WARN_MIN_CM = 40;                  // kedip tercepat saat 15 cm
// const uint16_t WARN_PERIOD_MS      = 1000;  // periode dasar 1 s
// const uint16_t WARN_ON_AT_START_MS = 50;    // di 300 cm: ON 50ms
// const unsigned long PERIOD_MAX_MS = 2000;    // 100 cm -> 2000 ms
// const unsigned long PERIOD_MIN_MS = 30;      // 15 cm  -> 30 ms
// unsigned long lastBlinkMs = 0;
// unsigned long blinkPeriod = PERIOD_MAX_MS;
// bool blinkEnable = false;
// bool ledState = false;

const bool LED_ACTIVE_LOW     = false;   // ubah ke false jika modul aktif-HIGH
const int  START_DIST_CM      = 400;    // mulai bunyi pada 3 m
const int  SOLID_DIST_CM      = 50;     // <= 40 cm: nyala terus
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

// ===== Helper display =====
static inline int minValid4(int a, int b, int c, int d) {
  const int INF = 10000;
  int m = INF;
  if (a > 0 && a < 500) m = min(m, a);
  if (b > 0 && b < 500) m = min(m, b);
  if (c > 0 && c < 500) m = min(m, c);
  if (d > 0 && d < 500) m = min(m, d);
  return (m == INF) ? -1 : m;
}

// ---- hitung lamanya OFF berdasarkan jarak (linier 300->40 cm : 950->0 ms) ----
static inline unsigned long offMsFromDistance(int dcm){
  if (dcm >= START_DIST_CM) return OFF_AT_STARTMS; //jarak aktual lebih besar dari 300cm maka off 950ms
  if (dcm <= SOLID_DIST_CM) return 0; //jika jarak lebih kecil dari 40cm maka full on atau off=0
  // linear interpolation: off = (d-40)/(300-40) * 950
  return (unsigned long)((long)(dcm - SOLID_DIST_CM) * OFF_AT_STARTMS / (START_DIST_CM - SOLID_DIST_CM));
}

// ---- update warning lamp sesuai aturan baru -------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------------

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

// ===== Intro non-blocking =====
enum State { INTRO, UKUR };
State state = INTRO;
uint32_t introStartMs = 0;

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

// ===== Setup =====
void setup() {
  // TRIG out, ECHO in
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
  }
  pinMode(ECHO[0], INPUT); // PD0
  pinMode(ECHO[1], INPUT); // PB2
  pinMode(ECHO[2], INPUT); // PD1
  pinMode(ECHO[3], INPUT); // PB7

  pinMode(lampu, OUTPUT);
  setLampu(false);

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
  scheduler_step();

  // 2) Intro 3 detik non-blocking
  if (state == INTRO) {
    if (millis() - introStartMs >= 3000UL) {
      lcd.clear();
      state = UKUR;
    } else {
      // tetap biarkan scheduler jalan selama intro
      return;
    }
  }
  unsigned long now = millis();
  // 3) Hitung d_min untuk kontrol LED
  int cm1 = distance_cm[0];
  int cm2 = distance_cm[1];
  int cm3 = distance_cm[2];
  int cm4 = distance_cm[3];

  if (cm1>500){cm1= -1;}
  if (cm2>500){cm2= -1;}
  if (cm3>500){cm3= -1;}
  if (cm4>500){cm1= -1;}

  // === aturan: jika salah satu "--", lampu OFF ===
   bool anyInvalid = (cm1<0 && cm3<0 && cm2<0 && cm4<0);

   int dmin = minValid4(cm1, cm2, cm3, cm4);
   updateWarningLamp(dmin, anyInvalid, now);

  // 5) Refresh LCD (throttle biar halus, 100 ms)
  static uint32_t lastLcd = 0;
  if (now - lastLcd >= 60) {
    lastLcd = now;
    printCell(0, 0, F("S1:"), cm1);
    printCell(8, 0, F("S2:"), cm2);
    printCell(0, 1, F("S3:"), cm3);
    printCell(8, 1, F("S4:"), cm4);
  }
}

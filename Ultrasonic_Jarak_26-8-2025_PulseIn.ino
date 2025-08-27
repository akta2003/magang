//nyalakan lampu kedip semain dekat semakin cepat

#include <LiquidCrystal.h>

const int rs=4,en=5,d4=6,d5=7,d6=8,d7=9;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// --- Mapping AMAN ---
const int TRIG1 = 16;     // PC2 / A2
const int ECHO1 = 0;      // PD0
const int TRIG2 = 15;     // (PB6/XTAL)
const int ECHO2 = 10;     // PB2
const int TRIG3 = 14;     // (PB7/XTAL)
const int ECHO3 = 1;      // PD1
const int TRIG4 = 20;     // tinggal masukkan pin
const int ECHO4 = 21;     // PD1

// const int lampu = 19;     // PC5
const int lampu = 3;     // PC5

byte image03[8] = {0b01000,0b11100,0b11110,0b11110,0b00000,0b00000,0b00000,0b00000};
byte image02[8] = {0b00000,0b00111,0b01111,0b11111,0b11100,0b11110,0b11110,0b11110};
byte image01[8] = {0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00001,0b00001};
byte image18[8] = {0b11110,0b11111,0b11101,0b11110,0b11101,0b11100,0b10100,0b10100};
byte image17[8] = {0b00001,0b00000,0b00000,0b00000,0b00001,0b00111,0b00100,0b01100};

enum State { INTRO, UKUR };
State state=INTRO;

const unsigned long TOUT_US = 30000UL; // 30 ms
const unsigned long GAP_MS  = 80;      // anti-crosstalk utk JSN/SR04M

// === PARAM LOGIKA WARNING BARU ===
const bool LED_ACTIVE_LOW     = false;   // ubah ke false jika modul aktif-HIGH
const int  START_DIST_CM      = 300;    // mulai bunyi pada 3 m
const int  SOLID_DIST_CM      = 40;     // <= 40 cm: nyala terus
const unsigned ON_MS          = 50;     // 50 ms hidup
const unsigned OFF_AT_STARTMS = 950;    // 950 ms mati di 3 m (50+950=1000 ms)

// ======== STATE kedip (non-blocking) ========
enum BlinkPhase { PH_OFF, PH_ON };
BlinkPhase phase = PH_OFF;
unsigned long phaseStartMs = 0;

inline void setLampu(bool on){
  digitalWrite(lampu, LED_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

// ---- helper ukur ----
static int measure(int trig, int echo){
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(20);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, TOUT_US);
  if (!dur) return -1;
  return (int)(dur * 0.0343f / 2.0f);
}
static int measure2(int trig, int echo){
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(110);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, TOUT_US);
  if (!dur) return -1;
  return (int)(dur * 0.0343f / 2.0f);
}

// ---- helper: ambil jarak terkecil yang valid ----
static inline int minValid3(int a, int b, int c, int d) {
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
  if (dcm >= START_DIST_CM) return OFF_AT_STARTMS;
  if (dcm <= SOLID_DIST_CM) return 0;
  // linear interpolation: off = (d-40)/(300-40) * 950
  return (unsigned long)((long)(dcm - SOLID_DIST_CM) * OFF_AT_STARTMS / (START_DIST_CM - SOLID_DIST_CM));
}

// ---- update warning lamp sesuai aturan baru ----
void updateWarningLamp(int dmin, bool anyInvalid, unsigned long now){
  // 1) Jika ada sensor "--" => lampu OFF
  if (anyInvalid) { setLampu(false); phase = PH_OFF; phaseStartMs = now; return; }

  // 2) Jika semua > 300 cm atau tak ada baca valid => OFF
  if (dmin == -1 || dmin > START_DIST_CM) {
    setLampu(false); phase = PH_OFF; phaseStartMs = now; return;
  }

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

void setup(){
  pinMode(TRIG1,OUTPUT); digitalWrite(TRIG1,LOW); pinMode(ECHO1,INPUT);
  pinMode(TRIG2,OUTPUT); digitalWrite(TRIG2,LOW); pinMode(ECHO2,INPUT);
  pinMode(TRIG3,OUTPUT); digitalWrite(TRIG3,LOW); pinMode(ECHO3,INPUT);
  pinMode(TRIG4,OUTPUT); digitalWrite(TRIG4,LOW); pinMode(ECHO4,INPUT);
  pinMode(lampu,OUTPUT);  setLampu(false);

  lcd.begin(16,2);
  lcd.clear();
  lcd.createChar(0, image03);
  lcd.createChar(1, image02);
  lcd.createChar(2, image01);
  lcd.createChar(3, image18);
  lcd.createChar(4, image17);

  phaseStartMs = millis(); // init timer kedip
}

void loop(){
  if (state == INTRO) {
    lcd.clear();
    lcd.setCursor(9, 0); lcd.write(byte(0));
    lcd.setCursor(8, 0); lcd.write(byte(1));
    lcd.setCursor(7, 0); lcd.write(byte(2));
    lcd.setCursor(8, 1); lcd.write(byte(3));
    lcd.setCursor(7, 1); lcd.write(byte(4));
    delay(3000);
    state = UKUR;
    lcd.clear();
  }
  else if (state == UKUR){
    unsigned long now = millis();

    int cm1 = measure(TRIG1,ECHO1); delay(GAP_MS);
    int cm2 = measure2(TRIG2,ECHO2); delay(GAP_MS);
    int cm3 = measure2(TRIG3,ECHO3); delay(GAP_MS);
    int cm4 = measure(TRIG4,ECHO4);

    if (cm1>500){cm1= -1;}
    if (cm2>500){cm2= -1;}
    if (cm3>500){cm3= -1;}
    if (cm4>500){cm1= -1;}

    // === aturan: jika salah satu "--", lampu OFF ===
    bool anyInvalid = ((cm1<25) && (cm2<25) && (cm3<25) && (cm4<25));

    int dmin = minValid3(cm1, cm2, cm3, cm4);  // jarak terdekat (valid)
    updateWarningLamp(dmin, anyInvalid, now);

    // === LCD ===
    lcd.setCursor(0,0); lcd.print("S1: ");
    if (cm1<0) lcd.print("-- "); else { lcd.print(cm1); lcd.print(" "); }
    lcd.setCursor(9,0); lcd.print("S2: ");
    if (cm2<0) lcd.print("-- "); else { lcd.print(cm2); lcd.print(" "); }

    lcd.setCursor(0,1); lcd.print("S3: ");
    if (cm3<0) lcd.print("-- "); else { lcd.print(cm3); lcd.print("  "); }
    lcd.setCursor(9,1); lcd.print("S4: ");
    if (cm4<0) lcd.print("-- "); else { lcd.print(cm4); lcd.print("  "); }
  }
}

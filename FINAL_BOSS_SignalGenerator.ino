//koreksi double relay off normal relay normal

#include <EEPROM.h>
#include <FlexiTimer2.h>

// === KONFIGURASI PIN ======================================================
int segment[8] = { A0, A5, 7, 9, 10, A2, 6, 8 };  // a,b,c,d,e,f,g,dp
int digit[4] = { 11, 12, 13, 5 };                 // digit1–digit4

#define BUTTON_ADC7 7
#define SWITCH1 A1  //set waktu 1 menit
#define SWITCH2 A3  //set waktu 2 menit
#define SWITCH3 A4  //set waktu 3 menit
#define SWITCH4 4   //set waktu 4 menit
#define PIN_OUT 21  //pin untuk menyalakan relay
#define PIN_TRIGGER 2 //pin untuk membaca trigger

// === FSM STATES ==========================================================
enum State {
  STANDBY,
  INPUT_PASSWORD,
  VERIFY_PASSWORD,
  MENU_NAVIGATION,
  MENU_PASSWORD_CHANGE,
  PASSWORD_CHANGED,
  MENU_CT_SETUP,
  MENU_CT_PICK_TIME,
  COUNTDOWN_RUNNING,
  COUNTDOWN_DONE,
  PULSA_DISPLAY,
  RESET
};

State state = STANDBY; //awal start posisi state berada di standby

// === 7-SEGMENT DATA ======================================================
int angkaSegment[10][7] = {
  { 1, 1, 1, 1, 1, 1, 0 },  // 0
  { 0, 1, 1, 0, 0, 0, 0 },  // 1
  { 1, 1, 0, 1, 1, 0, 1 },  // 2
  { 1, 1, 1, 1, 0, 0, 1 },  // 3
  { 0, 1, 1, 0, 0, 1, 1 },  // 4
  { 1, 0, 1, 1, 0, 1, 1 },  // 5
  { 1, 0, 1, 1, 1, 1, 1 },  // 6
  { 1, 1, 1, 0, 0, 0, 0 },  // 7
  { 1, 1, 1, 1, 1, 1, 1 },  // 8
  { 1, 1, 1, 1, 0, 1, 1 }   // 9
};

byte hurufSegment[27][7] = {
  { 1, 1, 1, 1, 1, 0, 1 },  // A
  { 0, 0, 1, 1, 1, 1, 1 },  // B
  { 0, 0, 0, 1, 1, 0, 1 },  // C
  { 0, 1, 1, 1, 1, 0, 1 },  // D
  { 1, 0, 0, 1, 1, 1, 1 },  // E
  { 1, 0, 0, 0, 1, 1, 1 },  // F
  { 1, 0, 1, 1, 1, 1, 0 },  // G
  { 0, 0, 1, 0, 1, 1, 1 },  // H
  { 1, 0, 0, 0, 1, 0, 0 },  // I
  { 1, 0, 1, 1, 0, 0, 0 },  // J
  { 1, 0, 1, 0, 1, 1, 1 },  // K
  { 0, 0, 0, 1, 1, 1, 0 },  // L
  { 1, 0, 1, 0, 1, 0, 1 },  // M
  { 0, 0, 1, 0, 1, 0, 1 },  // N
  { 0, 0, 1, 1, 1, 0, 1 },  // O
  { 1, 1, 0, 0, 1, 1, 1 },  // P
  { 1, 1, 1, 0, 0, 1, 1 },  // Q
  { 0, 0, 0, 0, 1, 0, 1 },  // R
  { 1, 0, 1, 1, 0, 1, 1 },  // S
  { 0, 0, 0, 1, 1, 1, 1 },  // T
  { 0, 0, 1, 1, 1, 0, 0 },  // U
  { 0, 1, 0, 1, 0, 1, 0 },  // V
  { 0, 1, 0, 1, 0, 1, 1 },  // W
  { 0, 0, 1, 0, 1, 0, 0 },  // X
  { 0, 1, 1, 1, 0, 1, 1 },  // Y
  { 1, 1, 0, 1, 1, 0, 0 },   // Z
  { 0, 0, 0, 0, 0, 0, 1 }   // '-'
};

// === SET WAKTU TEKAN ==========================
#define SHORT_PRESS_MAX 500
#define LONG_PRESS_MIN 800
#define VERY_LONG_PRESS_MIN 7000

// === VARIABEL ===
unsigned long buttonPressTime = 0;
//bool buttonPressed = false;
bool lastButtonState = false;

bool newShortPress = false;
bool newLongPress = false;
bool newVeryLongPress = false;

int inputPassword[4] = { 0, 0, 0, 0 };
int savedPassword[4];
int digitIndex = 0;

int countdownMenit ;
unsigned int savedCountdownMenit;
uint8_t ctEditField = 0;          // 0 = edit menit, 1 = edit detik
uint8_t countdownDetik = 0;       // detik yang diset user
uint8_t savedCountdownDetik = 0;  // detik default dari EEPROM

unsigned long countdownStart = 0;
int menuIndex = 0;  // 0: UP, 1: CT, 2:EX, 3:PLS
const int JUMLAH_MENU = 4;

bool countdownBaruDimasuki = true;

unsigned long waktuMasukCTSetup = 0;
bool baruMasukCTSetup = true;

bool baruMasukMenuPassword = true;
unsigned long waktuMasukMenuPassword = 0;

unsigned long lastInteractionTime = 0;
const unsigned long IDLE_TIMEOUT = 300000;  // 30 detik untuk mati otomatis 300s

bool baruMasukCountdown = true;

bool currentTrigger = false;
bool prevTriggerState;  // Kondisi sebelumnya dari trigger
bool triggerPernahHIGH = false;        // Flag: apakah trigger pernah HIGH sejak hidup

// bool baruMasukInputPassword = true;

bool sudahWave = false;

//PULSE COUNT
volatile uint32_t pulse_count   = 0;  // ditambah oleh ISR INT0
volatile uint32_t display_value = 0;  // snapshot pulsa per 1 detik (di-set Timer2 ISR)
volatile uint32_t display_value2 = 0;  // snapshot pulsa per 1 detik (di-set Timer2 ISR)
uint16_t ms = 0;

// === SETUP ===========================================================================================
void setup() {
  attachInterrupt(digitalPinToInterrupt(PIN_TRIGGER), isrPulse, RISING);
  for (int i = 0; i < 8; i++) pinMode(segment[i], OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(digit[i], OUTPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);
  pinMode(SWITCH4, INPUT_PULLUP);
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_TRIGGER, INPUT); //BISA PAKE PULLUP KALO UDAH PAKE SENSOR
  digitalWrite(PIN_OUT, LOW);
  loadPassword();
  prevTriggerState = currentTrigger; // membaca kondisi pin trigger
  triggerPernahHIGH = (prevTriggerState == HIGH);  // trigger pernah high true kalau saat nyala langsung HIGH

  // FlexiTimer2::set(500, 1.0/1000, flash); // call every 500 1ms "ticks"
  FlexiTimer2::set(1, flash); // MsTimer2 style is also supported
  FlexiTimer2::start();

  lastInteractionTime = millis();
}

// === LOOP ===========================================================================================
void loop() {
  handleGlobalInterrupt();
  handleButton();

  //=== STANDBY =======================================================================================
  if (state == STANDBY) {
  //   if (!sudahWave) {            // cek apakah state sudah pernah menyala, jika belum mainkan wave
  //   modeStandbyWave();
  //   sudahWave = true;           //set ke true untuk penanda
  // } else {
      tampilkanHuruf("----");
  if (newShortPress) { //jika ditekan singkat
    for (int i = 0; i < 4; i++) inputPassword[i] = 0; //input password di 0 kan semua
    // baruMasukInputPassword = true; //set flag kalo baru masukinputpw = true
    state = INPUT_PASSWORD;
    digitIndex = 0;
    newShortPress = false;
    newLongPress = false;
  }  else if (newVeryLongPress){ //jika ditekan sangat lama maka akan ke state reset
    state = RESET;
    newVeryLongPress = false;}
  // }

    //=== INPUT_PASSWORD ========================================================
  } else if (state == INPUT_PASSWORD) {
  // Inisialisasi awal saat baru masuk ke state ini
  // if (baruMasukInputPassword) { //jika baru masuk input password = true
  lastInteractionTime = millis();      // set waktu awal
  //   baruMasukInputPassword = false;      // agar tidak diulang tiap loop
  // }

  tampilkanPasswordBerkedip(inputPassword, digitIndex); //menampilkan password di seven segment berkedip

  if (newShortPress) { //jika ditekan singkat
    lastInteractionTime = millis(); 
    inputPassword[digitIndex] = (inputPassword[digitIndex] + 1) % 10; //mengubah nilai input password dari 0-9 lalu disimpan ke array inputPassword[digitIndex] digit index 0-4

  } else if (newLongPress) { //jika tekan lama
    lastInteractionTime = millis();
    digitIndex++; //digit index +1 setiap longpress
    if (digitIndex >= 4) { //jika sudah 4 maka set ke 0 dan pergi ke verify
      digitIndex = 0;
      state = VERIFY_PASSWORD;
      newLongPress = false;
      // baruMasukInputPassword = true;  // siap untuk state berikutnya
    }
  }

  // Timeout jika tidak ada interaksi tombol selama 5 detik
  else if (millis() - lastInteractionTime > 5000) {
    digitIndex = 0;
    for (int i = 0; i < 4; i++) inputPassword[i] = 0;
    state = STANDBY;
    // baruMasukInputPassword = true;  // reset untuk saat masuk lagi
  }

    //=== VERIFY_PASSWORD =======================================================
  } else if (state == VERIFY_PASSWORD) {
    unsigned long start = millis();
    if (checkPassword()) { //jika cek password() = true maka akan open dan pindah ke menu navugasi
      while (millis() - start < 1500) {
        tampilkanHuruf("OPEN");
      }
      state = MENU_NAVIGATION;
    } else { //jika cek password = false maka akan pindah ke standby
      while (millis() - start < 2000) {
        tampilkanHuruf("FAIL");
      }
      for (int i = 0; i < 4; i++) inputPassword[i] = 0; //set pw ke 0 lagi
      state = STANDBY;
    }

    //=== MENU_NAVIGATION =======================================================
  } else if (state == MENU_NAVIGATION) {
    // Tampilkan menu sesuai index
    if (menuIndex == 0) tampilkanHuruf("UP__");
    else if (menuIndex == 1) tampilkanHuruf("CD__");
    else if (menuIndex == 2) tampilkanHuruf("EX__");
    else if (menuIndex == 3) tampilkanHuruf("PLS_");

    if (newShortPress) {
      lastInteractionTime = millis();
      menuIndex = (menuIndex + 1) % JUMLAH_MENU; //menu index +1 jika shortpress dan mengulang ke 0 setelah mencapai angka 2
    } else if (newLongPress) {
      lastInteractionTime = millis();
      if (menuIndex == 0) {
        state = MENU_PASSWORD_CHANGE;
      } else if (menuIndex == 1) {
        state = MENU_CT_SETUP;
      } else if (menuIndex == 2) {
        for (int i = 0; i < 4; i++) inputPassword[i] = 0;
        state = STANDBY;
      } else if (menuIndex == 3) {
        state = PULSA_DISPLAY;
      }
      digitIndex = 0;
      newShortPress = false;
      newLongPress = false;
    }
  }

  //=== MENU_PASSWORD_CHANGE ========================================================
  else if (state == MENU_PASSWORD_CHANGE) {
    if (baruMasukMenuPassword) { //cek jika baru masuk menu password = true
      waktuMasukMenuPassword = millis();  // catat waktu pertama masuk
      baruMasukMenuPassword = false;
      for (int i = 0; i < 4; i++) inputPassword[i] = savedPassword[i]; //input password disamakan dengan password yang tersimpan
    }
    tampilkanPasswordBerkedip(inputPassword, digitIndex);

    if (newShortPress) {
      lastInteractionTime = millis();
      inputPassword[digitIndex] = (inputPassword[digitIndex] + 1) % 10; //mengubah angka dari 0-9
      newShortPress = false;
    } else if (newLongPress) {
      lastInteractionTime = millis();
      digitIndex++; //digit +1
        if (digitIndex >= 4) { // jika 4 digit = 0
          digitIndex = 0;
          newLongPress = false;
          bool is9999 = true; 
            for (int i = 0; i < 4; i++) {
              if (inputPassword[i] != 9) { //cek apakah password 9999, Jika ada satu saja digit yang bukan 9
                is9999 = false; //maka false
                break;
              }
            }

            if (is9999) { //jika password 9999
              waktuMasukMenuPassword = millis();  // reset timeout
              while (millis() - waktuMasukMenuPassword < 2000) { // Tampilkan error, jangan simpan password
                tampilkanHuruf("ERR-");
                for (int i = 0; i < 4; i++) inputPassword[i] = savedPassword[i]; //mengembalikan input password ke password tersimpan
                state = MENU_NAVIGATION;
              }
            // Tetap di MENU_PASSWORD_CHANGE
            } else { //jika bukan 9999
                for (int i = 0; i < 4; i++) {
                  savedPassword[i] = inputPassword[i];  // Salin nilai inputpassword ke password tersimpan
                }
                savePassword(); //catat ke eeprom
                waktuMasukMenuPassword = millis();
                state = PASSWORD_CHANGED;
              }
        }
      }

    //=== PASSWORD_CHANGED ============================================================
  } else if (state == PASSWORD_CHANGED) {
    tampilkanHuruf("DONE");
    if (millis() - waktuMasukMenuPassword > 1000) {
      state = MENU_NAVIGATION;
      baruMasukMenuPassword = true;
    }
  }

  //=== MENU_CT_SETUP =================================================================
  else if (state == MENU_CT_SETUP) {
    if (baruMasukCTSetup) {
      waktuMasukCTSetup = millis();  // catat waktu pertama kali masuk
      baruMasukCTSetup = false;
    }
    tampilkanHuruf("SET");
    if (millis() - waktuMasukCTSetup > 1000 && newLongPress) { //jika waktu sekarang dikurangi waktu masuk lebih besar dari 1000 dan tombol ditekan lama
      lastInteractionTime = millis();
      countdownMenit = 0;            // ← reset dulu, biar tidak langsung 1
      countdownDetik = 0;
      ctEditField   = 0;           // 0 = edit menit
      state = MENU_CT_PICK_TIME;
      baruMasukCTSetup = true;  // reset untuk masuk berikutnya
    }

    //=== MENU_CT_PICK_TIME ============================================================
  } else if (state == MENU_CT_PICK_TIME) {
    // unsigned long total2 = countdownMenit * 60;
    // tampilkanWaktuSmart(total2);
    renderEditTimeBlink(countdownMenit, countdownDetik, ctEditField);
    //tampilkanAngkaCountdown(countdownMenit);
    if (newShortPress) {
      lastInteractionTime = millis();
      if (ctEditField == 0) { 
        countdownMenit = (countdownMenit + 1) % 61; // menit 0..60 (kalau mau 1..60 ganti jadi (countdownMenit % 60) + 1)
      } else { 
        countdownDetik = (countdownDetik + 1) % 60; // detik 0..60 (kalau mau 1..60 ganti jadi (countdownDetik % 60) + 1)
        }
    } else if (newLongPress) {
        lastInteractionTime = millis();
        if (ctEditField == 0) {
          // pindah dari menit ke detik
          ctEditField = 1;
        } else {
        // konfirmasi simpan, balik ke menu
        EEPROM.update(10, countdownMenit);
        EEPROM.update(11, countdownDetik);
        savedCountdownMenit = countdownMenit;
        savedCountdownDetik = countdownDetik;
        state = MENU_NAVIGATION;
        }
      }

    //=== COUNTDOWN_RUNNING =====================================================================
  } else if (state == COUNTDOWN_RUNNING) {
   
    if (baruMasukCountdown) {
    countdownStart = millis();      // Set sekali saja
    baruMasukCountdown = false;
  }
    unsigned long elapsed = (millis() - countdownStart) / 1000;
    unsigned long total = (unsigned long)countdownMenit * 60UL + (unsigned long)countdownDetik; // pakai menit+detik
    unsigned long sisa = (elapsed >= total) ? 0 : (total - elapsed);  //Jika waktu yang sudah lewat >= total, maka sisa waktu = 0
    tampilkanWaktuSmart(sisa);
     if (newVeryLongPress) {
      sisa = 0;
     }
    lastInteractionTime = millis();
    if (sisa == 0) state = COUNTDOWN_DONE;

    //=== COUNTDOWN_DONE ============================================================
  } else if (state == COUNTDOWN_DONE) {
    sudahWave = true;
    digitalWrite(PIN_OUT, LOW);
    //tampilkanAngkaKedip(0);
    //if (newShortPress) {
    //   lastInteractionTime = millis();
    //   if (digitalRead(PIN_TRIGGER) == LOW) {
    state = STANDBY;
    // } else{state = STANDBY;}
  
    //=== DISPLAY PULSA =======================================================================================
  }else if (state == PULSA_DISPLAY){
    if (digitalRead(PIN_OUT) == HIGH){
    tampilkanAngkaDesimal(display_value);
    } else {
      tampilkanAngkaDesimal(display_value2);
    }
    
    if (newShortPress) { //jika ditekan singkat
    lastInteractionTime = millis(); 
    state = STANDBY;
  }

    //=== RESET ===============================================================
  } else if (state == RESET) {
    savedPassword[0] = 1;
    savedPassword[1] = 2;
    savedPassword[2] = 3;
    savedPassword[3] = 4;
    savePassword();
    // Tampilkan RST selama 1 detik
    unsigned long startRST = millis();
    while (millis() - startRST < 1000) {
      tampilkanHuruf("RST");
    }
    for (int i = 0; i < 4; i++) inputPassword[i] = 0;
    state = STANDBY;

  }

  //=== CEK TIMEOUT ============================================================================
  // Cek apakah waktu diam lebih dari settingan (30s) dan bukan sedang di state STANDBY
  if (millis() - lastInteractionTime > IDLE_TIMEOUT && state != STANDBY) {
    state = STANDBY;
    lastInteractionTime = millis();  // reset lagi supaya tidak langsung balik lagi
  }
}


// === TAMPILAN ====================================================================
void tampilkanAngkaKedip(int angka) {
  static unsigned long lastBlink = 0;
  static bool on = true;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    on = !on;
  }
  if (on) tampilkan1Digit(3, angka);
  else matikanDigit(3);
}

void tampilkan1Digit(int pos, int value) {
  for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH);
  delayMicroseconds(50);
  for (int j = 0; j < 7; j++) digitalWrite(segment[j], angkaSegment[value][j]);
  digitalWrite(segment[7], LOW);
  digitalWrite(digit[pos], LOW);
  delay(2);
  digitalWrite(digit[pos], HIGH);
}

void matikanDigit(int pos) {
  for (int j = 0; j < 7; j++) digitalWrite(segment[j], LOW);
  digitalWrite(digit[pos], HIGH);
}

// void tampilkanAngka1Digit(int n) {
//   tampilkan1Digit(3, n);
//   delay(2);
// }

// void tampilkanAngka2Digit(int n) {
//   int puluhan = n / 10;
//   int satuan = n % 10;

//   // Tampilkan puluhan di digit[2] (posisi 1 dari kiri)
//   tampilkan1Digit(2, puluhan);

//   // Tampilkan satuan di digit[3] (posisi paling kanan)
//   tampilkan1Digit(3, satuan);
// }

void tampilkanAngkaCountdown(int n) {
  // Matikan semua digit dulu
  for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH);

  if (n < 10) {
    // Hanya tampilkan 1 digit (satuan) di paling kanan
    tampilkan1Digit(3, n);  // digit paling kanan
  } else {
    int puluhan = n / 10;
    int satuan = n % 10;

    tampilkan1Digit(2, puluhan); // digit tengah kanan
    tampilkan1Digit(3, satuan);  // digit paling kanan
  }
}


void tampilkanHuruf(const char* teks) {
  for (int i = 0; i < 4; i++) {
    char c = teks[i];
    if (c >= 'A' && c <= 'Z') {
      tampilkanHuruf1Digit(i, c - 'A');
    } else if (c == '-') {
      tampilkanHuruf1Digit(i, 26);  // indeks ke-26 untuk karakter '-'
    } else {
      matikanDigit(i);  // karakter tidak dikenali, matikan saja
    }
    delay(2);
  }
}

void tampilkanHuruf1Digit(int pos, int idx) {
  for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH);
  delayMicroseconds(50);
  for (int j = 0; j < 7; j++) digitalWrite(segment[j], hurufSegment[idx][j]);
  digitalWrite(digit[pos], LOW);
  delay(2);
  digitalWrite(digit[pos], HIGH);
}

void tampilkanAngkaDesimal(unsigned int val) {
  int angka[4] = { 0 };
  int len = 0;
  do {
    angka[3 - len] = val % 10;
    val /= 10;
    len++;
  } while (val > 0);
  for (int i = 4 - len; i < 4; i++) {
    tampilkan1Digit(i, angka[i]);
    delay(2);
  }
}


void matikanDisplay() {
  for (int i = 0; i < 8; i++) digitalWrite(segment[i], LOW);
  for (int i = 0; i < 4; i++) digitalWrite(digit[i], HIGH);
}

void tampilkanPasswordBerkedip(int arr[4], int aktifIdx) {
  static unsigned long lastBlink = 0;
  static bool on = true;

  if (millis() - lastBlink >= 300) {
    lastBlink = millis();
    on = !on;
  }

  for (int i = 0; i < 4; i++) {
    if (i == aktifIdx && !on) {
      matikanDigit(i);
    } else {
      tampilkan1Digit(i, arr[i]);
    }
    delay(2);
  }
}

void modeStandbyWave() {
  const unsigned long perStep = 120;   // durasi tiap langkah (ms) → atur rasa smooth
  const unsigned long holdFull = 300;  // tahan "----" di akhir sedikit lebih lama

  // 2x sweep “single dash” berjalan
  for (int rep = 0; rep < 2; rep++) { //renderDashFrame(bool d0, bool d1, bool d2, bool d3, unsigned long ms) diulang 2x
    renderDashFrame(true,  false, false, false, perStep); // -___  jadi d0: nyala, d1 = mati, d2 = mati, d3 mati dan ms di set 120ms (ms untuk waktu eksekusi seberapa lama)
    renderDashFrame(false, true,  false, false, perStep); // _-__
    renderDashFrame(false, false, true,  false, perStep); // __-_
    renderDashFrame(false, false, false, true,  perStep); // ___-
  }

  // Sweep terakhir: cumulative fill → -, --, ---, ----
  renderDashFrame(true,  false, false, false, perStep);   // -
  renderDashFrame(true,  true,  false, false, perStep);   // --
  renderDashFrame(true,  true,  true,  false, perStep);   // ---
  renderDashFrame(true,  true,  true,  true,  holdFull);  // ---- (tahan agak lama supaya bisa menyattu dengan ---- standby)
}

// Render satu "frame" selama ms tertentu, dengan pilihan digit mana yang menampilkan '-'.
// d0..d3 = apakah digit 0..3 menampilkan '-' (true) atau kosong (false).
void renderDashFrame(bool d0, bool d1, bool d2, bool d3, unsigned long ms) {
  unsigned long start = millis(); //catat waktu awal start
  while (millis() - start < ms) { //menahan fungsi tetap merender frame ini selama waktu ms.
    for (int i = 0; i < 4; i++) {
      for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH); // matikan semua digit dulu
      delayMicroseconds(50);

      bool on =
        (i == 0 ? d0 : //jika nilai i == 0 maka on = d0
        (i == 1 ? d1 :
        (i == 2 ? d2 : d3))); //jika nilai i == 2 maka on = d2 jika tidak maka i sudah pasti 3 dan on = d3

      if (on) { //jika on = true maka menyalakan segment g atau -
        // huruf '-' ada di indeks 26 pada hurufSegment
        for (int j = 0; j < 7; j++) digitalWrite(segment[j], hurufSegment[26][j]);
      } else { //jika on = false maka segment akan dimatikan
        for (int j = 0; j < 7; j++) digitalWrite(segment[j], LOW);
      }
      digitalWrite(segment[7], LOW); // DP off

      digitalWrite(digit[i], LOW);   // aktifkan digit i
      delay(2);                      // waktu tampil per digit
      digitalWrite(digit[i], HIGH);  // matikan lagi
    }
  }
}


// Tampilkan satu digit dengan opsi blank & DP
void tampilkanDigitDP(int pos, int value, bool dotOn, bool blank) {
  for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH); //offkan semua digit
  delayMicroseconds(50);

  if (blank) { //kalo blank true = mematikan semua segment jika false maka akan memunculkan nilai pada value
    for (int j = 0; j < 7; j++) digitalWrite(segment[j], LOW);
    digitalWrite(segment[7], LOW);
  } else {
    for (int j = 0; j < 7; j++) digitalWrite(segment[j], angkaSegment[value][j]);
    digitalWrite(segment[7], dotOn ? HIGH : LOW); //cek apakah dot on?? jika iya dinyalakan jika tidak matikan
  }

  digitalWrite(digit[pos], LOW);
  delay(2);
  digitalWrite(digit[pos], HIGH);
}

// Tampilkan waktu sesuai aturan: M.SS (tanpa leading zero) atau detik saja
void tampilkanWaktuSmart(unsigned long totalDetik) {
  if (totalDetik >= 60) { // jika total detik lebih dari 60, maka mm diisi dengan misalkan 123s / 60 : 2 | lalu ss diisi dengan modulus dari total detik 123 % 60 = 3
    int mm = totalDetik / 60;
    int ss = totalDetik % 60;

    int d0 = (mm >= 10) ? (mm / 10) : 0;   // puluhan menit | jadi ini cek apakah mm lebih dari 10? jika iya maka nilai d0 = mm/10 jika tidak maka diset 0 (digit0)
    int d1 = mm % 10;                      // satuan menit | ini untuk menyimpan satuan menit dengan modulus (digit1)
    int d2 = ss / 10;                      // puluhan detik | ini untuk menyimpan puluhan pada second
    int d3 = ss % 10;                      // satuan detik | ini untuk menyimpan satuan pada second

    bool blankD0 = (mm < 10);              // kosongkan puluhan menit jika nilai mm lebih kecil dari 10
    // DP hanya di antara menit & detik (digit pos=1)
    tampilkanDigitDP(0, d0, false, blankD0);  // digit 0, nilai d0 akan ditampilkan jika blankD0 bernilai = 0 / false.
    tampilkanDigitDP(1, d1, true,  false); // DP ON di sini | pada digit 1, akan menampilkan nilai d1, dp = true, blank false
    tampilkanDigitDP(2, d2, false, false);
    tampilkanDigitDP(3, d3, false, false);

  } else {
    // totalDetik < 60 → tampil detik saja, tanpa DP (jika nilai detik sudah dibawah 60 second)
    int ss = totalDetik;
    if (ss >= 10) { //jika ss lebih besar dari 10
      tampilkanDigitDP(0, 0, false, true);             // digit 0, nilai 0, dp = false, blank = true
      tampilkanDigitDP(1, 0, false, true);             // blank
      tampilkanDigitDP(2, ss / 10, false, false);      // digit 2, nilai yang ditampilkan = ss dibagi 10 (nilai puluhan), dp = false, blank = false
      tampilkanDigitDP(3, ss % 10, false, false);
    } else { //jika ss dibawah 10
      // 1 digit detik saja di paling kanan
      tampilkanDigitDP(0, 0, false, true);             // digit 0, nilai0,dp=false, blank = true
      tampilkanDigitDP(1, 0, false, true);             // blank
      tampilkanDigitDP(2, 0, false, true);             // blank
      tampilkanDigitDP(3, ss, false, false);           // digit 3, nilai = ss, dp=false, blank = false
    }
  }
}

// DP di digit-1 (pemisah) dibuat KONSTAN:
// - Selalu panggil DPOnly(1) sekali per siklus (DP menyala sendiri, stabil).
// - Angka menit di digit-1 ditampilkan TANPA DP; saat blink-off, angka tidak ditampilkan.
// - Pola kedip: <10 → kedip satuan saja; >=10 → puluhan+satuan kedip.
// - Detik sama persis logikanya di digit-2/3 (tanpa DP).
void renderEditTimeBlink(uint8_t mm, uint8_t ss, uint8_t field) {
  static unsigned long lastBlink = 0; 
  static bool on = true;
  if (millis() - lastBlink >= 300) { lastBlink = millis(); on = !on; }

  bool editingMinute = (field == 0);
  bool editingSecond = (field == 1);

  // ---------- MENIT (D0 puluhan, D1 satuan) ----------
  // D0: tampil jika >=10; kedip bila editingMinute && mm>=10
  bool d0BlinkOff = (editingMinute && mm >= 10 && !on);
  if (mm >= 10 && !d0BlinkOff) tampilkanDigitDP(0, (mm / 10) % 10, false, false);
  else                          tampilkanDigitDP(0, 0, false, true);

  // D1: SELALU nyalakan DP-only dulu agar DP stabil (tidak ikut kedip)
  tampilkanDPOnly(1);

  // Lalu tampilkan angka satuan menit TANPA DP; saat blink-off (editingMinute && !on) -> jangan tampilkan angkanya
  if (!(editingMinute && !on)) {
    tampilkanDigitDP(1, mm % 10, false /* DP off di sini */, false);
  }

  // ---------- DETIK (D2 puluhan, D3 satuan) ----------
  // D2: tampil jika >=10; kedip bila editingSecond && ss>=10
  bool d2BlinkOff = (editingSecond && ss >= 10 && !on);
  if (ss >= 10 && !d2BlinkOff) tampilkanDigitDP(2, (ss / 10) % 10, false, false);
  else tampilkanDigitDP(2, 0, false, true);

  // D3: satuan detik; kedip bila editingSecond
  bool d3Blank = (editingSecond && !on);
  tampilkanDigitDP(3, ss % 10, false, d3Blank);
}


// Nyalakan hanya DP pada digit 'pos' (angka mati)
void tampilkanDPOnly(int pos) {
  for (int d = 0; d < 4; d++) digitalWrite(digit[d], HIGH);
  delayMicroseconds(50);
  for (int j = 0; j < 7; j++) digitalWrite(segment[j], LOW); // semua segmen angka off
  digitalWrite(segment[7], HIGH);                             // DP ON
  digitalWrite(digit[pos], LOW);
  delay(2);
  digitalWrite(digit[pos], HIGH);
}


//===================================BUTTON=====================================================
void handleButton() {
  int tombol = analogRead(BUTTON_ADC7);
  bool currentButton = tombol < 100;
  unsigned long now = millis();

  // Reset sinyal input
  newShortPress = false;
  newLongPress = false;
  newVeryLongPress = false;

  static bool longTriggered = false;
  static bool veryLongTriggered = false;

  // Tombol baru ditekan
  if (currentButton && !lastButtonState) { //jika kondisi tombol saat ini ditekan dan last button statenya false (baru ditekan)
    buttonPressTime = now; //simpan waktu mulai
    longTriggered = false;
    veryLongTriggered = false;
  }

  // Tombol masih ditekan
  if (currentButton && lastButtonState) { //jika tombol saat ini ditekan dan lastbuttonstatenya TRUE (artinya masih ditekan meskipun sudah pada loop berikutnya)
    unsigned long duration = now - buttonPressTime; //mulai hitung durasi penekanan

    // Hanya trigger very long press
    if (duration >= VERY_LONG_PRESS_MIN && !veryLongTriggered) { //jika durasinya lebih dari = 7000ms dan veryLongTriggered false
      newVeryLongPress = true;
      veryLongTriggered = true;
      longTriggered = true; // anggap long juga selesai
    }

    // Trigger long press hanya jika sudah melebihi batas long,
    // tapi masih di bawah batas very long
    else if (duration >= LONG_PRESS_MIN && duration < VERY_LONG_PRESS_MIN && !longTriggered) { //jika durasinya lebih dari = 1000ms serta durasi lebih kecil dari 7000 dan LongTriggered false 
      newLongPress = true;
      longTriggered = true;
    }
  }

  // Tombol dilepas
  else if (!currentButton && lastButtonState) { //jika kondisi button tidak ditekan dan lastbuttonstate true(artinya baru dilepas)
    unsigned long duration = now - buttonPressTime;

    if (duration < SHORT_PRESS_MAX) { //dan cek nilai durasi apakah lebih kecil dari 500ms
      newShortPress = true;
    }

    longTriggered = false;
    veryLongTriggered = false;
  }

  // Simpan state saat ini
  //buttonPressed = currentButton; 
  lastButtonState = currentButton; //lastbuttonstate simpan kondisi dari button sekarang
}




// === PASSWORD =======================================================================================
bool checkPassword() {
  for (int i = 0; i < 4; i++)
    if (inputPassword[i] != savedPassword[i]) return false;
  return true;
}

void savePassword() {
  for (int i = 0; i < 4; i++) EEPROM.update(i, savedPassword[i]);
}

void loadPassword() {
  for (int i = 0; i < 4; i++) savedPassword[i] = EEPROM.read(i);
  if (savedPassword[0] > 9) {
    savedPassword[0] = 1;
    savedPassword[1] = 2;
    savedPassword[2] = 3;
    savedPassword[3] = 4;
  }
  savedCountdownMenit = EEPROM.read(10);
  if (savedCountdownMenit > 60) savedCountdownMenit = 1;

  savedCountdownDetik = EEPROM.read(11);            // ← NEW
  if (savedCountdownDetik > 60) savedCountdownDetik = 0;

  countdownMenit = savedCountdownMenit;             // nilai aktif awal
  countdownDetik = savedCountdownDetik;             // nilai aktif awal
}
void isrPulse() { pulse_count++; }

//===INTERRUPT=======================================================================
void flash(){
  if (++ms >= 1000) {
    ms = 0;
    // Ambil jumlah pulsa 1 detik terakhir (singkat, tanpa digitalWrite di ISR)
    uint32_t raw = pulse_count;
    // 0.961538... ≈ 25/26, 0.869565... ≈ 20/23
    display_value  = (raw * 25u + 13u) / 26u; // ON
    display_value2 = (raw * 20u + 11u) / 23u; // OFF
    
    // display_value = pulse_count;
    // display_value2 = pulse_count;
    // display_value *= 0.9615384615; //0.9519276535
    // display_value2 *= 0.8695652174;
    pulse_count   = 0;
    // value_dirty   = true;
  }

}

// === SWITCH GLOBAL =============================================================================
void handleGlobalInterrupt() {
  if(digitalRead(PIN_OUT) == HIGH){
      if (display_value >= 1000){
    currentTrigger = true;
    } else {
      currentTrigger = false;
      }
  } else {
      if (display_value2 >= 1000){
    currentTrigger = true;
  } else {
    currentTrigger = false;
  }
  }

  // Catat bahwa trigger sudah pernah HIGH
  if (currentTrigger == true) {
    triggerPernahHIGH = true;
    digitalWrite(PIN_OUT, HIGH);

  }

  // FALLING EDGE: HIGH → LOW → hanya jika sudah pernah HIGH
  if (prevTriggerState == HIGH && currentTrigger == LOW && triggerPernahHIGH) {
    if (/*state == PULSA_DISPLAY ||*/ state == STANDBY || state == INPUT_PASSWORD || state == VERIFY_PASSWORD || state == MENU_NAVIGATION || state == MENU_PASSWORD_CHANGE || state == PASSWORD_CHANGED || state == MENU_CT_SETUP || state == MENU_CT_PICK_TIME) {
      // Tentukan countdownMenit berdasarkan preset switch
      byte totalMenit = 0;
      if (digitalRead(SWITCH1) == LOW) totalMenit += 1;
      if (digitalRead(SWITCH2) == LOW) totalMenit += 2;
      if (digitalRead(SWITCH3) == LOW) totalMenit += 3;
      if (digitalRead(SWITCH4) == LOW) totalMenit += 4;

      // Jika tidak ada switch yang aktif, gunakan nilai yang disimpan
      if (totalMenit == 0) {
        countdownMenit = savedCountdownMenit;
        countdownDetik = savedCountdownDetik;   // pakai default simpanan
      } else {
        countdownMenit = totalMenit;
        countdownDetik = 0; 
      }

      baruMasukMenuPassword = true;
      baruMasukCTSetup = true;
      countdownStart = millis();
      state = COUNTDOWN_RUNNING;
      triggerPernahHIGH = false;  // Reset flag agar harus HIGH lagi nanti
    }
  }

  // RISING EDGE: LOW → HIGH → batalkan countdown
  // else if (prevTriggerState == LOW && currentTrigger == HIGH) (program sebelumnya)
  else if (prevTriggerState == LOW && currentTrigger == HIGH) {
    if (state == COUNTDOWN_RUNNING) {
      state = STANDBY;
      //byte totalMenit = 0;
    }
  }

  prevTriggerState = currentTrigger;
}
//pppppppppp

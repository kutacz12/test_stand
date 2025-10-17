#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

/*
  Teensy 4.0 + Waveshare High-Precision AD HAT (ADS1263)
  Rocket test stand logger — filtered logging @ 100 Hz (every 10 ms)

  - ADC1 ~1 kSPS, SINC3, PGA=32, REF = AVDD-AVSS
  - IIR low-pass ~20 Hz @ 1 kSPS
  - Log filtered grams @ 100 Hz to SD (CSV), flush batched (every 200 lines)
  - No calibration UI here. Either set CALIBRATION_FACTOR constant,
    or put a file "calib.txt" on SD with a single number (g/LSB).
*/

// =================== PINOUT ===================
#define SD_CS_PIN         10   // SD Card CS
#define BUZZER_PIN         9   // buzzer
#define STOP_BUTTON_PIN    8   // STOP button (to GND, INPUT_PULLUP)

#define ADS_CS_PIN         7   // ADS1263 CS
#define ADS_DRDY_PIN       2   // ADS1263 DRDY (FALLING)
#define ADS_RST_PIN        3   // ADS1263 RESET (active low)
// Teensy 4.0 SPI: SCK=13, MOSI=11, MISO=12

// =================== ADS1263 COMMANDS ===================
#define ADS_CMD_RESET      0x06
#define ADS_CMD_START1     0x08
#define ADS_CMD_STOP1      0x0A
#define ADS_CMD_RREG       0x20
#define ADS_CMD_WREG       0x40
#define ADS_CMD_SDATAC1    0x11
#define ADS_CMD_RDATAC1    0x10

// =================== ADS1263 REGISTERS ===================
#define REG_ID          0x00
#define REG_POWER       0x01
#define REG_INTERFACE   0x02
#define REG_MODE0       0x03
#define REG_MODE1       0x04
#define REG_MODE2       0x05
#define REG_INPMUX      0x06
#define REG_REFMUX      0x0F

// =================== GLOBALS ===================
static const SPISettings ADS_SPI(2000000, MSBFIRST, SPI_MODE1);

static volatile bool adsDataReady = false;
File dataFile;
bool recording = true;

// --- Calibration & tare ---
float      CALIBRATION_FACTOR = 1.0f;   // [g/LSB] — overriden from calib.txt if present
long long  tareOffset = 0;

// --- Logging/filters ---
const uint16_t ADC_FS_HZ           = 1000; // target ADC1 sampling ~1 kSPS
const uint16_t LOG_RATE_HZ         = 100;  // write every 10 ms
const uint16_t SAMPLES_PER_LOG     = ADC_FS_HZ / LOG_RATE_HZ; // 10
const uint16_t FLUSH_EVERY         = 200;  // batch flush

// IIR low-pass (~20 Hz cutoff @ 1 kHz)
const float LPF_CUTOFF_HZ          = 20.0f;
const float LPF_ALPHA              = (2.0f * 3.14159265f * LPF_CUTOFF_HZ) / (float)ADC_FS_HZ; // ~0.126
volatile float lpf_state           = 0.0f;

// Prototypes
bool readOneNet(int32_t &net);
void doTare(uint16_t N=400);
void ads_apply_config_1kHz();
void ads_dump_registers();
bool loadCalibrationFromSD();

// Status flag: we disable STATUS to keep 4-byte frames (faster)
bool g_statusEnabled = false;

// =================== LOW-LEVEL ADS I/O ===================
inline void ads_cs_low()  { digitalWrite(ADS_CS_PIN, LOW); }
inline void ads_cs_high() { digitalWrite(ADS_CS_PIN, HIGH); }
inline void ads_spi_begin() { SPI.beginTransaction(ADS_SPI); }
inline void ads_spi_end()   { SPI.endTransaction(); }
uint8_t ads_transfer(uint8_t b) { return SPI.transfer(b); }

void ads_writeCommand(uint8_t cmd) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(cmd);
  ads_cs_high(); ads_spi_end();
}

void ads_writeRegister(uint8_t addr, uint8_t value) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(ADS_CMD_WREG | (addr & 0x1F));
  ads_transfer(0x00); // 1 byte
  ads_transfer(value);
  ads_cs_high(); ads_spi_end();
}

uint8_t ads_readRegister(uint8_t addr) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(ADS_CMD_RREG | (addr & 0x1F));
  ads_transfer(0x00); // 1 byte
  uint8_t v = ads_transfer(0x00);
  ads_cs_high(); ads_spi_end();
  return v;
}

void drdy_isr() { adsDataReady = true; }

void ads_reset() {
  digitalWrite(ADS_RST_PIN, LOW); delay(2);
  digitalWrite(ADS_RST_PIN, HIGH); delay(5);
  ads_writeCommand(ADS_CMD_RESET); delay(5);
}

// Continuous read (RDATAC), STATUS disabled → read 4 data bytes MSB..LSB
bool ads_readData32_cont_raw(int32_t &value) {
  if (digitalRead(ADS_DRDY_PIN) == HIGH) return false; // no new data
  ads_spi_begin(); ads_cs_low();

  // If STATUS were enabled, we'd first read+discard 1 byte here.
  uint8_t b0 = ads_transfer(0x00);
  uint8_t b1 = ads_transfer(0x00);
  uint8_t b2 = ads_transfer(0x00);
  uint8_t b3 = ads_transfer(0x00);

  ads_cs_high(); ads_spi_end();

  uint32_t raw = ((uint32_t)b0<<24)|((uint32_t)b1<<16)|((uint32_t)b2<<8)|b3;
  value = (int32_t)raw; // 2's complement
  return true;
}

// =================== CONFIG (1 kSPS, SINC3, PGA=32, REF=AVDD-AVSS) ===================
void ads_apply_config_1kHz() {
  ads_writeCommand(ADS_CMD_SDATAC1);
  ads_writeCommand(ADS_CMD_STOP1);

  // Interface: STATUS off, 32-bit data
  ads_writeRegister(REG_INTERFACE, 0b00001010); // STATUS=0, CRC=0, 32-bit=1
  g_statusEnabled = false;

  // Filters & rate: SINC3, ~1 kSPS; PGA=32 (MAX for Waveshare HAT)
  ads_writeRegister(REG_MODE0,     0b00000000); // SINC3
  ads_writeRegister(REG_MODE1,     0b00110010); // PGA=32, BOOST x2
  ads_writeRegister(REG_MODE2,     0b00101001); // ~1 kSPS

  // Input MUX: AIN0 / AIN1 (Sig+ / Sig−)
  ads_writeRegister(REG_INPMUX,    (0b00000<<4) | 0b00001); // AIN0/AIN1

  // Reference: AVDD-AVSS (Waveshare default)
  ads_writeRegister(REG_REFMUX,    0b00000000); // AVDD-AVSS

  ads_writeCommand(ADS_CMD_RDATAC1);
  ads_writeCommand(ADS_CMD_START1);
}

void ads_dump_registers() {
  uint8_t id  = ads_readRegister(REG_ID);
  uint8_t pwr = ads_readRegister(REG_POWER);
  uint8_t ifc = ads_readRegister(REG_INTERFACE);
  uint8_t m0  = ads_readRegister(REG_MODE0);
  uint8_t m1  = ads_readRegister(REG_MODE1);
  uint8_t m2  = ads_readRegister(REG_MODE2);
  uint8_t mux = ads_readRegister(REG_INPMUX);
  uint8_t ref = ads_readRegister(REG_REFMUX);

  Serial.println(F("=== ADS reg dump ==="));
  Serial.print(F("ID=0x"));     Serial.println(id,  HEX);
  Serial.print(F("POWER=0x"));  Serial.println(pwr, HEX);
  Serial.print(F("INTERFACE=0x"));  Serial.println(ifc, HEX);
  Serial.print(F("MODE0=0x"));  Serial.println(m0,  HEX);
  Serial.print(F("MODE1=0x"));  Serial.println(m1,  HEX);
  Serial.print(F("MODE2=0x"));  Serial.println(m2,  HEX);
  Serial.print(F("INPMUX=0x")); Serial.println(mux, HEX);
  Serial.print(F("REFMUX=0x")); Serial.println(ref, HEX);
  Serial.println(F("===================="));
}

// =================== APP HELPERS ===================
void startSignal(){ for(int i=0;i<2;i++){digitalWrite(BUZZER_PIN,HIGH);delay(150);digitalWrite(BUZZER_PIN,LOW);delay(150);} }
void stopSignal(){ digitalWrite(BUZZER_PIN,HIGH);delay(600);digitalWrite(BUZZER_PIN,LOW); }
void errorSignal(){ for(int i=0;i<3;i++){digitalWrite(BUZZER_PIN,HIGH);delay(250);digitalWrite(BUZZER_PIN,LOW);delay(250);} }

bool readOneNet(int32_t &net){
  int32_t raw;
  if(!ads_readData32_cont_raw(raw)) return false;
  net = (int32_t)((long long)raw - tareOffset);
  return true;
}

// Tare (avg N raw samples, before filtering)
void doTare(uint16_t N){
  Serial.println(F("Taring..."));
  delay(20);
  long long acc=0; int32_t raw;
  for(uint16_t i=0;i<N;i++){
    while(!ads_readData32_cont_raw(raw)){}
    acc += (long long)raw;
  }
  tareOffset = acc / N;
  lpf_state = 0.0f; // reset filter
  Serial.print(F("Tare complete. tareOffset = "));
  Serial.println((long long)tareOffset);
}

bool loadCalibrationFromSD(){
  File f = SD.open("calib.txt", FILE_READ);
  if(!f) return false;
  String s = f.readStringUntil('\n');
  f.close();
  s.trim();
  if(s.length()==0) return false;
  CALIBRATION_FACTOR = s.toFloat();
  return (CALIBRATION_FACTOR != 0.0f && isfinite(CALIBRATION_FACTOR));
}

// =================== SETUP ===================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RST_PIN, OUTPUT);
  pinMode(ADS_DRDY_PIN, INPUT_PULLUP); // DRDY active low
  ads_cs_high();

  // SD CS high before SPI init
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.begin(500000); delay(50);
  Serial.println(F("Starting (Logger)..."));

  SPI.begin();

  // ADS init
  ads_reset();
  ads_apply_config_1kHz();
  ads_dump_registers();
  attachInterrupt(digitalPinToInterrupt(ADS_DRDY_PIN), drdy_isr, FALLING);

  // SD init
  if (!SD.begin(SD_CS_PIN)) { Serial.println(F("SD card initialization failed!")); errorSignal(); while(1){} }
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (!dataFile) { Serial.println(F("Error opening file!")); errorSignal(); while(1){} }
  dataFile.println("Time_ms,Weight_g"); dataFile.flush();

  // Try load calibration from SD
  if(loadCalibrationFromSD()){
    Serial.print(F("Loaded CALIBRATION_FACTOR from calib.txt: "));
    Serial.println(CALIBRATION_FACTOR, 9);
  } else {
    Serial.print(F("Using built-in CALIBRATION_FACTOR: "));
    Serial.println(CALIBRATION_FACTOR, 9);
  }

  // Quick tare
  doTare(400);
  startSignal();

  Serial.println(F("Komendy:"));
  Serial.println(F("  t   -> tare"));
}

// =================== LOOP ===================
void loop() {
  static uint16_t sampleCount = 0;
  static uint16_t flushCnt = 0;

  // Acquire @ ~1 kSPS (via RDY ISR flag)
  if (adsDataReady) {
    adsDataReady = false;

    int32_t netLSB;
    if (readOneNet(netLSB)) {
      // IIR low-pass (first order): y += a*(x - y)
      float x = (float)netLSB;
      lpf_state += LPF_ALPHA * (x - lpf_state);

      // Downsample/log every 10 samples -> 100 Hz
      if (++sampleCount >= SAMPLES_PER_LOG) {
        sampleCount = 0;

        float grams = lpf_state * CALIBRATION_FACTOR;
        uint32_t now = millis();

        dataFile.print(now);
        dataFile.print(",");
        dataFile.println(grams, 3);

        if (++flushCnt >= FLUSH_EVERY) { dataFile.flush(); flushCnt = 0; }

         //Serial peek
         //Serial.print(F("t=")); Serial.print(now); Serial.print(F("ms  W=")); Serial.println(grams, 3);
      }
    }
  }

  // STOP button
  if (recording && digitalRead(STOP_BUTTON_PIN) == LOW) {
    Serial.println(F("STOP pressed. Closing file."));
    dataFile.close(); recording = false; stopSignal();
  }

  // Simple commands
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd == "t") doTare(400);
  }
}

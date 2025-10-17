#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ===== PINY / SPRZĘT jak w loggerze =====
#define SD_CS_PIN         10
#define ADS_CS_PIN         7
#define ADS_DRDY_PIN       2
#define ADS_RST_PIN        3

// ===== ADS =====
#define ADS_CMD_RESET      0x06
#define ADS_CMD_START1     0x08
#define ADS_CMD_STOP1      0x0A
#define ADS_CMD_RREG       0x20
#define ADS_CMD_WREG       0x40
#define ADS_CMD_SDATAC1    0x11
#define ADS_CMD_RDATAC1    0x10

#define REG_ID          0x00
#define REG_POWER       0x01
#define REG_INTERFACE   0x02
#define REG_MODE0       0x03
#define REG_MODE1       0x04
#define REG_MODE2       0x05
#define REG_INPMUX      0x06
#define REG_REFMUX      0x0F

static const SPISettings ADS_SPI(2000000, MSBFIRST, SPI_MODE1);
static volatile bool adsDataReady = false;

inline void ads_cs_low()  { digitalWrite(ADS_CS_PIN, LOW); }
inline void ads_cs_high() { digitalWrite(ADS_CS_PIN, HIGH); }
inline void ads_spi_begin() { SPI.beginTransaction(ADS_SPI); }
inline void ads_spi_end()   { SPI.endTransaction(); }
uint8_t ads_transfer(uint8_t b) { return SPI.transfer(b); }

void drdy_isr() { adsDataReady = true; }

void ads_writeCommand(uint8_t cmd) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(cmd);
  ads_cs_high(); ads_spi_end();
}
void ads_writeRegister(uint8_t addr, uint8_t value) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(ADS_CMD_WREG | (addr & 0x1F));
  ads_transfer(0x00);
  ads_transfer(value);
  ads_cs_high(); ads_spi_end();
}
uint8_t ads_readRegister(uint8_t addr) {
  ads_spi_begin(); ads_cs_low();
  ads_transfer(ADS_CMD_RREG | (addr & 0x1F));
  ads_transfer(0x00);
  uint8_t v = ads_transfer(0x00);
  ads_cs_high(); ads_spi_end();
  return v;
}
void ads_reset() {
  digitalWrite(ADS_RST_PIN, LOW); delay(2);
  digitalWrite(ADS_RST_PIN, HIGH); delay(5);
  ads_writeCommand(ADS_CMD_RESET); delay(5);
}
bool ads_readData32_cont_raw(int32_t &value) {
  if (digitalRead(ADS_DRDY_PIN) == HIGH) return false;
  ads_spi_begin(); ads_cs_low();
  uint8_t b0 = ads_transfer(0x00);
  uint8_t b1 = ads_transfer(0x00);
  uint8_t b2 = ads_transfer(0x00);
  uint8_t b3 = ads_transfer(0x00);
  ads_cs_high(); ads_spi_end();
  uint32_t raw = ((uint32_t)b0<<24)|((uint32_t)b1<<16)|((uint32_t)b2<<8)|b3;
  value = (int32_t)raw;
  return true;
}

// Konfiguracja na 1 kSPS (szybka kalibracja), PGA=32, REF=AVDD-AVSS
void ads_apply_config_1kHz() {
  ads_writeCommand(ADS_CMD_SDATAC1);
  ads_writeCommand(ADS_CMD_STOP1);
  ads_writeRegister(REG_INTERFACE, 0b00001010); // STATUS=0, 32-bit
  ads_writeRegister(REG_MODE0,     0b00000000); // SINC3
  ads_writeRegister(REG_MODE1,     0b00110010); // PGA=32
  ads_writeRegister(REG_MODE2,     0b00101001); // ~1 kSPS
  ads_writeRegister(REG_INPMUX,    (0b00000<<4) | 0b00001); // AIN0/AIN1
  ads_writeRegister(REG_REFMUX,    0b00000000); // AVDD-AVSS
  ads_writeCommand(ADS_CMD_RDATAC1);
  ads_writeCommand(ADS_CMD_START1);
}

long long tareOffset = 0;

void doTare(uint16_t N=400){
  Serial.println(F("Taring..."));
  delay(20);
  long long acc=0; int32_t raw;
  for(uint16_t i=0;i<N;i++){
    while(!ads_readData32_cont_raw(raw)){}
    acc += (long long)raw;
  }
  tareOffset = acc / N;
  Serial.print(F("Tare complete. tareOffset = "));
  Serial.println((long long)tareOffset);
}

bool readOneNet(int32_t &net){
  int32_t raw;
  if(!ads_readData32_cont_raw(raw)) return false;
  net = (int32_t)((long long)raw - tareOffset);
  return true;
}

long long averageNetSamples(uint32_t samples) {
  long long acc = 0;
  int32_t net;
  for (uint32_t i=0; i<samples; i++){
    while(!readOneNet(net)){}
    acc += (long long)net;
  }
  return acc / (long long)samples;
}

bool saveCalibrationToSD(float factor){
  File f = SD.open("calib.txt", FILE_WRITE);
  if(!f) return false;
  f.seek(0); // overwrite
  f.println(factor, 9);
  f.flush();
  f.close();
  return true;
}

void setup() {
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RST_PIN, OUTPUT);
  pinMode(ADS_DRDY_PIN, INPUT_PULLUP);
  ads_cs_high();

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.begin(500000); delay(50);
  Serial.println(F("Starting (Calibration)..."));

  SPI.begin();
  ads_reset();
  ads_apply_config_1kHz();
  attachInterrupt(digitalPinToInterrupt(ADS_DRDY_PIN), drdy_isr, FALLING);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD init failed — calib.txt won't be saved automatically."));
  } else {
    Serial.println(F("SD OK — wynik zapisze do calib.txt"));
  }

  doTare(600);

  Serial.println(F("Instrukcja:"));
  Serial.println(F("  t           -> tare"));
  Serial.println(F("  m <gramy>   -> połóż wzorzec, wpisz np. 'm 10000'"));
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line == "t") {
    doTare(600);
    return;
  }
  if (line.startsWith("m ")) {
    float mass_g = line.substring(2).toFloat();
    if (mass_g <= 0) { Serial.println(F("Blad: masa > 0")); return; }

    Serial.println(F("Zbieram ~3000 probek (~3 s)..."));
    long long lsb_avg = averageNetSamples(3000);
    Serial.print(F("Srednie LSB po tare: ")); Serial.println((long long)lsb_avg);

    if (lsb_avg == 0) { Serial.println(F("Wynik 0 — sprawdz polaczenia.")); return; }

    float factor = mass_g / (float)lsb_avg; // [g/LSB]
    Serial.print(F("CALIBRATION_FACTOR [g/LSB] = "));
    Serial.println(factor, 9);

    if (SD.begin(SD_CS_PIN) && saveCalibrationToSD(factor)) {
      Serial.println(F("Zapisano do calib.txt (pierwsza linia)."));
    } else {
      Serial.println(F("Nie zapisano do calib.txt — wpisz recznie w loggerze lub wrzuc plik na SD."));
    }
    return;
  }

  Serial.println(F("Nieznana komenda. Uzyj: t  lub  m <gramy>"));
}

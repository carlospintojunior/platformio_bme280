#include <Arduino.h>
#include <Wire.h>

// Endereço I2C do BME280 (pode ser 0x76 ou 0x77)
#define BME280_I2C_ADDR  0x76

// Registradores
#define REG_CALIB_00     0x88
#define REG_CALIB_26     0xE1
#define REG_CTRL_HUM     0xF2
#define REG_STATUS       0xF3
#define REG_CTRL_MEAS    0xF4
#define REG_CONFIG       0xF5
#define REG_DATA         0xF7

// Variáveis de calibração
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t  dig_H1;
int16_t  dig_H2, dig_H3;
int16_t  dig_H4, dig_H5;
int8_t   dig_H6;

// Variável interna para compensação de temperatura
int32_t t_fine;

// Leitura de múltiplos bytes
void readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(BME280_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_I2C_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

// Escrita de um byte
void writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BME280_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Carrega todos os coeficientes de calibração do sensor
void readCalibration() {
  uint8_t buf1[24];
  readBytes(REG_CALIB_00, buf1, 24);

  dig_T1 = (buf1[1] << 8) | buf1[0];
  dig_T2 = (buf1[3] << 8) | buf1[2];
  dig_T3 = (buf1[5] << 8) | buf1[4];

  dig_P1 = (buf1[7] << 8) | buf1[6];
  dig_P2 = (buf1[9] << 8) | buf1[8];
  dig_P3 = (buf1[11] << 8) | buf1[10];
  dig_P4 = (buf1[13] << 8) | buf1[12];
  dig_P5 = (buf1[15] << 8) | buf1[14];
  dig_P6 = (buf1[17] << 8) | buf1[16];
  dig_P7 = (buf1[19] << 8) | buf1[18];
  dig_P8 = (buf1[21] << 8) | buf1[20];
  dig_P9 = (buf1[23] << 8) | buf1[22];

  // H1
  readBytes(0xA1, &dig_H1, 1);

  uint8_t buf2[7];
  readBytes(REG_CALIB_26, buf2, 7);
  dig_H2 = (buf2[1] << 8) | buf2[0];
  dig_H3 = buf2[2];
  dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F);
  dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
  dig_H6 = (int8_t)buf2[6];
}

// Compensação de temperatura (ºC *100)
int32_t compensateTemperature(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                  ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

// Compensação de pressão (Pa)
uint32_t compensatePressure(int32_t adc_P) {
  int64_t var1 = (int64_t)t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)dig_P1)) >> 33;
  if (var1 == 0) return 0; // evita divisão por zero
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

// compensação em float para pressão, retorna Pa
float compensatePressureFloat(int32_t adc_P) {
  float var1 = ((float)t_fine / 2.0f) - 64000.0f;
  float var2 = var1 * var1 * (float)dig_P6 / 32768.0f;
  var2  = var2 + var1 * (float)dig_P5 * 2.0f;
  var2  = (var2 / 4.0f) + ((float)dig_P4 * 65536.0f);
  var1  = (((float)dig_P3 * var1 * var1) / 524288.0f + ((float)dig_P2 * var1)) / 524288.0f;
  var1  = (1.0f + var1 / 32768.0f) * (float)dig_P1;
  if (fabs(var1) < 0.0001f) return 0;  // evita divisão por zero

  float p = 1048576.0f - (float)adc_P;
  p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
  var1 = ((float)dig_P9 * p * p) / 2147483648.0f;
  var2 = (p * (float)dig_P8) / 32768.0f;
  p    = p + (var1 + var2 + (float)dig_P7) / 16.0f;

  return p;
}

// Compensação de umidade (RH *1024)
uint32_t compensateHumidity(int32_t adc_H) {
  int32_t v_x1 = t_fine - ((int32_t)76800);
  v_x1 = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v_x1)) +
         ((int32_t)16384)) >> 15) *
         (((((((v_x1 * (int32_t)dig_H6) >> 10) * (((v_x1 * (int32_t)dig_H3) >> 11) + 
         ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * (int32_t)dig_H2 + 8192) >> 14));
  v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (uint32_t)(v_x1 >> 12);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA, SCL no ESP32 (ajuste se necessário)
  delay(100);

  // Reset opcional (escreve 0xB6 no reg 0xE0)
  writeByte(0xE0, 0xB6);
  delay(100);

  readCalibration();

  // Configura oversampling:
  // Humidity x1
  writeByte(REG_CTRL_HUM, 0x01);
  // Temp x1, Press x1, Forced mode
  writeByte(REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x01);
  // Config: t_sb=1000ms, filtro off, SPI desativado
  writeByte(REG_CONFIG, (0x05 << 5) | (0 << 2) | 0);
}

void loop() {
  // Modo forçado: toda leitura precisa reconfigurar CTRL_MEAS
  writeByte(REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x01);
  delay(100);  // aguarda conversão

  uint8_t data[8];
  readBytes(REG_DATA, data, 8);

  int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
  int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
  int32_t adc_H = ((uint32_t)data[6] << 8)  | data[7];

  int32_t  T  = compensateTemperature(adc_T);      // em centésimos de °C
  uint32_t P  = compensatePressure(adc_P);         // em Pa
  uint32_t H  = compensateHumidity(adc_H);         // em milésimos de %RH
  float P_pa  = compensatePressureFloat(adc_P);
  float P_hpa = P_pa / 100.0f;

  Serial.print("T = "); Serial.print(T / 100.0); Serial.print(" °C\t");
  // Serial.print("P = "); Serial.print(P / 100.0); Serial.print(" hPa\t");
  Serial.print("P = "); Serial.print(P_hpa); Serial.println(" hPa");
  Serial.print("H = "); Serial.print(H / 1024.0); Serial.println(" %");

  delay(1000);
}

#include <SparkFunBME280.h>

#include <Wire.h>
#include <stdint.h>
#include <arduino.h>
#include <math.h>

#define I2C_ADDRES_ACCEL 0x1D     //加速度センサー(ADXL345)
#define I2C_ADDRES_COMPASS 0x1E   //地磁器センサー(HMC5883L)
#define I2C_ADDRES_AIRSTATE 0x77  //温度、湿度、大気圧センサー(BME280)

#define COMPASS_DATA_X 0x03
#define COMPASS_DATA_Y 0x07
#define COMPASS_DATA_Z 0x05

#define ByteToInt(under, upper) (upper<<8 | under) //8bitを16bitに変換する

enum{X, Y, Z};
enum{TEMP, PRESS, HUM};

int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;


void setup()
{
  Wire.begin();
  writeI2C(I2C_ADDRES_ACCEL, 0x31, 0x00);
  writeI2C(I2C_ADDRES_ACCEL, 0x2D, 0x08);

  writeI2C(I2C_ADDRES_COMPASS, 0x02, 0x00);  //連続測定モード

  writeI2C(I2C_ADDRES_AIRSTATE, 0xF2, 0x01); //湿度のオーバーサンプリング設定
  writeI2C(I2C_ADDRES_AIRSTATE, 0xF4, 0x27); //温度、気圧のオーバーサンプリング設定
  writeI2C(I2C_ADDRES_AIRSTATE, 0xF5, 0xA0); //サンプリング周期1000ms/IIRフィルタOFF
  
  readTrim();   //BME280のキャリブレーションデータを保存
  
  Serial.begin(9600);
  Serial.println();
  Serial.println("I2C SensorTest");
}


void loop()
{
  float accel_data[3];
  float airstate_data[3];
  int compass_data[3];
  float deg;

  getAirstate(airstate_data);
  getCompass(compass_data);
  deg = getDegree(compass_data);

  //  Serial.println(deg);
/*
  Serial.print(compass_data[X]);
  Serial.print(" ");
  Serial.print(compass_data[Y]);
  Serial.print(" ");
  Serial.println(compass_data[Z]);
  */

  Serial.print(airstate_data[TEMP]);
  Serial.print("℃　");
  Serial.print(airstate_data[PRESS]);
  Serial.print("hPa　");
  Serial.print(airstate_data[HUM]);
  Serial.println("%");

  /*
    Serial.println(accel_data[0]);
    Serial.println(accel_data[1]);
    Serial.println(accel_data[2]);
    Serial.println("");
  */
  delay(1000);
}

/*********************************************************************/
/*     HMC5883                                                       */
/*********************************************************************/
void getCompass(int compass_data[])
{
  byte data[6];

  //センサー値をレジスタから取得
  readI2C(I2C_ADDRES_COMPASS, 0x03, 6, data);

  //取得したデータを16bitのデータにする
  compass_data[X] = ByteToInt(data[0], data[1]);
  compass_data[Z] = ByteToInt(data[2], data[3]);
  compass_data[Y] = ByteToInt(data[4], data[5]);
}

float getDegree(int compass_data[])
{
  float deg;

  deg = atan2(compass_data[Y], compass_data[X]);
  if(deg < 0) deg += 2*PI;
  if(deg > 2*PI) deg -= 2*PI;
  deg *= 180/M_PI;
  if(deg > 360.0) deg = deg - 360.0;
  return deg; 
}


/*********************************************************************/
/*     ADXL345                                                       */
/*********************************************************************/
void getAccel(float accel_data[])
{
  byte data[6];
  float raw_x, raw_y, raw_z;
  static float low_x = 0, low_y = 0, low_z = 0;

  //センサー値をレジスタから取得
  readI2C(I2C_ADDRES_ACCEL, 0x32, 6, data);

  //取得したデータを16bitのデータにする
  raw_x = ByteToInt(data[0], data[1]);
  raw_y = ByteToInt(data[2], data[3]);
  raw_z = ByteToInt(data[4], data[5]);

  /* ローパスフィルタ */
  low_x = 0.9 * raw_x + 0.1 * low_x;
  low_y = 0.9 * raw_y + 0.1 * low_y;
  low_z = 0.9 * raw_z + 0.1 * low_z;

  //実際の加速度にする
  accel_data[X] = low_x / 256;
  accel_data[Y] = low_y / 256;
  accel_data[Z] = low_z / 256;
}


/*********************************************************************/
/*     BME280                                                        */
/*********************************************************************/
void getAirstate(float airstate_data[])
{
  uint32_t hum_raw, temp_raw, pres_raw;
  float temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  long temp_cal;
  uint32_t press_cal, hum_cal;
  
  readData(&pres_raw, &temp_raw, &hum_raw);

  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  
  temp_act = (float)temp_cal / 100.0;
  press_act = (float)press_cal / 100.0;
  hum_act = (float)hum_cal / 1024.0;

  airstate_data[TEMP] = temp_act;
  airstate_data[PRESS] = press_act;
  airstate_data[HUM] = hum_act;
}

void readTrim()
{
  byte data[32];
  int i = 0;

  //補正値取得
  readI2C(I2C_ADDRES_AIRSTATE, 0x88, 24, &data[i]);
  i+=24;
  readI2C(I2C_ADDRES_AIRSTATE, 0xA1, 1, &data[i]);
  i++;
  readI2C(I2C_ADDRES_AIRSTATE, 0xE1, 7, &data[i]);
  i+=7;
  dig_T1 = ByteToInt(data[0], data[1]);
  dig_T2 = ByteToInt(data[2], data[3]);
  dig_T3 = ByteToInt(data[4], data[5]);

  dig_P1 = ByteToInt(data[6], data[1]);
  dig_P2 = ByteToInt(data[8], data[1]);
  dig_P3 = ByteToInt(data[10], data[11]);
  dig_P4 = ByteToInt(data[12], data[13]);
  dig_P5 = ByteToInt(data[14], data[15]);
  dig_P6 = ByteToInt(data[16], data[17]);
  dig_P7 = ByteToInt(data[18], data[19]);
  dig_P8 = ByteToInt(data[20], data[21]);
  dig_P9 = ByteToInt(data[22], data[23]);

  dig_H1 = data[24];
  dig_H2 = ByteToInt(data[25], data[26]);
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}

void readData(uint32_t *pres_raw, uint32_t *temp_raw, uint32_t *hum_raw)
{
  byte data[8];
  uint32_t ui32data[8];
  
  readI2C(I2C_ADDRES_AIRSTATE, 0xF7, 8, data);  //生データの取得
  for(int i=0; i<8; i++) ui32data[i] = data[i]; //型変換
  *pres_raw = (ui32data[0] << 12) | (ui32data[1] << 4) | (ui32data[2] >> 4);
  *temp_raw = (ui32data[3] << 12) | (ui32data[4] << 4) | (ui32data[5] >> 4);
  *hum_raw  = (ui32data[6] << 8) | ui32data[7];
}


int32_t calibration_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
  var2 = (((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

uint32_t calibration_P(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t P;
  
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
  if (var1 == 0) return 0;
  P = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000) P = (P << 1) / ((uint32_t) var1);
  else P = (P / (uint32_t)var1) * 2;
  var1 = (((int32_t)dig_P9) * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(P >> 2)) * ((int32_t)dig_P8)) >> 13;
  P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

uint32_t calibration_H(int32_t adc_H)
{
  int32_t v_x1;

  v_x1 = (t_fine - ((int32_t)76800));
  
  v_x1 = ((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v_x1)) + 16384) >> 15) * 
          (((((((v_x1 * ((int32_t)dig_H6)) >> 10) * (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t)2097152)) * ((int32_t) dig_H2) + 8192) >> 14);
          
  v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
  v_x1 = v_x1 < 0 ? 0 : v_x1;
  v_x1 = v_x1 > 419430400 ? 419430400 : v_x1;
  return (uint32_t)(v_x1 >> 12);
}


/*********************************************************************/
/*     I2Cread/write関数                                             */
/*********************************************************************/
void writeI2C(byte I2C_addres, byte register_addres, byte value)
{
  Wire.beginTransmission(I2C_addres);
  Wire.write(register_addres);
  Wire.write(value);
  Wire.endTransmission();
}

void readI2C(byte I2C_addres, byte register_addres, byte byte_num, byte* ret_ary)
{
  int i;
  Wire.beginTransmission(I2C_addres);
  Wire.write(register_addres);
  Wire.endTransmission();
  Wire.requestFrom(I2C_addres, byte_num);
  i = 0;
  while (Wire.available()) {
    ret_ary[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}


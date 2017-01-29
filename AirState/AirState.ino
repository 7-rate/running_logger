/*********************************************************************/
/* include                                                           */
/*********************************************************************/
#include <LiquidCrystal.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <stdint.h>
#include <arduino.h>
#include <math.h>

/*********************************************************************/
/* Constant Definition                                               */
/*********************************************************************/
#define I2C_ADDRES_ACCEL	0x1D	// ADXL345
#define I2C_ADDRES_COMPASS	0x1E	// HMC5883L
#define I2C_ADDRES_AIRSTATE	0x76	// BME280

enum{TEMP, PRESS, HUM};

/*********************************************************************/
/* Global variable                                                   */
/*********************************************************************/
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

/*********************************************************************/
/* macro                                                             */
/*********************************************************************/
#define ByteToInt(under, upper) (upper<<8 | under)


/*********************************************************************/
/* LCDpin                                                            */
/*********************************************************************/
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


/*********************************************************************/
/* setup                                                             */
/*********************************************************************/
void setup()
{
	Wire.begin();
	writeI2C(I2C_ADDRES_AIRSTATE, 0xF2, 0x01); // Humidity Setting
	writeI2C(I2C_ADDRES_AIRSTATE, 0xF4, 0x27); // Temp,Press over sampling
	writeI2C(I2C_ADDRES_AIRSTATE, 0xF5, 0xA0); // sampling cycle time 1000ms IIR filterOFF
	readTrim();   // save calibration data
	Serial.begin(9600);
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
}

/*********************************************************************/
/* main                                                              */
/*********************************************************************/
void loop()
{
	float airStateData[3];
	
	getAirState(airStateData);
	lcd.setCursor(0, 0);
	lcd.print("Temp :");
	lcd.print(airStateData[TEMP]);
	lcd.setCursor(0, 1);
	lcd.print("Humi :");
	lcd.print(airStateData[HUM]);
	
	delay(1000);
}


/*********************************************************************/
/*     BME280                                                        */
/*********************************************************************/
void getAirState(float airState[])
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

	airState[TEMP] = temp_act;
	airState[PRESS] = press_act;
	airState[HUM] = hum_act;
}

void readTrim()
{
	byte data[32];
	int i = 0;

	readI2C(I2C_ADDRES_AIRSTATE, 0x88, 24, &data[i]);
	i+= 24;
	readI2C(I2C_ADDRES_AIRSTATE, 0xA1, 1, &data[i]);
	i++;
	readI2C(I2C_ADDRES_AIRSTATE, 0xE1, 7, &data[i]);
	i+= 7;
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

	readI2C(I2C_ADDRES_AIRSTATE, 0xF7, 8, data);
	for(int i=0; i<8; i++) { 
		ui32data[i] = data[i];
	}
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

	v_x1 = t_fine - ((int32_t)76800);

	v_x1 = ((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v_x1)) + 16384) >> 15) * 
        (((((((v_x1 * ((int32_t)dig_H6)) >> 10) * (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t)2097152)) * ((int32_t) dig_H2) + 8192) >> 14);

	v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
	v_x1 = v_x1 < 0 ? 0 : v_x1;
	v_x1 = v_x1 > 419430400 ? 419430400 : v_x1;
	return (uint32_t)(v_x1 >> 12);
}

/*********************************************************************/
/* I2Cread/write                                                     */
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
}


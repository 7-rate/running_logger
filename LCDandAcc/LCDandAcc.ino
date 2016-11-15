//#include <SparkFunBME280.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <stdint.h>
#include <arduino.h>
#include <math.h>
#include <MsTimer2.h>
#include <EEPROM.h>

#define I2C_ADDRES_ACCEL 0x1D		//加速度センサー(ADXL345)
#define I2C_ADDRES_COMPASS 0x1E		//地磁器センサー(HMC5883L)
#define I2C_ADDRES_AIRSTATE 0x77	//温度、湿度、大気圧センサー(BME280)

#define COMPASS_DATA_X 0x03
#define COMPASS_DATA_Y 0x07
#define COMPASS_DATA_Z 0x05
#define CABLESELECTPIN 3


#define ByteToInt(under, upper) (upper<<8 | under) //8bitを16bitに変換する

enum{X, Y, Z};
enum{TEMP, PRESS, HUM};

volatile int16_t speed_x, speed_y;
volatile long cm_x=0, cm_y=0;
volatile int16_t raw_x, raw_y;
volatile int16_t low_x=0, low_y=0, low_z;
volatile int16_t hx, hy;
volatile float fx, fy;
boolean isSdWrite;
char filename[16] = {0};

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
	int num;
	
	Wire.begin();
	writeI2C(I2C_ADDRES_ACCEL, 0x31, 0x00); //range +-2g
	writeI2C(I2C_ADDRES_ACCEL, 0x2D, 0x08); //sleep off
	
	Serial.begin(9600);
	Serial.println();
	Serial.println("I2C SensorTest");
	
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	
	pinMode(10, OUTPUT);
	
	// SD初期化
	num = EEPROM.read(0);
	sprintf(filename, "log_%03d.csv", num);
	if( !SD.begin(CABLESELECTPIN) ) {
		lcd.setCursor(0, 0);
		lcd.print("Failed:SD.begin ");
		isSdWrite = false;
	} else {
		lcd.setCursor(0, 0);
		lcd.print("Success:SD.begin");
		isSdWrite = true;
	}
	if(isSdWrite) {
		SDWriteString( filename, "logingTest\n");
	}
	EEPROM.write(0, ++num);

	lcd.setCursor(15, 1);
	lcd.print(num);
}

void loop()
{
	static long l;

	if(l != millis()) {
		Accel_integral2();
		l = millis();
		//if(!(l % 500)) Accel_integral_print3();
		if(!(l%1000)) {
			writeSd_Accel();
			Accel_integral_print3();
		}
	}
}

void Accel_integral2() {
	byte data[6];


	//センサー値をレジスタから取得
	readI2C(I2C_ADDRES_ACCEL, 0x32, 6, data);

	//取得したデータを16bitのデータにする
	raw_x = ByteToInt(data[0], data[1]);
	raw_y = ByteToInt(data[2], data[3]);

	//ローパスフィルタ
	low_x = 8 * raw_x + 2 * low_x;
	low_y = 8 * raw_y + 2 * low_y;
	
	low_x /= 10;
	low_y /= 10;

	fx = low_x / 256.0;
	fy = low_y / 256.0;
/*	
	fx = raw_x / 256.0;
	fy = raw_y / 256.0;
	hx = raw_x - low_x;
	hy = raw_y - low_y;
	
	speed_x += hx*100/256;
	speed_y += hy*100/256;
*/	
	
/*	  cm_x += raw;
	cm_y += low_y / 256.0;*/
}

void Accel_integral() {
	float accel_data[3];
	byte data[6];
	float raw_x, raw_y, raw_z;

	//センサー値をレジスタから取得
	readI2C(I2C_ADDRES_ACCEL, 0x32, 6, data);

	//取得したデータを16bitのデータにする
	raw_x = ByteToInt(data[0], data[1]);
	raw_y = ByteToInt(data[2], data[3]);
	raw_z = ByteToInt(data[4], data[5]);

	//ローパスフィルタ
	low_x = 0.9 * raw_x + 0.1 * low_x;
	low_y = 0.9 * raw_y + 0.1 * low_y;
	low_z = 0.9 * raw_z + 0.1 * low_z;

	//実際の加速度にする
	accel_data[X] = low_x / 256;
	accel_data[Y] = low_y / 256;
	accel_data[Z] = low_z / 256;;

	speed_x += accel_data[X];
	speed_y += accel_data[Y];
	cm_x += speed_x;
	cm_y += speed_y;
}

void Accel_integral_print() {
	Serial.print(cm_x);
	Serial.print(",");
	Serial.println(cm_y);
}

void Accel_integral_print2() {
	Serial.print(fx);
	Serial.print(",");
	Serial.println(fy);
}

void Accel_integral_print3() {
	lcd.setCursor(0, 1);
	lcd.print("                ");
	lcd.setCursor(0, 1);
	lcd.print(fx);
	lcd.setCursor(8, 1);
	lcd.print(fy);
}

void writeSd_Accel() {
	char buf[64] = {0};

	
	sprintf( buf, "%d,%d\n", raw_x, raw_y);
	SDWriteString( filename, buf);
}

/*********************************************************************/
/*    ADXL345                                                       */
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

	//ローパスフィルタ
	low_x = 0.9 * raw_x + 0.1 * low_x;
	low_y = 0.9 * raw_y + 0.1 * low_y;
	low_z = 0.9 * raw_z + 0.1 * low_z;

	//実際の加速度にする
	accel_data[X] = low_x / 256;
	accel_data[Y] = low_y / 256;
	accel_data[Z] = low_z / 256;
#if 0
	accel_data[X] = raw_x / 256;
	accel_data[Y] = raw_y / 256;
	accel_data[Z] = raw_z / 256;
#endif
}
/*********************************************************************/
/* SDread/write関数                                                  */
/*********************************************************************/
boolean SDWriteString( const char* pszFileName, const char* pszString )
{
	File file = SD.open( pszFileName, FILE_WRITE );
	if( false == file )
	{
		return false;
	}
	file.print( pszString );
	file.close();
	return true;
}

boolean SDReadString( const char* pszFileName, char* pszBuffer, int iBufferSize )
{
	File file = SD.open( pszFileName, FILE_READ );
	if( false == file )
	{
		return false;
	}
	int iIndex = 0;
	while( file.available() )
	{
		if( iBufferSize - 1 <= iIndex )
		{
			break;
		}
		pszBuffer[iIndex] = file.read();
		iIndex++;
	}
	pszBuffer[iIndex] = '\0';
	file.close();
	return true;
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

//	for(i=0; i<byte_num; i++) ret_ary[i] = Wire.read();
#if 1  
	while (Wire.available()) {
		ret_ary[i] = Wire.read();
		i++;
	}
#endif
//	Wire.endTransmission();
}


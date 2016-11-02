//#include <SparkFunBME280.h>

#include <Wire.h>
#include <stdint.h>
#include <arduino.h>
#include <math.h>
#include <MsTimer2.h>

#define I2C_ADDRES_ACCEL 0x1D		//加速度センサー(ADXL345)
#define I2C_ADDRES_COMPASS 0x1E		//地磁器センサー(HMC5883L)
#define I2C_ADDRES_AIRSTATE 0x77	//温度、湿度、大気圧センサー(BME280)

#define COMPASS_DATA_X 0x03
#define COMPASS_DATA_Y 0x07
#define COMPASS_DATA_Z 0x05

#define ByteToInt(under, upper) (upper<<8 | under) //8bitを16bitに変換する

enum{X, Y, Z};
enum{TEMP, PRESS, HUM};

float speed_x=0.0, speed_y=0.0;
float cm_x=0.0, cm_y=0.0;
float low_x = 0.0, low_y = 0.0, low_z = 0.0;

void setup()
{
    Wire.begin();
    writeI2C(I2C_ADDRES_ACCEL, 0x31, 0x00);
    writeI2C(I2C_ADDRES_ACCEL, 0x2D, 0x08);
    
    Serial.begin(9600);
    Serial.println();
    Serial.println("I2C SensorTest");
    
//  MsTimer2::set(1000, AccelTest);
//  MsTimer2::start();
}

void loop()
{
    static long l;

    if(l != millis()) {
        Accel_integral();
        l = millis();
        if(!(l % 1000)) Accel_integral_print();
    }
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

#if 0
void loop()
{
  float accel_data[3];

  getAccel(accel_data);
  
  Serial.print("x:");
  Serial.print(accel_data[0]);
  Serial.print("  ");
  Serial.print("y:");
  Serial.print(accel_data[1]);
  Serial.print("  ");
  Serial.print("z:");
  Serial.println(accel_data[2]);
  
  delay(1000);
}
#endif
/*********************************************************************/
/*	   ADXL345														 */
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
/*	   I2Cread/write関数											 */
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

//  for(i=0; i<byte_num; i++) ret_ary[i] = Wire.read();
#if 1  
    while (Wire.available()) {
        ret_ary[i] = Wire.read();
        i++;
    }
#endif
//  Wire.endTransmission();
}


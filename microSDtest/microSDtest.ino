#include <SPI.h>

#include <SD.h>

#define CABLESELECTPIN	3
// ファイル名は、8.3形式対応（拡張子でない部分は8文字以内にしないと、SD.open()に失敗する）
#define STRING_FILENAME		("test.txt")


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

void setup()
{
	Serial.begin(9600);

	// EhternetShieldの場合は、CSはpin4だが、
	// その場合でも、pin10は、出力ピンに設定する必要がある。
	pinMode(10, OUTPUT);

	Serial.print("SD read write test start...\n");

	// 初期化
	if( !SD.begin(CABLESELECTPIN) )
	{
		Serial.print("Failed : SD.begin()\n");
		return;
	}

	// 追加書き込み
	char szWriteString[] = "abcdefghi1234567890\n";
	Serial.print( "Write String : " );
	Serial.print( szWriteString );
	Serial.print( "\n" );
	if( !SDWriteString( STRING_FILENAME, szWriteString ) )
	{
		Serial.print("Failed : WriteStringAtEndOfFile()\n");
		return;
	}
	
	// 読み込み
	char szBuffer[256];
	if( !SDReadString( STRING_FILENAME, szBuffer, 256 ) )
	{
		Serial.print("Failed : ReadString()\n");
		return;
	}
	Serial.print( "Read String :\n" );
	Serial.print( "---start---\n" );
	Serial.print( szBuffer );

	Serial.print( "---end---\n" );
	Serial.print("Succeeded.\n\n");
}

void loop()
{
	;
}


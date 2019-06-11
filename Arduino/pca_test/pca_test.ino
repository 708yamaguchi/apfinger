#include "Wire.h"

#define I2C_ADRS 0x71

int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
     unsigned char c ;
     int  ans ;

     Wire.beginTransmission(adrs) ;     // 通信の開始
     c = ch & 0x07 ;                    // チャネル(bit0-2)を取り出す
     c = c | 0x08 ;                     // enableビットを設定する
     Wire.write(c) ;                    // Control register の送信
     ans = Wire.endTransmission() ;     // データの送信と通信の終了
     return ans ;
}

void setup() {
  // put your setup code here, to run once:
  // Ｉ２Ｃの初期化マスターとする
  Wire.begin();
  Serial.begin(9600);
  // I2Cマルチプレクサー(PCA9547)を1チャンネルに切り換える
  Serial.println(ChgI2CMultiplexer(I2C_ADRS,1));
}

void loop() {
  // put your main code here, to run repeatedly:

}

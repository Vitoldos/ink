#include "OneWire.h"
#include "LiquidCrystal.h"
#include "DHT.h"
#define DHTPIN 7     // датчик dht11 на 7 pin
//#define DHTTYPE DHT11   // DHT 11 тип датчика
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
#define DS18S20_ID 0x10
#define DS18B20_ID 0x28

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // инициализируем LCD
OneWire ds(8); // датчик ds18b20 pin на 8 входе Ардуино

byte i;
byte present = 0;
byte data[12];
byte addr[8];

int val = 0;  // переменная для хранения значения датчика жидкости
int Max, MFract; //переменная для хранения значения максимальной температуры
int HighByte, LowByte, Whole, SignBit, Fract, TReading, Tc_100, HighTemp, Fast;
DHT dht(DHTPIN, DHTTYPE); //инициализация DHT11

uint8_t strelka_vverh[8] =
{
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100
}; //закодировано в двоичной системе заначек стрелка вверх

uint8_t strelka_vniz[8] =
{
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
}; //закодировано в двоичной системе заначек стрелка вниз

uint8_t temp_cel[8] =
{
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000
}; //закодировано в двоичной системе заначек градуса

void setup(void){
#define dotimes(n, code) for (int i = 0; i < (n); ++i) code;
  Serial.begin(9600);
  dht.begin();
  lcd.begin(16,2);//инициализация дисплея
  lcd.createChar(1,strelka_vverh);
  lcd.createChar(2,strelka_vniz);
  lcd.createChar(3,temp_cel);
  lcd.setCursor(16,0);
  lcd.autoscroll();
  const char *text = "Welcome    ";

  dotimes(strlen(text),
  {

    lcd.write(text[i]);
    delay(300);
  }
  );

  lcd.noAutoscroll();
   delay(1000);
  lcd.clear();
  lcd.print("03.11.2013");
  lcd.setCursor(0,2);
  lcd.print("V 0.6b");
  delay(1000);
  Serial.print("Wellcome ");
  Serial.print("Incubator ISM-1\n");
  Serial.print("Firmware 0.6.0 m,-beta. Demon 31.01.2013 (03.11.2013) \n");
  pinMode(10, OUTPUT);//инициализация светодиода
  pinMode(14, INPUT);//подключение датчика жидкости
  delay(500); 
}

void getSerial() {
  if ( !ds.search(addr)) {
    Serial.print("No more addresses.\n");
    ds.reset_search(); //сброс датчика температуры
    return;
  }
  Serial.print("R=");
  for( i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("CRC is not valid!\n");
    lcd.print("Error sensor T");
    return; //сброс датчика температуры
  }
  if ( addr[0] == 0x10) {
    Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[0] == 0x28) {
    Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
    Serial.print("Device family is not recognized: 0x");
    Serial.println(addr[0],HEX);
    return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);  
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("P=");
  Serial.print(present,HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print( OneWire::crc8( data, 8), HEX);
  Serial.println();
}
void getTemp() {

  LowByte = data[0];
  HighByte = data[1];
  HighTemp = data[2];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit

  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;
  Serial.print("TemperatureDS: ");
  if (SignBit) // If its negative
  {
    Serial.print("-");
  }
  Serial.print(Whole);
  Serial.print(".");
  if (Fract < 10)
  {
    Serial.print("0");
  }
  Serial.print(Fract);
  Serial.print(" C");
  Serial.print("\n");
} 
void getHum(void) {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
    lcd.print("Error DHT");
  } 
  else {
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("TemperatureDHT: "); 
    Serial.print(t);
    Serial.println(" *C");
  }
}
void printTempHum(void) {
  float h = dht.readHumidity();
  //lcd.begin(16, 2);
  lcd.setCursor(0,0);
  //lcd.noDisplay();
  lcd.print("T:");

  if (SignBit)
  {
    lcd.print("-");
  }

  lcd.print(Whole);
  lcd.print(".");
  if (Fract < 10)
  {
    lcd.print("0");
  }

  lcd.print(Fract);
  lcd.print("  H:");
  lcd.print(h,1);
  lcd.print("%");
    if (Whole < 34)
  {

    tone(9, 800,600);
    Serial.print("Attention! The low temperature! \n");
    lcd.setCursor(8,2);
    lcd.print("T");
    lcd.print("\2");
  } 
  else   if  (Whole > 40)
  {
    lcd.display();
    tone(9, 800,600);
    Serial.print("Danger! The higth temperature! \n");
    lcd.setCursor(8,2);
    lcd.print("T");
    lcd.print("\1");
  }
  else { 
    lcd.setCursor(8,2);
    lcd.print("  ");
  }
  if (h >80 )
  {
    tone(9, 800,600);
    Serial.print("Attention! The higth humidity! \n");
    lcd.setCursor(11, 2);
    lcd.print("H");
    lcd.print("\1");
  }
  else if (h < 10)
  {
    tone(9, 800,600);
    Serial.print("Attention! The low humidity! \n");
    lcd.setCursor(11, 2);
    lcd.print("H");
    lcd.print("\2");
  } 
  else { 
    lcd.setCursor(11,2);
    lcd.print("  ");
  }

  if ((Whole > 31) && (Whole < 40))
  {
    Serial.println("The normal incubation");
  }         
  if (Max < Whole)
  {
    Max=Whole;
    Serial.print("Maximum ");
    Serial.print(Max);
  }

  if (MFract < Fract)
  {
    MFract=Fract;
    Serial.print(".");
    Serial.print(Fract);
    Serial.println(" *C");
  }  
  lcd.setCursor(0, 2);
  lcd.print("Tm:");
  lcd.print(Max);
  lcd.print("\3");
  lcd.print("C");
 }

void WaterSensor(void)
{
  digitalWrite(14, HIGH);
  val = digitalRead(14);// считываем значение с входа
  if (val==LOW)
  {
    digitalWrite(10, HIGH);
    Serial.print("Attention! The low water! \n");
    lcd.setCursor(14, 2);
    lcd.print("W");
    lcd.print("\2");
  }
  if (val==HIGH)
  {
    digitalWrite(10, LOW);
    lcd.setCursor(14, 2);
    lcd.print("  ");
  }
}
void loop(void)
{
  getSerial();
  getTemp();
  printTempHum();
  getHum();
  WaterSensor();
}

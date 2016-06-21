/*http://demon200870.narod.ru*/
#include <avr/wdt.h>
#include <PID_v1.h>
#include <OneWire.h> 
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <EEPROM.h>
#include <DHT.h>


#define SER_Pin  8   //pin 14 (DS) на 75HC595
#define RCLK_Pin  7  //pin 12 (ST_CP) на 75HC595
#define SRCLK_Pin  9 //pin 11 (SH_CP) на 75HC595

//количество регистров
#define number_of_74hc595s 2

//не трогать !
#define numOfRegisterPins number_of_74hc595s * 8
boolean registers[numOfRegisterPins];
//

#define NUM_KEYS 5
#define Zum 14 //Пьезопищалка
//#define Water_Sensor 15 //Датчик уровня воды
#define DoorSensor 3 //Датчик дверцы
#define heater 2 //нагреватель
#define WSensor 15 //Датчик жидкости

#define ONE_WIRE_BUS 11 // DS18B20 на 11pin
#define DHTPIN 5     // датчик dht11 на 5 pin
//#define DHTTYPE DHT11   // DHT 11 тип датчика
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//#define addr0 0
#define addrbeep_alarm 1
#define addrLight_LCD 2//ячейка для включение подсветки

#define addrTmax 9 //
#define addrTmin 14 //
#define addrTp 19 //

#define del_knopki 200//задержка для кнопок
#define del_MineMenu 60 //задержка смены основного меню
int button;
const int BUTTON_NONE   = 0;
const int BUTTON_SAVE   = 2;
const int BUTTON_MINUS  = 1;
const int BUTTON_EXIT   = 3;
const int BUTTON_PLUS   = 4;
const int BUTTON_SELECT = 5;

int  adc_key_val[NUM_KEYS] = {
  30, 160, 360, 535, 760};

boolean val_water;// переменная для хранения значения датчика жидкости
boolean val_door;
boolean beep_alarm; //переменная для хранения значения сигнализации
boolean Light_LCD;
int Tp;//время переворота в сек
int  m=0;//переменная для экранов меню

float Tmax; //максимальная критическая температура
float Tmin; //значение минимальной температуры
float h;//переменная для влажности
float temperature;//переменеая для температуры DS18B20
float t;//переменная для температуры для DHT
float resistor1Value;//переменная для потенциометра

uint8_t strelka_vverh[8] =
{
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100
}; //закодировано в двоичной системе значек стрелка вверх

uint8_t strelka_vniz[8] =
{
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
}; //закодировано в двоичной системе значек стрелка вниз

uint8_t temp_cel[8] =
{
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000
}; //закодировано в двоичной системе значек градуса

unsigned long currentTime;               // задаем переменные для тайминга поворота
unsigned long loopTime;

unsigned long TTime; //текущее время currentTime для влажности
unsigned long LTime; // LoopTime

unsigned long WTime; //текущее время currentTime для влажности
unsigned long wTime; // LoopTime

DHT dht(DHTPIN, DHTTYPE); //инициализация DHT11
OneWire oneWire(ONE_WIRE_BUS);                // на цифровом пине 14
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
DeviceAddress Thermometer; // адрес устройства
/*В этой версии избавиться от лишних переменных
значения коэффициентов перенести в инициализацию ПИД*/
//float consKp=10000, consKi=3500, consKd=5;
//объявляем переменные для ПИД
double Setpoint, Input, Output;//Инициализируем ПИД-библиотеку и коэффициенты
PID myPID(&Input, &Output, &Setpoint,12000,0,6000, DIRECT);
int WindowSize = 2000;// ширина окна терморегулятора 2 секунды.
unsigned long windowStartTime;
LiquidCrystal_I2C lcd(0x20,16,2);  // инициализация библиотеки с перечнем задействованных выводов

void setup()
{
  wdt_disable(); // запретили как можно скорее собаку, что-бы не уйти в бесконечный ребут

  Serial.begin(9600);
  lcd.init();
  //lcd.backlight();
  Light(); 
  lcd.begin(16, 2);       // устанавливаем кол-во столбцов и строк
  delay(100); //чтобы успевала иницилизация
  lcd.createChar(1,strelka_vverh);//стрелка вверх
  lcd.createChar(2,strelka_vniz);
  lcd.createChar(3,temp_cel);
  pinMode(heater, OUTPUT); //реле
  pinMode(WSensor, INPUT);//подключение датчика жидкости
  pinMode(DoorSensor, INPUT);//датчик двери
  digitalWrite(DoorSensor, HIGH);

  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  registerWrite(1,1);
  registerWrite(2,1);
  registerWrite(3,1);
  registerWrite(4,1);
  registerWrite(4,1);
  registerWrite(6,1);
  registerWrite(7,1);
  registerWrite(8,1);
  delay(1000);
  registerWrite(1,0);
  registerWrite(2,0);
  registerWrite(3,0);
  registerWrite(4,0);
  registerWrite(4,0);
  registerWrite(6,0);
  registerWrite(7,0);
  registerWrite(8,0);

  Serial.println("\n[memCheck]");
  Serial.println(freeRam());
  // Start up the library
  dht.begin(); //инициализация датчика DHT11(22)
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(Thermometer, 12);
  //sensors.setResolution(outsideThermometer, 12);
  // sensors.setResolution(dogHouseThermometer, 10);

  windowStartTime = millis();
  //задаем лимиты ширины ПИД-импульса от 0 до 2 секунды.
  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetMode(AUTOMATIC); //включаем ПИД-регулирование
  //myPID.SetSampleTime(500);//Узнать для чего это
  myPID.SetSampleTime(500);//How often, in milliseconds, the PID will be evaluated. 

  currentTime = millis();// считываем время, прошедшее с момента запуска программы
  loopTime = currentTime; 

  beep_alarm = EEPROM.read(addrbeep_alarm);
  Tmin = EEPROM_float_read(addrTmin);//присвоение переменной Tmin для минимальной температуры
  Tmax = EEPROM_float_read(addrTmax);//присвоение переменной Tmaxдля максимальной температуры
  Tp = EEPROM.read(addrTp);//присвоение значения переворота Tp
  
  lcd.print("    Welcome ");
  lcd.setCursor(0,1);
  lcd.print("Fw 0.4.30m-a");
  Serial.println("FW 0.4.30m-alfa");
  delay(1000);
  lcd.clear();
  lcd.print("Software");
  lcd.setCursor(0,1);
  lcd.print("version:");    
  Serial.print("Software version: ");
  delay(1000);
  lcd.clear();
  lcd.print(__DATE__);
  lcd.setCursor(0,1);
  lcd.print(__TIME__);
  delay(2000);
  lcd.clear();
  lcd.print(__FILE__);
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println(__FILE__);
  delay(2000);
  lcd.clear();
  h = dht.readHumidity();//что бы значение влажности появлялось сразу
  wdt_enable(WDTO_8S); // активировали таймер, каждые 8 секунд его нужно сбрасывать
 
}

void loop()
{
  wdt_reset(); // говорим собаке что "В Багдаде все спокойно", начинается очередной отсчет 4-х секунд.

  TTime = millis();
  if (TTime >= (LTime + 20000))
  {
    DHTT();//запуск функции измерения влажности раз в 10 сек
    LTime = TTime;
  }
  WTime = millis();
  if (WTime >= (wTime + 50000))
  {
    WaterSensor();//запуск функции измерения уровня жидкости раз в 10 сек
    wTime = WTime;
  } 
  knopki();
  menu();
  ErrorTemt();
  registerWrite(1,1);
  
  val_door = digitalRead(DoorSensor);// считываем значение с входа
  registerWrite(5,0);
  if (val_door==LOW)//проверяем датчик двери
  {
    wdt_reset();
    myPID.Compute();
    unsigned long now = millis();
    temperature = sensors.getTempCByIndex(0);
   
    Input = temperature;

    resistor1Value = analogRead(A3);
    resistor1Value = map(resistor1Value, 0, 1023, 300, 410);
    Setpoint = resistor1Value / 10;

    if(now - windowStartTime>WindowSize)
    {                       //время для перещелкивания периода окна
      windowStartTime += WindowSize;

      currentTime = millis();                           // считываем время, прошедшее с момента запуска программы

      sensors.requestTemperatures();
      Serial.println(temperature);
      lcd.setCursor(0, 0); 
      lcd.print("T:");
      lcd.print(temperature, 1);              // печать температуры на дисплей
      lcd.print(" ");
      lcd.setCursor(7,0);
      lcd.print("H:");
      lcd.print(h,0);
      lcd.print(" % ");
      lcd.setCursor(13, 0);
      lcd.print(int(Output/20)); //Продумать вывод мощности
      lcd.print(" ");
      lcd.setCursor(0, 1);                 // устанавливаем курсор в 0-ом столбце, 1 строка (начинается с 0)
      lcd.print("Tt:");
      lcd.print(Setpoint, 1);
      Serial.print("Output = ");
      Serial.println(int(Output/20)); //это приведенный к 100% выходной коэффициент. У нас же импульс 2 секунды (2000мС), вот мы его на 20 и делим чтобы 100 получить в максимальном значении.
    }
    if(Output > now - windowStartTime)
    {
      Serial.println("TEN HIGH");
      digitalWrite(heater,HIGH);
      registerWrite(2,1);
     
    }
    else
    {
      Serial.println("TEN LOW");    
      digitalWrite(heater,LOW);
      registerWrite(2,0);
      
    }
  }
  if (val_door==HIGH)//если дверь открыта ВСЕ ОТКЛЮЧИТЬ
  {
    wdt_reset();
    digitalWrite(heater,LOW);
    registerWrite(2,0);//выключить светодиод
    registerWrite(5,1);//выключить светодиод
    lcd.setCursor(0,0);
    lcd.print ("Door Open       ");
    lcd.setCursor(0,1);
    lcd.print("PID OFF         ");
    lcd.setCursor(0,0);
    myPID.SetMode(AUTOMATIC); //включаем ПИД-регулирование
    myPID.SetSampleTime(500);//How often, in milliseconds, the PID will be evaluated. 
    Serial.println("Door Open. PID OFF");
  }
}

void DHTT()
{
  
  h = dht.readHumidity();
  t = dht.readTemperature();
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed DHT");
    lcd.setCursor(10,0);
    lcd.print("Err");
  } 
  else {
    Serial.print("H: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("TempDHT: "); 
    Serial.println(t);
  }
}

void EEPROM_float_write(int addr, float val) // запись в ЕЕПРОМ
{  
  byte *x = (byte *)&val;
  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
}

float EEPROM_float_read(int addr) // чтение из ЕЕПРОМ
{    
  byte x[4];
  for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
  float *y = (float *)&x;
  return y[0];
}

void ErrorTemt()
{
  
  if ((sensors.getTempCByIndex(0)== -127.00)|| (sensors.getTempCByIndex(0)== 0.00) || (sensors.getTempCByIndex(0)== 85.00))
  {
    lcd.clear();
    do
    {
      digitalWrite(heater, LOW);//выключить реле
      registerWrite(4,1);
       Serial.println("ERROR DS18B20");
      lcd.print("ERROR DS18B20");
      lcd.setCursor(0,0);
      Serial.println(sensors.getTempCByIndex(0));   
      
    }
    while ((sensors.getTempCByIndex(0)== -127.00)|| (sensors.getTempCByIndex(0)== 0.00));// || (TError != 85.00)); //разобраться!!!
    lcd.clear();
    registerWrite(4,0);
  } 
  float h = dht.readHumidity();
 
  if (h > 80)
  {
    lcd.setCursor(11, 1);
    lcd.print("H");
    lcd.print("\1");
    lcd.setCursor(0, 0);                 // устанавливаем курсор в 0-ом столбце, 1 строка (начинается с 0)
  } 
  else  if (h < 10)
  { 
    lcd.setCursor(11, 1);
    lcd.print("H");
    lcd.print("\2"); 
    lcd.setCursor(0, 0);                 // устанавливаем курсор в 0-ом столбце, 1 строка (начинается с 0)
  } 
  else {
    lcd.setCursor(11,1); 
    lcd.print("  ");
  }

  if (sensors.getTempCByIndex(0) > Tmax) 
  {
   
    registerWrite(4,1);
    sound();
    lcd.setCursor(9, 1);
    lcd.print("T");
    lcd.print("\1");
  }
  else  if (sensors.getTempCByIndex(0) < Tmin)
  {
    
    registerWrite(3,1);
    sound();
    lcd.setCursor(9, 1);
    lcd.print("T");
    lcd.print("\2");
  }else {
    registerWrite(3,0);
    registerWrite(4,0); 
    lcd.setCursor(9,1);
    lcd.print("  ");
  }
}
void WaterSensor()
{
  digitalWrite(WSensor, HIGH);
  val_water = digitalRead(WSensor);// считываем значение с входа

  if (val_water==LOW)
  {
    
    Serial.print("The low water! \n");
    lcd.setCursor(14, 1);
    lcd.print("W");
    lcd.print("\2");
    registerWrite(6,1);
  }
  if (val_water==HIGH)
  {
    lcd.setCursor(14, 1);
    lcd.print("  ");
    registerWrite(6,0);
  }
}

void knopki()
{
  button = getPressedButton();
  if (button == BUTTON_SELECT) {
    Serial.println("BUTTON_SELECT");
   }
  if (button == BUTTON_MINUS)  {
    Serial.println("BUTTON_MINUS");
   }
  if (button == BUTTON_SAVE)     {
    Serial.println("BUTTON_SAVE");
   }
  if (button == BUTTON_EXIT)   {
    Serial.println("BUTTON_EXIT");
   }
  if (button == BUTTON_PLUS)   {
    Serial.println("BUTTON_PLUS");
   }
}

byte get_key(int key_pin)
{
  if(analogRead(key_pin) < adc_key_val[NUM_KEYS-1])
  {
    byte k[3];
    for(byte i = 0; i < 3; i++)
    {
      delay(10);
      k[i] = 0;
      for(byte j = 0; j < NUM_KEYS; j++)
      { 
        if(analogRead(key_pin) < adc_key_val[j]) 
        {
          k[i] = j + 1;
          break;
        }
      }
    }
    if((k[0] > 0) && (k[0] == k[1]) && (k[0] == k[2]) && (k[1] == k[2])) return k[0];
  }
  return 0;
}


int getPressedButton()
{
  int key = get_key(A2);//считываем значение с аналогового входа(А2)
  if (key == 1) {
    return BUTTON_EXIT;
  }
  else if (key == 2) {
    return BUTTON_SAVE;
  }
  else if (key == 3) {
    return BUTTON_MINUS;
  }
  else if (key == 4) {
    return BUTTON_PLUS;
  }
  else if (key == 5) {
    return BUTTON_SELECT;
  }
  return BUTTON_NONE;
}
void menu()
{
  if (button==BUTTON_SELECT)
  {
    digitalWrite(heater,LOW);
    registerWrite(2,0);
    lcd.clear();
    while(1)
    {
      wdt_reset(); 
      menu2();
      button = getPressedButton();
      if (button == BUTTON_EXIT)
      { 
        delay(del_knopki);
        m=0;
        lcd.clear();
        
        break;
      }
    }
  }
}
void menu2()
{
  if (button==BUTTON_PLUS)
  {
    m++;//увеличиваем переменную уровня меню
  }
  if (m>6)//если уровень больше 5
  {
    m=0;// то вернуться к началу
  }
  //Обработка нажатия кнопки назад
  if (button==BUTTON_MINUS)
  {
    m--;
    if (m<0)
    {
      m=6;
    }
  }
  //вывод меню
  if (m==0)
  {
    lcd.setCursor(0,0);
    lcd.print("   Mine menu    ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    Serial.println("Main Menu");
    delay(del_MineMenu);
  } 
  else if (m==1)
  {
    lcd.setCursor(0,0);
    lcd.print("Max Temp Set ");
    lcd.setCursor(0,1);
    lcd.print("Tmax=");
    lcd.print(Tmax);
    //Serial.setCursor(0, 0);
    Serial.println("Max Temp Set");
    delay(del_MineMenu);
    //Обработка Tmax
    if (button == BUTTON_SELECT) 
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Tmax=");
      lcd.print(Tmax);
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_PLUS)
        {
          Tmax+=0.1;
          delay(del_knopki);
          Serial.print("Tmax=");
          Serial.println(Tmax);
          lcd.setCursor(5,0);
          lcd.print(Tmax);
        }
        if (button==BUTTON_MINUS)
        {
          Tmax-=0.1;
          delay(del_knopki);
          Serial.print("Tmax=");
          Serial.println(Tmax);
          lcd.setCursor(5,0);
          lcd.print(Tmax);
        }
        if (button==BUTTON_SAVE)
        {
          EEPROM_float_write(addrTmax, Tmax); //зпись Tmax в память
          Serial.println("SAVE Tmax");
          lcd.setCursor(0,1);
          lcd.print("SAVE Tmax");
        }
        if (button==BUTTON_EXIT){
          delay(del_knopki);
          m=1;
          Serial.print("EXIT"); 
          break;
        }
      }
    }
  }
  else if (m==2)
  {
    lcd.setCursor(0,0);
    lcd.print("Min Temp Set ");
    lcd.setCursor(0,1);
    lcd.print("Tmin=");
    lcd.print(Tmin);
    //Serial.setCursor(0, 0);
    Serial.println("Min Temp Set");
    delay(del_MineMenu);
    //Обработка Tmin
    if (button == BUTTON_SELECT)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Tmin=");
      lcd.print(Tmin);
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_PLUS)
        {
          Tmin+=0.1;
          delay(del_knopki);
          Serial.print("Tmin=");
          Serial.println(Tmin);
          lcd.setCursor(5,0);
          lcd.print(Tmin);
        }
        if (button==BUTTON_MINUS)
        {
          Tmin-=0.1;
          delay(del_knopki);
          Serial.print("Tmin=");
          Serial.println(Tmin);
          lcd.setCursor(5,0);
          lcd.print(Tmin);
        }
        if (button==BUTTON_SAVE)
        {
          EEPROM_float_write(addrTmin, Tmin); //зпись Tmax в память
          Serial.println("SAVE Tmin");
          lcd.setCursor(0,1);
          lcd.print("SAVE Tmin");
        }
        if (button==BUTTON_EXIT){
          delay(del_knopki);
          Serial.print("EXIT"); 
          m=2;
          break;
        }
      }
    }
  }
  //------------------Переворот-----------------------
  else if (m==3)
  {
    lcd.setCursor(0,0);
    lcd.print("Perevorot Set");
    lcd.setCursor(0,1);
    lcd.print("Time=");
    lcd.print(Tp);
    lcd.print("      ");
    lcd.print("   ");
    //Serial.setCursor(0, 0);
    Serial.println("Perevorot Set");
    delay(del_MineMenu);
    //Обработка времени преворота
    if (button == BUTTON_SELECT)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Perevorot=");
      lcd.print(Tp);
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_PLUS)
        {
          Tp++;
          delay(del_knopki);
          Serial.print("Time Per=");
          Serial.println(Tp);
          lcd.setCursor(10,0);
          lcd.print(Tp);
        }
        if (button==BUTTON_MINUS)
        {
          Tp--;
          delay(del_knopki);
          Serial.print("Time per=");
          Serial.println(Tp);
          lcd.setCursor(10,0);
          lcd.print(Tp);
        }
        if (button==BUTTON_SAVE)
        {
          EEPROM.write(addrTp, Tp); //запись значения переворота  
          Serial.println("SAVE Tp");
          lcd.setCursor(0,1);
          lcd.print("SAVE Tp");
        }
        if (button==BUTTON_EXIT){
          delay(del_knopki);
          Serial.print("EXIT"); 
          m=3;
          break;
        }
      }
    }
  } 
  else if (m==4)
  {
    lcd.setCursor(0,0);
    lcd.print("Zavod setup     ");
    lcd.setCursor(0,1);
    lcd.print("Enter Select");
    //Serial.setCursor(0, 0);
    Serial.println("Zavod setup");
    delay(del_MineMenu);
    //Обработка возврат к заводским настройкам
    if (button == BUTTON_SELECT) 
    {
      lcd.setCursor(0,1);
      lcd.print("YES=SAVE NO=EXIT");
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_SAVE)
        {
          lcd.clear();
          EEPROM_float_write(addrTmax, 40); //зпись Tmax в память однократно
          EEPROM.write(addrTp, 10); //запись значения переворота 
          EEPROM_float_write(addrTmin, 30); //запись значения  Tmin
          lcd.print("Reset OK!");
          lcd.setCursor(0,1); 
          lcd.print("Reboot after 8c");
          Serial.println("Reset OK!");
          delay(1000);
          reboot();
        }
        if (button==BUTTON_EXIT){
          delay(del_knopki);
          Serial.print("EXIT"); 
          m=4;
          break;
        }
      }
    } 
  }
  //------------------Включение отключение звуковой сигнализации------------------
  else if (m==5)
  {
    lcd.setCursor(0,0);
    lcd.print("Beep Alarm  ");
    //lcd.setCursor(0,1);
    //lcd.print("                ");
    beep_alarm=EEPROM.read(addrbeep_alarm);
    if (beep_alarm==0)
    {
      lcd.setCursor(12,0);   
      lcd.print("OFF");
    } 
    else {
      lcd.print("ON ");
    }
    lcd.setCursor(0,1);
    lcd.print("                ");
    Serial.println("Beep Alarm Set");
    delay(del_MineMenu);
    //Обработка 
    if (button == BUTTON_SELECT)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Beep Alarm Set");
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_PLUS)
        {
          beep_alarm =1;
          delay(del_knopki);
          Serial.print("beep_alarm");
          Serial.println(beep_alarm);
          lcd.setCursor(0,1);
          lcd.print("ON ");
        }
        if (button==BUTTON_MINUS)
        {
          beep_alarm =0;
          delay(del_knopki);
          Serial.print("beep_alarm");
          Serial.println(beep_alarm);
          lcd.setCursor(0,1);
          lcd.print("OFF ");
        }
        if (button==BUTTON_SAVE)
        {
          EEPROM.write(addrbeep_alarm, beep_alarm); //запись значения переворота  
          Serial.println("SAVE Alarm");
          lcd.setCursor(3,1);
          lcd.print(" SAVE Alarm  ");
        }
        if (button==BUTTON_EXIT)
          {
          delay(del_knopki);
          Serial.print("EXIT"); 
          m=5;
          break;
        }
        
      }
    }
  }
  else if (m==6)
  {
    lcd.setCursor(0,0);
    lcd.print("Light LCD ");
    //lcd.setCursor(0,1);
    //lcd.print("                ");
    Light_LCD=EEPROM.read(addrLight_LCD);
    if (Light_LCD==0)
    {
      lcd.setCursor(10,0);   
      lcd.print("OFF ");
    } 
    else {
      lcd.print("ON  ");
    }
    lcd.setCursor(0,1);
    lcd.print("                ");
    Serial.println("Light LCD");
    delay(del_MineMenu);
    //Обработка 
    if (button == BUTTON_SELECT)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Light LCD Set   ");
      while(1)
      {
        wdt_reset(); 
        button = getPressedButton();
        if (button==BUTTON_PLUS)
        {
          Light_LCD =1;
          lcd.backlight();
          delay(del_knopki);
          Serial.print("light_LCD");
          Serial.println(Light_LCD);
          lcd.setCursor(0,1);
          lcd.print("ON ");
        }
        if (button==BUTTON_MINUS)
        {
          Light_LCD =0;
          lcd.noBacklight();
          delay(del_knopki);
          Serial.print("light_LCD");
          Serial.println(Light_LCD);
          lcd.setCursor(0,1);
          lcd.print("OFF ");
        }
        if (button==BUTTON_SAVE)
        {
          EEPROM.write(addrLight_LCD, Light_LCD); //запись значения переворота  
          Serial.println("SAVE light_LCD");
          lcd.setCursor(3,1);
          lcd.print(" SAVE light  ");
        }
        if (button==BUTTON_EXIT)
          {
          delay(del_knopki);
          Serial.print("EXIT"); 
          m=6;
          break;
        }
        
      }
    }
  }
}
void registerWrite(int whichPin, int whichState) {
  registers[whichPin] = whichState;

  digitalWrite(RCLK_Pin, LOW);

  for(int i = numOfRegisterPins - 1; i >=  0; i--){
    digitalWrite(SRCLK_Pin, LOW);

    int val = registers[i];

    digitalWrite(SER_Pin, val);
    digitalWrite(SRCLK_Pin, HIGH);

  }
  digitalWrite(RCLK_Pin, HIGH);
}
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
void reboot() {
  wdt_disable();  
  wdt_enable(WDTO_8S);
  while (1) {
  }
}
void sound()
{
 
  if (beep_alarm == 1)
  {
    tone(Zum, 800);
    delay(150);
    noTone(Zum);
  }
  else if (beep_alarm == 0)
  {
    noTone(Zum);
  }
}

void Light()
{
  Light_LCD = EEPROM.read(addrLight_LCD);
  if (Light_LCD == 1)
  {
    lcd.backlight();
  }
  else if (Light_LCD == 0)
  {
    lcd.noBacklight();
  } 
}

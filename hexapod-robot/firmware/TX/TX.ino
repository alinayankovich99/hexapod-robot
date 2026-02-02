/*
   Пример скетча "управление с телеметрией", то есть модуль ПЕРЕДАТЧИК
   шлёт на ПРИЁМНИК команды управления, ПРИЁМНИК при получении пакета данных
   отправляет ПЕРЕДАТЧИКУ пакет телеметрии (какие-то свои данные). ПЕРЕДАТЧИК
   эти данные принимает. Вот такие пироги. Также в этом примере реализован
   расчёт RSSI (процент ошибок связи), на основании которого можно судить о
   качестве связи между модулями.
*/

//--------------------- НАСТРОЙКИ ----------------------
#define CH_NUM 0x70   // номер канала (должен совпадать с приёмником)

#define pin_button_1  42
#define pin_button_2  43
#define pin_button_3  31
#define pin_button_4  29
/*
#define pin_button_1  26
#define pin_button_2  30
#define pin_button_3  28
#define pin_button_4  24
*/
#define pin_x_joystick_1  A1
#define pin_y_joystick_1  A0
#define pin_button_joystick_1  46

#define pin_x_joystick_2  A4
#define pin_y_joystick_2  A5
#define pin_button_joystick_2  47

#define pin_button_switch_1 44
#define pin_button_switch_2 45
#define pin_button_switch_3 23
#define pin_button_switch_4 25

#define pin_encoder_button  4
#define pin_encoder_s1  А8
#define pin_encoder_s2  А9

#define pin_ptentiometer_1 A2
#define pin_ptentiometer_2 A3

#define TFT_CS  12  // SS
#define TFT_RST 6
#define TFT_RS  8
#define TFT_SDI 11 // MOSI
#define TFT_CLK 13  // SCK
#define TFT_LED 10   // 0 if wired to +5V directly

#define TFT_BRIGHTNESS 120 // Initial brightness of TFT backlight (optional)

//--------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------
// УРОВЕНЬ МОЩНОСТИ ПЕРЕДАТЧИКА
// На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define SIG_POWER RF24_PA_MIN

// СКОРОСТЬ ОБМЕНА
// На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
// должна быть одинакова на приёмнике и передатчике!
// при самой низкой скорости имеем самую высокую чувствительность и дальность!!
// ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
#define SIG_SPEED RF24_1MBPS


//--------------------- БИБЛИОТЕКИ ----------------------
#include <SPI.h>
#include "TFT_22_ILI9225.h"
#include <EncButton.h>
#include "nRF24L01.h"
#include "RF24.h"

//--------------------- ПЕРЕМЕННЫЕ ----------------------
RF24 radio(9, 53); // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
//[0] - номер пакета передаваемых данных
//[1] - кнопка джостика 1 
int transmit_data[12];     // массив пересылаемых данных
int data[12];              // массив  данных
//int telemetry[12];         // массив принятых от приёмника данных телеметрии
byte rssi;
int trnsmtd_pack = 1, failed_pack;
unsigned long RSSI_timer;

Button button_1(pin_button_1);
Button button_2(pin_button_2);
Button button_3(pin_button_3);
Button button_4(pin_button_4);

Button button_joystick_1(pin_button_joystick_1);
Button button_joystick_2(pin_button_joystick_2);

Button button_switch_1(pin_button_switch_1);
Button button_switch_2(pin_button_switch_2);
Button button_switch_3(pin_button_switch_3);
Button button_switch_4(pin_button_switch_4);

Button encoder_button(pin_encoder_button);

int center_x_joystick_2 = 499; 
int center_y_joystick_2 = 513; 

int center_x_joystick_1 = 512; 
int center_y_joystick_1 = 509; 

enum elements{
    BUTTON_SWITCH_1,
    BUTTON_SWITCH_2,
    BUTTON_SWITCH_3,
    BUTTON_SWITCH_4,
    PTENTIOMETR_1,
    PTENTIOMETR_2, 
    JOYSTICK_1_BUTTON,
    JOYSTICK_1_X,
    JOYSTICK_1_Y,
    JOYSTICK_2_BUTTON,
    JOYSTICK_2_X,
    JOYSTICK_2_Y
};

TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK, TFT_LED,TFT_BRIGHTNESS);

//--------------------- ФУНКЦИИ ----------------------

void get_sensors_data() {
  
  data[BUTTON_SWITCH_1] = button_switch_1.read();
  data[BUTTON_SWITCH_2] = button_switch_2.read();
  data[BUTTON_SWITCH_3] = button_switch_3.read();
  data[BUTTON_SWITCH_4] = button_switch_4.read();


  data[PTENTIOMETR_1] = map(analogRead(pin_ptentiometer_1), 0, 1020, 0, 255);
  data[PTENTIOMETR_2] = map(analogRead(pin_ptentiometer_2), 0, 1020, 0, 255);

  if (button_joystick_1.tick()) {
    if (button_joystick_1.click()) {data[JOYSTICK_1_BUTTON] = (data[JOYSTICK_1_BUTTON] + 1) % 2;};
  }

  if (button_joystick_2.tick()) {
    if (button_joystick_2.click()) {data[JOYSTICK_2_BUTTON] = (data[JOYSTICK_2_BUTTON] + 1) % 2;};
  }

  data[JOYSTICK_1_X] = analogRead(pin_x_joystick_1); // считываем значение оси Х
  data[JOYSTICK_1_Y] = analogRead(pin_y_joystick_1); // считываем значение оси Y 

  data[JOYSTICK_2_X] = analogRead(pin_x_joystick_2); // считываем значение оси Х
  data[JOYSTICK_2_Y] = analogRead(pin_y_joystick_2); // считываем значение оси Y 

  convert_data_joystick();
}

void convert_data_joystick(){

  if (data[JOYSTICK_1_X] >= center_x_joystick_1 - 1 && data[JOYSTICK_1_X] <= center_x_joystick_1 + 1) {
    data[JOYSTICK_1_X] = 0;
  } else if (data[JOYSTICK_1_X] < center_x_joystick_1 - 1) {
    data[JOYSTICK_1_X] = map(data[JOYSTICK_1_X], 0, center_x_joystick_1 - 2, -60, 0);
  } else if (data[JOYSTICK_1_X] > center_x_joystick_1 + 1) {
    data[JOYSTICK_1_X] = map(data[JOYSTICK_1_X], center_x_joystick_1 + 2, 1022, 0, 60);
  };

  if (data[JOYSTICK_1_Y] >= center_y_joystick_1 - 1 && data[JOYSTICK_1_Y] <= center_y_joystick_1 + 1) {
    data[JOYSTICK_1_Y] = 0;
  } else if (data[JOYSTICK_1_Y] < center_y_joystick_1 - 1) {
    data[JOYSTICK_1_Y] = map(data[JOYSTICK_1_Y], 0, center_y_joystick_1 - 2, 60, 0);
  } else if (data[JOYSTICK_1_Y] > center_y_joystick_1 + 1) {
    data[JOYSTICK_1_Y] = map(data[JOYSTICK_1_Y], center_y_joystick_1 + 2, 1022, 0, -60);
  }; 

  if (data[JOYSTICK_2_X] >= center_x_joystick_2 - 1 && data[JOYSTICK_2_X] <= center_x_joystick_2 + 1) {
    data[JOYSTICK_2_X] = 0;
  } else if (data[JOYSTICK_2_X] < center_x_joystick_2 - 1) {
    data[JOYSTICK_2_X] = map(data[JOYSTICK_2_X], 0, center_x_joystick_2 - 2, 100, 0);
  } else if (data[JOYSTICK_2_X] > center_x_joystick_2 + 1) {
    data[JOYSTICK_2_X] = map(data[JOYSTICK_2_X], center_x_joystick_2 + 2, 1022, 0, -100);
  };

  if (data[JOYSTICK_2_Y] >= center_y_joystick_2 - 1 && data[JOYSTICK_2_Y] <= center_y_joystick_2 + 1) {
    data[JOYSTICK_2_Y] = 0;
  } else if (data[JOYSTICK_2_Y] < center_y_joystick_2 - 1) {
    data[JOYSTICK_2_Y] = map(data[JOYSTICK_2_Y], 0, center_y_joystick_2 - 2, -100, 0);
  } else if (data[JOYSTICK_2_Y] > center_y_joystick_2 + 1) {
    data[JOYSTICK_2_Y] = map(data[JOYSTICK_2_Y], center_y_joystick_2 + 2, 1022, 0, 100);
  };

}


void setup() {
  Serial.begin(9600); // открываем порт для связи с ПК
  radioSetup();
  //подклучаем чтение данных 
  pinMode(pin_x_joystick_1, INPUT);
  pinMode(pin_y_joystick_1, INPUT);
  pinMode(pin_x_joystick_2, INPUT);
  pinMode(pin_y_joystick_2, INPUT);
 
  pinMode(pin_ptentiometer_1, INPUT);
  pinMode(pin_ptentiometer_2, INPUT);
  //дисплей
  tft.begin();
  tft.clear();
  
  tft.setOrientation(2); 
  tft.setFont(Terminal6x8);

  delay(1000);

   
  tft.setFont(Terminal12x16);
  tft.drawText(45, 20, "HEXAPOD");
  delay(1000);
  tft.drawText(50, 50, "TikTok");
  delay(1000);
  tft.setFont(Terminal6x8);
  tft.drawText(23, 70, "@robotics_yankovich");
  delay(1000);
tft.clear();
}

void loop() {
  
  get_sensors_data();

  bool flag;

  for (int i=0; i < 12; i++){
    if (transmit_data[i] != data[i]) {
      transmit_data[i] = data[i];
      flag = true;
    }
  } 

  // отправка пакета transmit_data
  if (flag){
    
  //radio.powerUp();                    // начать работу 
  if (radio.write(&transmit_data, sizeof(transmit_data))) {
    trnsmtd_pack++;
    if (!radio.available()) {   // если получаем пустой ответ
    //Serial.println("pusto");
    } else {
      while (radio.available() ) {                    // если в ответе что-то есть
       // radio.read(&telemetry, sizeof(telemetry));    // читаем
      }
    }
  //radio.powerDown();
  } else {
    failed_pack++;
  }

  /*tft.setFont(Trebuchet_MS16x21);
  tft.drawText(48, 90, "1000");
  delay(1000);
  tft.setFont(Terminal6x8);
  tft.drawText(44, 120, "podpischikov");
  delay(1000);
  tft.setFont(Terminal12x16);
  tft.drawText(60, 140, "Wow!");
    delay(300);
  tft.drawText(60, 160, "Wow!");
    delay(300);
  tft.drawText(60, 180, "Wow!");
  */
  tft.drawText(5, 5, String(rssi) + "   " );
  tft.drawText(5, 25, String(data[BUTTON_SWITCH_1]) + " "  + data[BUTTON_SWITCH_2]+ " " + data[BUTTON_SWITCH_3]+ " " + data[BUTTON_SWITCH_4]);
  tft.drawText(5, 35, String(button_1.read()) + " "  + button_2.read()+ " " + button_3.read()+ " " + button_4.read());
  
  tft.drawText(5, 45, String(data[PTENTIOMETR_1]) + "    ");
  tft.drawText(5, 55, String(data[PTENTIOMETR_2]) + "    ");
  tft.drawText(5, 75, String(data[JOYSTICK_1_BUTTON]) + "    ");
  //tft.drawText(5, 75, String(data[JOYSTICK_2_BUTTON]) + "    ");
  tft.drawText(5, 95, String(data[JOYSTICK_1_X]) + "    ");
  tft.drawText(5, 105, String(data[JOYSTICK_1_Y]) + "    ");
  
  } 

  if (millis() - RSSI_timer > 1000) {    // таймер RSSI
    // расчёт качества связи (0 - 100%) на основе числа ошибок и числа успешных передач
    rssi = trnsmtd_pack *100 / (trnsmtd_pack + failed_pack);

    // сбросить значения
    failed_pack = 0;
    trnsmtd_pack = 0;
    RSSI_timer = millis();
  }

}

void radioSetup() {
  
  radio.begin();                      // активировать модуль
  radio.setAutoAck(1);                // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);            // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();           // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);           // размер пакета, в байтах
  radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
  radio.setChannel(CH_NUM);           // выбираем канал (в котором нет шумов!)
  radio.setPALevel(SIG_POWER);        // уровень мощности передатчика
  radio.setDataRate(SIG_SPEED);       // скорость обмена
                                      // должна быть одинакова на приёмнике и передатчике!
                                      // при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();                    // начать работу
  radio.stopListening();              // не слушаем радиоэфир, мы передатчик

}



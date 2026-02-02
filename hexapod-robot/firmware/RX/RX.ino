/*
   Пример скетча "управление с телеметрией", то есть модуль ПЕРЕДАТЧИК
   шлёт на ПРИЁМНИК команды управления, ПРИЁМНИК при получении пакета данных
   отправляет ПЕРЕДАТЧИКУ пакет телеметрии (какие-то свои данные). ПЕРЕДАТЧИК
   эти данные принимает. Вот такие пироги. Также в этом примере реализован
   расчёт RSSI (процент ошибок связи), на основании которого можно судить о
   качестве связи между модулями.
*/

// ЭТО СКЕТЧ ПРИЁМНИКА!!!

//--------------------- НАСТРОЙКИ ----------------------
#define CH_NUM 0x70   // номер канала (должен совпадать с передатчиком)

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
#include <VarSpeedServo.h> 
#include "nRF24L01.h"
#include "RF24.h"

//--------------------- ПЕРЕМЕННЫЕ ----------------------
RF24 radio(9, 53);   // "создать" модуль на пинах 9 и 10 для НАНО/УНО
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

int recieved_data[12];   // массив принятых данных
int data[12];       // массив данных телеметрии (то что шлём на передатчик)
int led_pins[6] = {2, 3, 4, 5, 6, 7}; //массив пинов светодиода

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

int counter_button_1 = 0;
int counter_button_2 = 0;
int counter_joystick_1_button = 0;

uint16_t coxa_length  = 45;
uint16_t femur_length = 100;
uint16_t tibia_length = 188;

uint16_t state_hexapod;
uint16_t action_hexapod;

VarSpeedServo servo_coxa_1,  servo_coxa_2,  servo_coxa_3,  servo_coxa_4,  servo_coxa_5,  servo_coxa_6, servo_femur_1, servo_femur_2, servo_femur_3, servo_femur_4, servo_femur_5, servo_femur_6, servo_tibia_1, servo_tibia_2, servo_tibia_3, servo_tibia_4, servo_tibia_5, servo_tibia_6;

enum type_servo{
    COXA,
    FEMUR,
    TIBIA
};

enum state{
  SIT,
  STAND,
  MOVEMENT
};

enum action_list{
  SIT_DOWN = 1,
  STAND_UP = 2,
  FORWARD = 3
};

struct servo_info_t{
  VarSpeedServo servo;
  int8_t pin;
  bool invert;
  int16_t zero_rotate;
  int16_t min_angle;
  int16_t max_angle; 
};

struct point_3d_t{
  float x;
  float y;
  float z;
};

struct limb_info_t{      
    servo_info_t servos[3];
    point_3d_t position; 
    point_3d_t start_position;
    point_3d_t final_position;
    bool point_reachable;    
    float angles[3]; 
    bool in_direction;
};

limb_info_t limbs[6] = {
  {
    {
      {servo_coxa_1,  22,  false,  55,  20,  160},
      {servo_femur_1,  24,  false,  0,  0,  131},
      {servo_tibia_1,  26,  false,  16,  0,  146}
    }
  }
  ,
  {
    {
      {servo_coxa_2,  23,  false,  0,  20,  160},
      {servo_femur_2,  25,  false,  0,  0,  131},
      {servo_tibia_2,  27,  false,  16,  0,  146}
    }
  }
  ,
  {
    {
      {servo_coxa_3,  28,  false,  -55,  20,  160},
      {servo_femur_3,  30,  false,  0,  0,  131},
      {servo_tibia_3,  32,  false,  16,  0,  146}
    }
  }
  ,
  {
    {
      {servo_coxa_4,  29,  false,  -125,  20,  160},
      {servo_femur_4,  31,  false,  0,  0,  131},
      {servo_tibia_4,  33,  false,  16,  0,  146}
    }
  }
  ,
  {
    {
      {servo_coxa_5,  34,  false, 180,  20,  160},
      {servo_femur_5,  36,  false,  0,  0,  131},
      {servo_tibia_5,  38,  false,  16,  0,  146}
    }
  }
  ,
  {
    {
      {servo_coxa_6,  35,  false, 125,  20,  160},
      {servo_femur_6,  37,  false,  0,  0,  131},
      {servo_tibia_6,  39,  false,  16,  0,  146}
    }
  }
};

point_3d_t base_position_sit[6] = {
  {80,115,-25},
  {140,0,-25},
  {80,-115,-25},
  {-80,-115,-25},
  {-140,0,-25},
  {-80,115,-25}
};

point_3d_t base_position_stand[6] = {
  
{74,106,-120},
  {110,0,-120},
  {74,-106,-120},
  {-74,-106,-120},
  {-110,0,-120},
  {-74,106,-120}
};

/*{75,106,-120},
  {120,0,-120},
  {75,-106,-120},
  {-75,-106,-120},
  {-120,0,-120},
  {-75,106,-120}*/

float n = 12;
float m = 1;
float r = 60;
int walking[6] = {0,1,0,1,0,1};
//--------------------- ФУНКЦИИ ----------------------

//////////////////////////////////////////////////////////
//Функция вычесляет углы для достижения заданной точки
//параметры: 
//  limb_info: структура с данными конесности
void kinematic_calculate_angles(limb_info_t &limb_info){ 

  limb_info.point_reachable = true; 

  int16_t coxa_zero_rotate_deg = limb_info.servos[COXA].zero_rotate;
  int16_t femur_zero_rotate_deg = limb_info.servos[FEMUR].zero_rotate;
  int16_t tibia_zero_rotate_deg = limb_info.servos[TIBIA].zero_rotate;

  float x = limb_info.position.x;
  float y = limb_info.position.y;
  float z = limb_info.position.z;

  //////////////////////////////////////////////////////////
  // Перемешение в систему координат сохи (X*, Y*, Z*)
  float coxa_zero_rotate_rad = radians(coxa_zero_rotate_deg);
  float x1 = x * cos(coxa_zero_rotate_rad) + y * sin(coxa_zero_rotate_rad);
  float y1 = -x * sin(coxa_zero_rotate_rad) + y * cos(coxa_zero_rotate_rad);
  float z1 = z;

  /////////////////////////////////////////////////////////
  // Рассчет угла COXA в радианах
  float coxa_angle_rad = atan2(y1, x1);

  /////////////////////////////////////////////////////////
  // Подготовка для расчета углов FEMUR и TIBIA
  x1 = hypot(x, y) - coxa_length;
    
  // Стороны треугольника 
  float c = hypot(x1, z1);
  float a = tibia_length;
  float b = femur_length;

  if (c > femur_length + tibia_length) {
    limb_info.point_reachable = false; // Точка не достижима
  }
    
  // Вычисление углов треугольника
  float fi    = atan2(z1, x1);
  float alpha = acos((b * b + c * c - a * a) / (2 * b * c));
  float gamma = acos((a * a + b * b - c * c) / (2 * a * b));

  /////////////////////////////////////////////////////////
  // Рассчет углов COXA, FEMUR, TIBIA в градусах 
  float coxa_angle_deg = 90 + degrees(coxa_angle_rad);
  float femur_angle_deg = (femur_zero_rotate_deg - degrees(alpha) - degrees(fi))*-1;
  float tibia_angle_deg = degrees(gamma) + tibia_zero_rotate_deg;
  //добавить проверку на инвертированые сервы 
  limb_info.angles[COXA] = 180 - coxa_angle_deg;
  limb_info.angles[FEMUR] = femur_angle_deg;
  limb_info.angles[TIBIA] = 180 - tibia_angle_deg;
    
  /////////////////////////////////////////////////////////
  // Проверка углов
  for (int i=0; i < 3; i++){
    if (isnan(limb_info.angles[i]) || limb_info.angles[i] < limb_info.servos[i].min_angle 
    || limb_info.angles[i] > limb_info.servos[i].max_angle) {
      limb_info.point_reachable = false;
    }
  } 
}
  
void kinematic_calculate(limb_info_t &limb_info){
  
  float x = ((n-m) * limb_info.start_position.x + m * limb_info.final_position.x) / (n);
  float y = ((n-m) * limb_info.start_position.y + m * limb_info.final_position.y) / (n);
  float z;

  if (limb_info.in_direction) { 
    float a = 180/n;
    float z_1 = abs((r * sin(radians(a*m))));
    z = limb_info.start_position.z + z_1;
    if (x < 0) {
    x = x - z_1;} else {x = x + z_1;}
  } else {
    z = ((n-m) * limb_info.start_position.z + m * limb_info.final_position.z) / (n);
  }

  limb_info.position.x = x;
  limb_info.position.y = y;
  limb_info.position.z = z; 

  // выводим в Serial Monitor
  //Serial.print(x);                       
  //Serial.print("\t");                    
  //Serial.print(y);
  //Serial.print("\t");                    
  //Serial.println(z);

}

void kinematic_write(limb_info_t &limb_info, int speed){
  
  limb_info.servos[COXA].servo.write(int(limb_info.angles[COXA]), speed);
  limb_info.servos[FEMUR].servo.write(int(limb_info.angles[FEMUR]), speed);
  limb_info.servos[TIBIA].servo.write(int(limb_info.angles[TIBIA]), speed);
  Serial.println(limb_info.angles[FEMUR]);  

}

void connect_servos(){
  
  for (int i=0; i < 6; i++){
    limbs[i].servos[COXA].servo.attach(limbs[i].servos[COXA].pin);
    limbs[i].servos[TIBIA].servo.attach(limbs[i].servos[TIBIA].pin);
    limbs[i].servos[FEMUR].servo.attach(limbs[i].servos[FEMUR].pin);
     delay(500);
  } 
  delay(2000);
  for (int i=0; i < 6; i++){
    limbs[i].position = base_position_sit[i];
    limbs[i].start_position = base_position_sit[i];
    limbs[i].final_position = base_position_stand[i];

    kinematic_calculate_angles(limbs[i]);
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], 30);
    }
  } 
}

void radioSetup() {             // настройка радио
  radio.begin();                // активировать модуль
  radio.setAutoAck(1);          // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);      // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();     // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     // размер пакета, байт
  radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
  radio.setChannel(CH_NUM);     // выбираем канал (в котором нет шумов!)
  radio.setPALevel(SIG_POWER);  // уровень мощности передатчика
  radio.setDataRate(SIG_SPEED); // скорость обмена
  // должна быть одинакова на приёмнике и передатчике!
  // при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();         // начать работу
  radio.startListening();  // начинаем слушать эфир, мы приёмный модуль
}

void sit_down() {
  
  if (action_hexapod != SIT_DOWN) {
    
    n = 12;
    m = 1;
    action_hexapod = SIT_DOWN;

    for (int i=0; i < 6; i++){
      limbs[i].in_direction = false;
      limbs[i].start_position = base_position_stand[i];
      limbs[i].final_position = base_position_sit[i];
    }
  }
  
  if (m > n) {
    return;
  }

  bool point = true;
  bool point1;
  bool point2;
  bool point3;

  bool point_reached = true;

  for (int i=0; i < 6; i++){

    kinematic_calculate(limbs[i]);
    kinematic_calculate_angles(limbs[i]);
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], 80);
    }

    point1 = limbs[i].servos[COXA].servo.isMoving();
    point2 = limbs[i].servos[FEMUR].servo.isMoving();
    point3 = limbs[i].servos[TIBIA].servo.isMoving(); 
    
    if (!point1 && !point2 && !point3) {
      point = false;
    }

    if (limbs[i].position.z != limbs[i].final_position.z) {
      point_reached = false;
    }
  }

  if (point_reached && !point){
    state_hexapod = SIT;
    if (recieved_data[JOYSTICK_1_BUTTON] != data[JOYSTICK_1_BUTTON])
      data[JOYSTICK_1_BUTTON] = recieved_data[JOYSTICK_1_BUTTON];
  } else if (!point){
    m+=1;
    
  } 
}

void stand_up() {
  
  if (action_hexapod != STAND_UP) {
    
    n = 12;
    m = 1;
    action_hexapod = STAND_UP;

    for (int i=0; i < 6; i++){
      limbs[i].in_direction = false;
      limbs[i].start_position = base_position_sit[i];
      limbs[i].final_position = base_position_stand[i];
    }
  }
  
  if (m > n) {
    return; 
  }

  bool point = true;
  bool point1;
  bool point2;
  bool point3;

  bool point_reached = true;

  for (int i=0; i < 6; i++){

    kinematic_calculate(limbs[i]);
    kinematic_calculate_angles(limbs[i]);
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], 80);
    }

    point1 = limbs[i].servos[COXA].servo.isMoving();
    point2 = limbs[i].servos[FEMUR].servo.isMoving();
    point3 = limbs[i].servos[TIBIA].servo.isMoving(); 
    
    if (!point1 && !point2 && !point3) {
      point = false;
    }

    if (limbs[i].position.z != limbs[i].final_position.z) {
      point_reached = false;
    }
  }

  if (point_reached && !point){
    state_hexapod = STAND;
    if (recieved_data[JOYSTICK_1_BUTTON] != data[JOYSTICK_1_BUTTON])
      data[JOYSTICK_1_BUTTON] = recieved_data[JOYSTICK_1_BUTTON];
  } else if (!point){
    m+=1;
  } 
}

void forward() {
  if (action_hexapod != FORWARD) {
    
    float y = data[JOYSTICK_1_Y]; 
    float x = data[JOYSTICK_1_X];

    int c = sqrt(y*y + x*x)/10;
    Serial.print(c); 
    n = c;
    m = 1;

    action_hexapod = FORWARD;
    state_hexapod = MOVEMENT;

    for (int i=0; i < 6; i++){
      if (walking[i] == 0) {
        walking[i] = 1;
      } else {
        walking[i] = 0;
      }
      limbs[i].start_position = limbs[i].position;    
      limbs[i].final_position = base_position_stand[i];
      //0 - против направления 1- по направлению 
      if (walking[i] == 0) {
        limbs[i].final_position.y = base_position_stand[i].y - data[JOYSTICK_1_Y];
        limbs[i].final_position.x = base_position_stand[i].x - data[JOYSTICK_1_X];
      } else if (walking[i] == 1) {
        limbs[i].final_position.y = base_position_stand[i].y + data[JOYSTICK_1_Y];
        limbs[i].final_position.x = base_position_stand[i].x + data[JOYSTICK_1_X];
      }
      limbs[i].in_direction = walking[i];
    }
  }
  
  if (m > n) {
    return; 
  }

  bool point = true;
  bool point1;
  bool point2;
  bool point3;

  bool point_reached = true;

  for (int i=0; i < 6; i++){

    kinematic_calculate(limbs[i]);
    kinematic_calculate_angles(limbs[i]);
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], recieved_data[PTENTIOMETR_1]);
    }

    point1 = limbs[i].servos[COXA].servo.isMoving();
    point2 = limbs[i].servos[FEMUR].servo.isMoving();
    point3 = limbs[i].servos[TIBIA].servo.isMoving(); 
    
    if (!point1 && !point2 && !point3) {
      point = false;
    }

    if (limbs[i].position.y != limbs[i].final_position.y) {
      point_reached = false;
    }
  }

  if (point_reached && !point){
    state_hexapod = STAND;
    action_hexapod = 0;
    for (int i=0; i < 12; i++){
    if (recieved_data[i] != data[i]) {
      data[i] = recieved_data[i];
      //flag = true;
    }
  }
  } else if (!point){
    m+=1;
    
  } 

}



void reversal() {

    float x = data[JOYSTICK_1_X];
    x = x/3;

  for (int i=0; i < 6; i++){
    limbs[i].angles[COXA] = 90 + x;  
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], recieved_data[PTENTIOMETR_1]);
    }
  }


  for (int i=0; i < 12; i++){
    if (recieved_data[i] != data[i]) {
      data[i] = recieved_data[i];  
  }
  } 
}

void preparation_for_movement() {
  if (action_hexapod != FORWARD) {

    n = 12;
    m = 1;
    action_hexapod = FORWARD;

    for (int i=0; i < 6; i++){
      if (walking[i] == 0) {
        walking[i] = 1;
      } else {
        walking[i] = 0;
      }
      limbs[i].start_position = limbs[i].position;    
      limbs[i].final_position = base_position_stand[i];
      //0 - против направления 1- по направлению 
      if (walking[i] == 0) {
        limbs[i].final_position.y = base_position_stand[i].y - 60;
      } else if (walking[i] == 1) {
        limbs[i].final_position.y = base_position_stand[i].y + 60;
      }
      limbs[i].in_direction = walking[i];
    }
  }
  
  if (m > n) {
    return; 
  }

  bool point = true;
  bool point1;
  bool point2;
  bool point3;

  bool point_reached = true;

  for (int i=0; i < 6; i++){

    kinematic_calculate(limbs[i]);
    kinematic_calculate_angles(limbs[i]);
    if (limbs[i].point_reachable){
      kinematic_write(limbs[i], 100);
    }

    point1 = limbs[i].servos[COXA].servo.isMoving();
    point2 = limbs[i].servos[FEMUR].servo.isMoving();
    point3 = limbs[i].servos[TIBIA].servo.isMoving(); 
    
    if (!point1 && !point2 && !point3) {
      point = false;
    }

    if (limbs[i].position.y != limbs[i].final_position.y) {
      point_reached = false;
    }
  }

  if (point_reached && !point){
    state_hexapod = STAND;
    action_hexapod = 0;
  } else if (!point){
    m+=1;
  } 

}



void setup() {
  
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(led_pins[i], OUTPUT);
  }
  radioSetup();
  connect_servos();
  delay(2000);

}


int recieved_pack = 0;
unsigned long connection_timer, connection_timer_lost; 
bool connection_lost, connection = true;

void connection_check(){

  if (millis() - connection_timer > 1000) {
    // сбросить значения
    recieved_pack = 0;
    connection_timer = millis();
    if (recieved_pack != 0) {
      connection_timer_lost = millis();
    }
  }

  if (recieved_pack == 0) {
    if (millis() - connection_timer_lost > 5000) {
      connection_lost = true;
    }

  } else {
    connection_lost = false;
  }

}


void LED_control(){

  if (connection_lost) {
    for (int i = 0; i < 6; i++) {
      analogWrite(led_pins[i], 0); 
    }
  } else {
    for (int i = 0; i < 6; i++) {
      analogWrite(led_pins[i], recieved_data[PTENTIOMETR_2]); 
    }
  }

}

void loop() {         
  
  //////////////////////////////////////////////////////////
  // получаем данные с приемника
  ////////////////////////////////////////////////////////// 
  while (radio.available(&pipeNo)) {                    // слушаем эфир
    radio.read(&recieved_data, sizeof(recieved_data));  // чиатем входящий сигнал
    recieved_pack++;  
  }

  //////////////////////////////////////////////////////////
  // вычисляем есть ли связь
  //////////////////////////////////////////////////////////
  connection_check();

  // connection_lost - параметр потели связи

  //////////////////////////////////////////////////////////
  // управление светодиодами
  //////////////////////////////////////////////////////////
  LED_control(); 



  if (data[JOYSTICK_1_BUTTON] == 0 && state_hexapod == STAND) {
    sit_down();
  } else if (data[JOYSTICK_1_BUTTON] == 1 && state_hexapod == SIT) {
    stand_up();
  } else if (data[BUTTON_SWITCH_1] == 1 && (data[JOYSTICK_1_X] != 0 || data[JOYSTICK_1_Y] != 0) && state_hexapod == STAND) {
   //preparation_for_movement();
    forward();
  } else if (data[BUTTON_SWITCH_1] == 1 && (data[JOYSTICK_1_X] != 0 || data[JOYSTICK_1_Y] != 0) && state_hexapod == MOVEMENT) {
   forward();
  } else if (data[BUTTON_SWITCH_2] == 1 && state_hexapod == STAND) {
    reversal();
  }
  
   else {  
    for (int i=0; i < 12; i++){
    if (recieved_data[i] != data[i]) {
      data[i] = recieved_data[i];
    }
  } }
  
}

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h> 

const byte rxPin = 3; //rx2
const byte txPin = 1; //tx2
HardwareSerial dwin(1);
bool tool = false;
bool hill = false;

// inputs to esp32;
#define Rswitch 12
#define Lswitch 13
#define head 14
#define break_switchin 23
#define cruise_switchin 2
#define key_switchin 33
#define stand_switch 35
#define current_sensor 36
#define voltage_sensor 39
#define odometer 16
#define speedometer 17
#define echo_pin 5
#define Ignition_switchIn 27

// outputs from esp32;
#define break_servo 25
#define low_breakout 15
#define trig_pin 4
#define power_switch 18
#define cruise_switchout 19
#define LCD_PWR_switch 26
#define Ignition_switchOut 32


// hilll hold servo control;
#define DEGREE_TO_PULSE(degree) ((degree) * 660)

//password entry and exit;
unsigned char Password[15] = {0x5A, 0xA5, 0x0C, 0x83, 0x10, 0x00, 0x04, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0xFF, 0xFF};
unsigned char pass_check[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01};
unsigned char Off_command[9] =  {0x5A, 0xA5, 0x06, 0x83, 0x75, 0x00, 0x01, 0x00, 0x00};

//dashboard info;
unsigned char trip[8] =            {0x5a, 0xa5, 0x05, 0x82, 0x30 , 0x00, 0, 0x00};
unsigned char Speed[8] =           {0x5a, 0xa5, 0x05, 0x82, 0x35, 0x00, 0, 0x00};
unsigned char range[8] =           {0x5a, 0xa5, 0x05, 0x82, 0x40, 0x00, 0, 0x00};
unsigned char battery[8] =         {0x5a, 0xa5, 0x05, 0x82, 0x45, 0x00, 0, 0x00};

// indicator info;
unsigned char Right_indicator[8] = {0x5a, 0xa5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00};
unsigned char Stand[8] =           {0x5a, 0xa5, 0x05, 0x82, 0x55, 0x00, 0x00, 0x00};
unsigned char Headlight[8] =       {0x5a, 0xa5, 0x05, 0x82, 0x60, 0x00, 0x00, 0x00};
unsigned char Left_indicator[8] =  {0x5a, 0xa5, 0x05, 0x82, 0x65, 0x00, 0x00, 0x00};
unsigned char Cruise[8] =          {0x5a, 0xa5, 0x05, 0x82, 0x70, 0x00, 0x00, 0x00};

//variables;
int ADXL345 = 0x53;
int total_dist,velocity,battery_indi,total_range,duration,distanceCm, ct=0;
int count,start_time,end_time,vel_volt,volt_cal,curr_cal;
float X_out, Y_out, Z_out, soh ,tilt1;  

void setup()
{
  pinMode(Rswitch,INPUT_PULLUP);
  pinMode(Lswitch,INPUT_PULLUP);
  pinMode(head,INPUT_PULLUP);
  pinMode(break_switchin,INPUT_PULLUP);
  pinMode(cruise_switchin,INPUT_PULLUP);
  pinMode(key_switchin,INPUT_PULLUP);
  pinMode(stand_switch,INPUT_PULLUP);
  pinMode(current_sensor,INPUT);
  pinMode(voltage_sensor,INPUT);
  pinMode(speedometer,INPUT);
  pinMode(echo_pin,INPUT);
  pinMode(Ignition_switchIn,INPUT);
  
  pinMode(break_servo,OUTPUT);
  pinMode(low_breakout,OUTPUT);
  pinMode(trig_pin,OUTPUT);
  pinMode(power_switch,OUTPUT);
  pinMode(cruise_switchout,OUTPUT);
  pinMode(LCD_PWR_switch,OUTPUT);
  pinMode(Ignition_switchOut,OUTPUT);

  digitalWrite(LCD_PWR_switch,HIGH);
  digitalWrite(power_switch,HIGH);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,1);
  digitalWrite(LCD_PWR_switch,LOW);
  Serial.begin(115200);
  //begin i2c communication with adxl
  Wire.begin(); 
  Wire.beginTransmission(ADXL345); 
  Wire.write(0x2D);
  Wire.write(8); 
  Wire.endTransmission();
  delay(10);
  //Begin serial communication DWIN
  dwin.begin(115200, SERIAL_8N1, rxPin, txPin);
  dwin.write(Right_indicator, 8);
  dwin.write(Stand, 8);
  dwin.write(Headlight, 8);
  dwin.write(Left_indicator, 8);
  dwin.write(Cruise, 8);
}

void loop() 
{ 
  if(digitalRead(stand_switch) == LOW)
  digitalWrite(LCD_PWR_switch,LOW);
// starting conditions to start the vehicle;
  if (tool == false)
  {
   on(); 
   if(digitalRead(key_switchin)== LOW && ct == 0)
  {
    ct =1;
    dwin.write(pass_check, 10);
    digitalWrite(power_switch,LOW);
    if(digitalRead(Ignition_switchIn)== HIGH)
    digitalWrite(Ignition_switchOut,HIGH);
    tool = true;
  }
  }
  else
  {
    dashboard_data();//dashboard data;
    indicators();//indicators;
  }
  
// speed calculations
  vel_volt = (analogRead(speedometer)*(1000000+146000))/146000;
  velocity = 25; //vel_volt * 3.34;

// odometer calculations
  attachInterrupt(digitalPinToInterrupt(odometer),counter,RISING);

// battery level calculations
  volt_cal = (analogRead(voltage_sensor)*(1000000+55000))/55000;
  battery_indi = 50;// 100-((volt_cal/8.2)*100);

// Range calculations
  curr_cal = (analogRead(current_sensor)*5)/4096;
  total_range = 100;//((battery_indi*46*velocity)/((curr_cal-2.5)/0.185)*100);
  
// Cruise mode operations;
  if(digitalRead(cruise_switchin)==LOW)
  {
    Cruise[7] = 0x01;
    dwin.write(Cruise, 8);
    digitalWrite(cruise_switchout,HIGH);
    dwin.write(Cruise, 8); 
  }
  else
  {
    Cruise[7] = 0x00;
    dwin.write(Cruise, 8);
    digitalWrite(cruise_switchout,LOW);
  }
//hill hold code;
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); 
  Wire.endTransmission();
  Wire.requestFrom(ADXL345, 6, true); 
  X_out = ( Wire.read()| Wire.read() << 8); 
  X_out = X_out/256; 
  Y_out = ( Wire.read()| Wire.read() << 8);
  Y_out = Y_out/256;
  Z_out = ( Wire.read()| Wire.read() << 8);
  Z_out = Z_out/256; 
  //soh = Y_out/Z_out;
  //tilt1 = atan(soh)*114.592;
  if(digitalRead(break_switchin)== LOW && X_out>= 12)
  {
    start_time = millis();
    if(digitalRead(break_switchin)== LOW && X_out>= 12 )
    {
    end_time = millis();
    if((end_time - start_time)>=2000)
    {
      digitalWrite(break_servo, HIGH);
      delayMicroseconds(118620);
      digitalWrite(break_servo, LOW);
      delayMicroseconds(20000 - 118620);
    }
    if((end_time - start_time)<=2000)
    {
      digitalWrite(break_servo, HIGH);
      delayMicroseconds(0);
      digitalWrite(break_servo, LOW);
      delayMicroseconds(20000);
    }
    }
  }
  
//rear break;
if(digitalRead(break_switchin)== LOW)
{
  digitalWrite(break_servo, HIGH);
  delayMicroseconds(118620);
  digitalWrite(break_servo, LOW);
  delayMicroseconds(20000 - 118620);
}
else
{
  digitalWrite(break_servo, HIGH);
  delayMicroseconds(0);
  digitalWrite(break_servo, LOW);
  delayMicroseconds(20000 - 0);
}

// counter value update and switch off
if(digitalRead(key_switchin)==HIGH || digitalRead(stand_switch)== LOW)
{
  EEPROM.write(0, count);
  EEPROM.commit();
  digitalWrite(LCD_PWR_switch,HIGH);
  digitalWrite(power_switch,HIGH);
  digitalWrite(Ignition_switchOut,LOW);
  esp_deep_sleep_start();
}
}

//counter to count distance traveled;
void counter()
{
  count = EEPROM.read(0);
  count++;
  total_dist = (count*1.44)/2;
}

// display on commands;
void on(){
  if (dwin.available() >= 15) 
  {
    bool match = true;
    for (int i = 0; i < 15; i++) 
    {
      if (dwin.read() != Password[i]) 
      {
        match = false;
        break;
      }
    }
    if (match) 
    {
      dwin.write(pass_check, 10);
      digitalWrite(power_switch,LOW);
      if(digitalRead(Ignition_switchIn)== HIGH)
        digitalWrite(Ignition_switchOut,HIGH);
      tool = true;
    }
  }
}

void dashboard_data()
{  
  trip[6] = highByte(total_dist);
  trip[7] = lowByte(total_dist);
  dwin.write(trip, 8);
 
  Speed[6] = highByte(velocity);
  Speed[7] = lowByte(velocity);
  dwin.write(Speed, 8);
 
  range[6] = highByte(total_range);
  range[7] = lowByte(total_range);
  dwin.write(range, 8);
 
  battery[6] = highByte(battery_indi);
  battery[7] = lowByte(battery_indi);
  dwin.write(battery, 8);
}

void  indicators()
{ 
  if(digitalRead(Rswitch)==LOW)
  {
    Right_indicator[7] = 0x01;
    dwin.write(Right_indicator, 8);
  }
  else
  {
    Right_indicator[7] = 0x00;
    dwin.write(Right_indicator, 8);
  }
  if(digitalRead(Lswitch)==LOW)
  {
    Left_indicator[7] = 0x01;
    dwin.write(Left_indicator, 8);
  }
  else
  {
    Left_indicator[7] = 0x00;
    dwin.write(Left_indicator, 8);
  }
  if(digitalRead(stand_switch)==LOW)
  {
    Stand[7] = 0x01;
    dwin.write(Stand, 8);
  }
  else
  {
    Stand[7] = 0x00;
    dwin.write(Stand, 8);
  }
  if(digitalRead(head)==LOW)
  {
    Headlight[7] = 0x01;
    dwin.write(Headlight, 8);
  }
  else
  {
    Headlight[7] = 0x00;
    dwin.write(Headlight, 8);
  }
}

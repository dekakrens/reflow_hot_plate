#include <Thermistor.h> 
#include <NTC_Thermistor.h>
#include <SmoothThermistor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>

// Thermistor
#define SENSOR_PIN          A0
#define REF_RESISTANCE      4700
#define NOMINAL_RESISTANCE  87900
#define NOMINAL_TEMPERATURE 26.5
#define B_VALUE             3950
#define SMOOTHING_FACTOR    5

// Rotary Encoder
#define clk_pin 4
#define dt_pin 5
OneButton btn = OneButton(6, true);

// Instance
Thermistor* thermistor = NULL;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// IO
int but_1 = 12;
int but_2 = 11;
int but_3 = 10; 
int but_4 = 9;
int SSR = 3;
int buzzer = 7;

// Varialbles
unsigned int millis_lcd, millis_pid;            //We use these to create the loop refresh rate
unsigned int millis_now;
float refresh_rate = 500;                       //LCD refresh rate. You can change this if you want
float pid_refresh_rate  = 50;                   //PID Refresh rate
float seconds = 0;                              //Variable used to store the elapsed time                   
int mode = 0;                                   //We store the running selected mode here
int selected_mode = 0;                          //Selected mode for the menu
int max_modes = 2;                              //For now, we only work with 1 mode...
float temperature = 0;                          //Store the temperature value here
float preheat_setpoint = 140;                    //Mode 1 preheat ramp value is 140-150ºC
float soak_setpoint = 150;                       //Mode 1 soak is 150ºC for a few seconds
float reflow_setpoint = 200;                    //Mode 1 reflow peak is 200ºC
float temp_setpoint = 0;                        //Used for PID control
float pwm_value = 255;                          //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float MIN_PID_VALUE = 0;
float MAX_PID_VALUE = 180;                      //Max PID value. You can change this. 
float cooldown_temp = 40;                       //When is ok to touch the plate
unsigned int last_debounce = 0;                 //Store rotary debounce
unsigned int debounce_delay = 0.01;             //Debouncing rotary encoder
int prev_clk, prev_data;                        //Used by rotary encoder
String state;                                   //Store rotary state
boolean button_state = false;                   //Store button state

// PID Variables
float Kp = 2;               //Mine was 2
float Ki = 0.0025;          //Mine was 0.0025
float Kd = 9;               //Mine was 9
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;

void handleClick() {
  button_state = true;
}

void setup() {
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, HIGH);
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer, LOW);

  prev_clk = digitalRead(clk_pin);
  prev_data = digitalRead(dt_pin);
  btn.attachClick(handleClick);

  Thermistor* originThermistor = new NTC_Thermistor(
    SENSOR_PIN,
    REF_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE
  );
  thermistor = new SmoothThermistor(originThermistor, SMOOTHING_FACTOR);

  lcd.init();
  lcd.setBacklight(HIGH);
  tone(buzzer, 1800, 200);
  millis_lcd = millis();
  millis_now = millis();
}


void checkRotary() {
  if((prev_clk==1) && (prev_data==0)){
    if((digitalRead(clk_pin) == 0) && (digitalRead(dt_pin) == 1)){
      state="NEXT";
    }
    if((digitalRead(clk_pin) == 0) && (digitalRead(dt_pin) == 0)){
      state="PREV";
    }
  }

  if((prev_clk==1) && (prev_data==1)){
    if((digitalRead(clk_pin) == 0) && (digitalRead(dt_pin) == 1)){
      state="NEXT";
    }
    if((digitalRead(clk_pin) == 0) && (digitalRead(dt_pin) == 0)){
      state="PREV";
    }
  }
}

void coolingDown() {
  if(temperature < cooldown_temp) {
    mode = 0;
    tone(buzzer, 1000, 100);
  }
  digitalWrite(SSR, HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T: ");
  lcd.print(temperature,1);
  lcd.setCursor(9,0);
  lcd.print("SSR OFF");
  lcd.setCursor(0,1);
  lcd.print("    COOLDOWN    ");
}

void initial() {
  digitalWrite(SSR, HIGH);
  lcd.clear();
  lcd.setCursor(0,0);     
  lcd.print("T: ");
  lcd.print(temperature,1);   
  lcd.setCursor(9,0);      
  lcd.print("SSR OFF"); 
    
  lcd.setCursor(0,1);
  switch (selected_mode) {
    case 0:
      lcd.print("Select Mode");
      break;
    case 1:
      lcd.print("Reflow");
      break;
    case 2:
      lcd.print("Heater");
      break;
  }
}

void reflow() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T: ");
  lcd.print(temperature,1);
  lcd.setCursor(9,0);
  lcd.print("SSR ON");

  lcd.setCursor(0,1);
  lcd.print("S");  lcd.print(temp_setpoint,0);
  lcd.setCursor(5,1);
  lcd.print("PWM");  lcd.print(pwm_value,0);
  lcd.setCursor(12,1);
  lcd.print(seconds,0);
  lcd.print("s");
}

void loop() {
  btn.tick();                                          //Reading the button
  delay(10);
  if((millis() - last_debounce)>debounce_delay) {
    checkRotary();
    prev_clk = digitalRead(clk_pin);
    prev_data = digitalRead(dt_pin);
    last_debounce = millis();
  }

  millis_now = millis();
  if(millis_now - millis_pid > pid_refresh_rate){         //Refresh rate of the PID
    millis_pid = millis(); 
    
    temperature = thermistor->readCelsius();
    
    if(mode == 1){
      if(temperature < preheat_setpoint){
        temp_setpoint = seconds*1.666;                    //Reach 150ºC till 90s (150/90=1.666)
      }
        
      if(temperature > preheat_setpoint && seconds < 90){
        temp_setpoint = soak_setpoint;               
      }
        
      else if(seconds > 90 && seconds < 110){
        temp_setpoint = reflow_setpoint;                 
      }
       
      //Calculate PID
      PID_ERROR = temp_setpoint - temperature;
      PID_P = Kp*PID_ERROR;
      PID_I = PID_I+(Ki*PID_ERROR);      
      PID_D = Kd * (PID_ERROR-PREV_ERROR);
      PID_Output = PID_P + PID_I + PID_D;

      //Define maximun PID values
      if(PID_Output > MAX_PID_VALUE){
        PID_Output = MAX_PID_VALUE;
      }
      else if (PID_Output < MIN_PID_VALUE){
        PID_Output = MIN_PID_VALUE;
      }

      pwm_value = 255 - PID_Output;         //Invert pwm value
      analogWrite(SSR,pwm_value);           //We change the Duty Cycle applied to the SSR
      
      PREV_ERROR = PID_ERROR;
      
      if(seconds > 130){
        digitalWrite(SSR, HIGH);
        temp_setpoint = 0;
        lcd.clear();
        lcd.setCursor(0,1);     
        lcd.print("REFLOW COMPLETED");
        tone(buzzer, 1800, 1000);    
        seconds = 0;
        mode = 11;
        delay(3000);
      }
    }
  }

  millis_now = millis();
  if(millis_now - millis_lcd > refresh_rate){             //Refresh rate of prntiong on the LCD
    millis_lcd = millis();   
    seconds = seconds + (refresh_rate/1000);              //We count time in seconds
    
    switch (mode) {
      case 0:
        initial();
        break;
      case 11:
        coolingDown();
        break;
      case 1:
        reflow();
        break;
    }
  }

  if(state == "NEXT"){
    state = "STOP";
    selected_mode ++;   
    tone(buzzer, 2300, 40);  
    if(selected_mode > max_modes){
      selected_mode = 0;
    }
  }

  if(state == "PREV"){
    state = "STOP";
    selected_mode --;   
    tone(buzzer, 2300, 40);  
    if(selected_mode < 0){
      selected_mode = 2;
    }
  }

  if(button_state){
    if(mode == 1){
      digitalWrite(SSR, HIGH);
      mode = 0;
      selected_mode = 0; 
      tone(buzzer, 2500, 150);
      delay(130);
      tone(buzzer, 2200, 150);
      delay(130);
      tone(buzzer, 2000, 150);
      delay(130);
    }
    
    if(selected_mode == 0){
      mode = 0;
    }
    else if(selected_mode == 1){
      mode = 1;
      tone(buzzer, 2000, 150);
      delay(130);
      tone(buzzer, 2200, 150);
      delay(130);
      tone(buzzer, 2400, 150);
      delay(130);
      seconds = 0;
    }
    button_state = false;
  }
}
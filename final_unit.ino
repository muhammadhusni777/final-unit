#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include  <TimerOne.h>          
LiquidCrystal_I2C lcd(0x27, 16, 2);
#include <avr/wdt.h>


volatile int i=0;               
volatile boolean zero_cross=0;  
int AC_pin = 11;                
int dim;                    
int inc=1;                      

int freqStep = 75;    

int setpoint;
float sensor;
float sensor_raw;
float sum;
float error;
float P_control;
float kp = 3;// 1) 6.5 2) 3   3)3.4
float I_control;
float ki = 0.01; // 1) 0.023 2)0.01   3)0.015 
float PI_control;
int i_windup;
int saturation = 128;
int power;

float Time;
float elapsedTime;
float timePrev;

void setup() {                                      // Begin setup
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  
  pinMode(AC_pin, OUTPUT);                          // Set the Triac pin as output
  attachInterrupt(0, zero_cross_detect, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  Timer1.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(dim_check, freqStep);      
  Serial.begin(9600);           
  wdt_enable(WDTO_2S);                                 
}

void zero_cross_detect() {    
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  i=0;
  digitalWrite(AC_pin, LOW);       // turn off TRIAC (and AC)
}                                 

// Turn on the TRIAC at the appropriate time
void dim_check() {                   
  if(zero_cross == true) {              
    if(i>=dim) {                     
      digitalWrite(AC_pin, HIGH);        
      i=0;                       
      zero_cross = false; 
    } 
    else {
      i++;                      
    }                                
  }                                  
}                                   

void loop() {                        
    
 //setpoint = 50;
 setpoint = map(analogRead(A0), 0, 1023, 37, 44);
   //read sensor
   
  for (int i=1; i<=50; i++){
        //sensor_raw = ((0.1391*analogRead(A1))-46.893);
        sensor_raw = ((0.1155*analogRead(A1))-16.755);
        sum = sum + sensor_raw;
      }
      
 sensor = sum/50;
 sum =0;
   //error
 error = setpoint - sensor;
   //proportional control
 P_control = kp * error;

   //integral control
 I_control = ki*error*elapsedTime + I_control;
  
   //integral windup
  if(I_control > i_windup){
    I_control = i_windup;
  } else if (I_control < 0){
    I_control = 0; 
  }
  else {
    I_control = I_control;
  }
  //p+i control
 PI_control = P_control + I_control;
  
   //saturation
  if (PI_control > saturation){
    PI_control = saturation;
    i_windup = saturation-PI_control;
  } else if(PI_control < 0) {
     PI_control = 0;
  }
  else{
    i_windup = saturation;
  }    
  
  //dim = 0; //debug  
  dim = saturation - PI_control;
  power = map(dim, 128, 0, 0, 100);
  
  
  
  //display
  lcd.setCursor (0,0);
  lcd.print("Power : ");
  if(power < 10){
  lcd.setCursor (7,0);
  lcd.print("  ");  
  lcd.setCursor (9,0);
  lcd.print(power); 
  }
    if(power >= 10 && power < 100){
    lcd.setCursor (7,0);
    lcd.print(" ");  
    lcd.setCursor (8,0);
    lcd.print(power);  
    }
     if(power == 100){
    lcd.setCursor (7,0);
    lcd.print(power);  
    }
    lcd.setCursor (11,0);
    lcd.print("%");
    lcd.setCursor (0,1);
    lcd.print("SP: ");
    lcd.setCursor(4,1);
    lcd.print(setpoint);
     lcd.setCursor (7,1);
    lcd.print("PV: ");
    lcd.setCursor(11,1);
    lcd.print(sensor,1);

    //TIMER
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 

  //serial debug
  Serial.print("setpoint : ");
  Serial.print(setpoint);
  Serial.print(" sensor : ");
  Serial.print(sensor);
  Serial.print(" error : ");
  Serial.print(error);
  Serial.print(" P : ");
  Serial.print(P_control);
  Serial.print(" I : ");
  Serial.print(I_control);
  Serial.print(" PI : ");
  Serial.print(PI_control);
  Serial.print(" power : ");
  Serial.println(power);
  delay(100);  
  wdt_reset(); 
}

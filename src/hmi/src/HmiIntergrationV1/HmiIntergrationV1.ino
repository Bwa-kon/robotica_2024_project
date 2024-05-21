// Includes and LCD configuration of LCD adress
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ros.h>
#include <std_msgs/String.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Press buttons and emergency
const int Emergency = 2; 
const int StartButton = 4; 
const int StopButton = 5; 
const int ResetButton = 6; 

// Cycle switch
const int Switch1 = 7; 
const int Switch2 = 8;

// Status lamp
const int Green = 56; 
const int Orange = 55;  // Can also be yellow
const int Red = 54;  

// Leds
const int Led1 = 9;
const int Led2 = 10;
const int Led3 = 11;

// Delay constant variables
int dt = 800;
int dt2 = 100;

std_msgs::String msg;

const int ScrollInterval = 1000;  // Scroll interval in milliseconds
int lineCounter = 0;
unsigned long previousScrollTime = 0;

int a, b, c, d, e, f, g, h;

String lines[] = { "Bak 1: " + String(a),"Bak 2: " + String(b),
  "Bak 3: " + String(c), "Bak 4: " + String(d), "Bak 5: " + String(e),
  "Bak 6: " + String(f), "Bak 7: " + String(g), "Bak 8: " + String(h)};

void setup()
{
  // ALL inputs configurations
  pinMode(Emergency, INPUT_PULLUP);     
  pinMode(StartButton, INPUT_PULLUP);   
  pinMode(StopButton, INPUT_PULLUP);    
  pinMode(ResetButton, INPUT_PULLUP);   
  pinMode(Switch1, INPUT_PULLUP);       // 1-time cycle
  pinMode(Switch2, INPUT_PULLUP);       // repeating cycle
  
  // ALL output configuration
  pinMode(Green, OUTPUT);               // Green status light
  pinMode(Orange, OUTPUT);              // Orange status light
  pinMode(Red, OUTPUT);                 // Red status light
  pinMode(Led1, OUTPUT);                // Led 1 (red for 1-time cycle)
  pinMode(Led2, OUTPUT);                // Led 2 (red for repeating cycle)
  pinMode(Led3, OUTPUT);                // Led 3 (yellow for actuator busy)

  lcd.begin();
  lcd.backlight();
  lcd.noBlink();
  Serial.begin(9600);                   // Initialize the serial communication

 // Show that the third test will start
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Booting");
  lcd.setCursor(0, 1); lcd.print("Instalation");

  while (digitalRead(ResetButton) == HIGH)
  {    
    // Blink Green light
    digitalWrite(Green, HIGH); 
    delay(dt);
    digitalWrite(Green, LOW);
    delay(dt);
  }

  digitalWrite(Green, HIGH);
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Ready to start");

  delay(3000);
}

void loop() 
{
  if (digitalRead(Emergency) == HIGH) 
  {
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Emergency button");
    lcd.setCursor(0, 1); lcd.print("is pressed");
    analogWrite(Orange, 255);
    analogWrite(Red, 0);  
    digitalWrite(Green, HIGH);
    
    while (digitalRead(Emergency) == HIGH) {delay(dt2);} 
    
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("No more");
    lcd.setCursor(0, 1); lcd.print("Emergencies");
    analogWrite(Orange, 0);
    analogWrite(Red, 0);  

    delay(2000);
    
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Press any button");
    lcd.setCursor(0, 1); lcd.print("to continue");
  } 

  if (digitalRead(Switch1) == LOW) {
    digitalWrite(Led1, HIGH);
    digitalWrite(Led2, LOW);
  } 

  if (digitalRead(Switch2) == LOW) {
    digitalWrite(Led1, LOW);
    digitalWrite(Led2, HIGH);
  }   

  if (digitalRead(StopButton) == HIGH) {
    digitalWrite(Green, LOW);
    analogWrite(Orange, 0);
    analogWrite(Red, 255);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Stop is pressed");
    delay(dt2);
  }

  if (digitalRead(ResetButton) == LOW) {
    digitalWrite(Green, LOW);
    analogWrite(Orange, 0);
    analogWrite(Red, 0);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Reset is pressed");
    delay(dt2);
  } 

 if (digitalRead(StartButton) == LOW) {
    digitalWrite(Green, LOW);
    analogWrite(Orange, 255);
    analogWrite(Red, 0);
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Start is pressed");
    delay(dt2);
    }

/*
 *  Place Error and faulth problems files 
 *  
 *  Place increment code
 * 
 */
 
  lcd.clear();

  // Print two lines simultaneously
  lcd.setCursor(0, 0);
  lcd.print(lines[lineCounter]);

  if (lineCounter + 1 < sizeof(lines) / sizeof(lines[0])){
    lcd.setCursor(0, 1);
    lcd.print(lines[lineCounter + 1]);
  }

  lineCounter += 2;
  
  if (lineCounter >= sizeof(lines) / sizeof(lines[0])){
    lineCounter = 0;
  }

  delay(dt); // Delay before scrolling to the next lines
}

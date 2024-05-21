/*  In this progam there will be a loop that circulates trough the whole HMI.
The HMI consists of 3 buttons, 1 switch and a emergency button.
Also a LCD-display will be active and there are 5 LED. 
Although two leds will be used for feed switches, they will be connected with his program to test there working (stability).

Working: The programa will start with including the headerfiles and libaries,
consisting of the LCD I2C file and the basic commands of arduino with in- and outputs
After that the SETUP loop will configurate the in and output pins and ports.
Followig with the voidloop that goes trough all the components of the HMI.
Order of test:
- Buttons from left to right starting with the start, ending with the emergency
- Status lights left to right
- LEDS left to right

trough the whole text the LCD will give feedback of the working or the active testpart.

*/

// Includes and LCD configuration of LCD adress
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Press buttons and emergency
const int Noodstop = 2; 
const int Startknop = 4; 
const int Stopknop = 5; 
const int Resetknop = 6; 

// Cycle switch
const int Schakelaar1 = 7; 
const int Schakelaar2 = 8;

// Status lamp
const int Groen = 56; 
const int Oranje = 55; 
const int Rood = 54;  

// Pin numbers for the LEDs
int leds[] = {9, 10, 11, 12, 13};
int numLeds = sizeof(leds) / sizeof(leds[0]);

// Time delay between LED state changes (in milliseconds)
int delayTime = 1500;
int dt = 1000;
int dt2 = 2000;

void setup() 
{

  // ALL inputs configurations
  pinMode(Noodstop, INPUT_PULLUP); // Noodstop
  pinMode(Startknop, INPUT_PULLUP); // startknop
  pinMode(Stopknop, INPUT_PULLUP); // stopknop
  pinMode(Resetknop, INPUT_PULLUP); // resetknop
  pinMode(Schakelaar1, INPUT_PULLUP); // enkel
  pinMode(Schakelaar2, INPUT_PULLUP); // cyclus
  
  // ALL Status output configuration
  pinMode(Groen, OUTPUT); // groene lamp
  pinMode(Oranje, OUTPUT); // oranje lamp
  pinMode(Rood, OUTPUT); // rode lamp
  
  // LCD and serial communication config.
  lcd.begin();         // initialize the lcd
  lcd.backlight();    // Turn on the LCD screen backlight
  Serial.begin(9600); // Initialize the serial communication

  // Initialize all LED pins as outputs
  for (int i = 0; i < numLeds; i++) {
  pinMode(leds[i], OUTPUT);
  }

}

void loop() 
{
  // Opening print
  lcd.setCursor(0, 0); lcd.print("Running"); 
  lcd.setCursor(0, 1); lcd.print("interface test");
  
  delay(2000);

  // Print first test
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("First");
  lcd.setCursor(0, 1); lcd.print("Input test");

  delay(dt);

  // Wait for start
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Press start!");
  while (digitalRead(Startknop) == HIGH) {delay(1);}
  
  // Show that it works
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Start works!");
  delay(dt);
  
  // Wait for stop
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Press stop!");
  while (digitalRead(Stopknop) == LOW) {delay(1);}

  // Show that stop works
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Stop works!");
  delay(dt);

  // Wait for reset
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Press reset!");
  while (digitalRead(Resetknop) == HIGH) {delay(1);}
  
  // Show that reset works
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Reset works!");
  delay(dt);

  // Wait for switch to the left
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Switch left!");
  while (digitalRead(Schakelaar1) == HIGH) { delay(1);}

  // Show that we can run 1-time
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("One round!");
  delay(dt);

  // Wait for switch to the right
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Switch right!!");
  while (digitalRead(Schakelaar2) == HIGH) {delay(1);}

  // Show that cycles work
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cycle works!");
  delay(dt);
  
  // Wait for emergency button
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Emergency!");
  while (digitalRead(Noodstop) == LOW) {delay(1);}

  // Show that the emergency works
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Safety aquired!");
  delay(dt);

  // Wait for emergency reset
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Fix problem!");
  while (digitalRead(Noodstop) == HIGH) {delay(1);}
  
  delay(dt);

  // Show that the system continues
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Continuing ");
  lcd.setCursor(0, 1); lcd.print("System");

  delay(dt);

  // Show that the second test will start
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Second Test");
  lcd.setCursor(0, 1); lcd.print("Signal test");

  delay(dt);

  // Testing green statuslight
  digitalWrite(Groen, HIGH);
  delay(dt2);
  digitalWrite(Groen, LOW);

  // Testing orange statuslight
  analogWrite(Oranje, 255);
  delay(dt2);
  analogWrite(Oranje, 0);

  // Testing red statuslight
  analogWrite(Rood, 255);
  delay(dt2);
  analogWrite(Rood, 0);
  
  // Show that the third test will start
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Third Test");
  lcd.setCursor(0, 1); lcd.print("LED test");

  delay(dt);

  // Print that leds will shift to the right
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("shift to right");
  
  // Loop through each LED and toggle its state
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(leds[i], HIGH);  // Turn on the current LED
    delay(500);                   // Wait for the specified delay time
    digitalWrite(leds[i], LOW);   // Turn off the current LED
  }
  
  delay(dt);

  // Show that leds will shift to the left
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("shift to left");

  // Loop through each LED in reverse order and toggle its state
  for (int i = numLeds - 1; i >= 0; i--) {
    digitalWrite(leds[i], HIGH);  // Turn on the current LED
    delay(500);             // Wait for the specified delay time
    digitalWrite(leds[i], LOW);   // Turn off the current LED
  }

  delay(dt);

  // Show that the interface test has finished
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Interface test");
  lcd.setCursor(0, 1); lcd.print("Finished");
  
  // After 5 seconds return to the start of the void loop
  delay(5000);
}

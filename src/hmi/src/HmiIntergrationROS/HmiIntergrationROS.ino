#include <LiquidCrystal_I2C.h>  // Includes and LCD configuration of LCD adress
#include <Wire.h>               // Basics arduino wiring codes
#include <ros.h>                // ROS libary to communicate in ROS
#include <std_msgs/String.h>    // Message libary
#include <std_msgs/Bool.h>      // Message libary

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Press buttons and emergency
#define Emergency 2 
#define StartButton 4 
#define StopButton 5
#define ResetButton 6

// Cycle switch
#define Switch1 7
#define Switch2 8

// Status lamp
#define Green 56 
#define Orange 55  // Can also be yellow
#define Red 54  

// Leds
#define Led1 9
#define Led2 10
#define Led3 11

// Linecounter for LCD and integers for object counting
int lineCounter = 0;        // Put line counter to 0
int a = 0;
int b = 0;
int c = 0;
int d = 0;
int e = 0;
int f = 0;
int g = 0;
int h = 0;

// Creates string to print with integers to increase
String lines[] = { "Bak 1: " + String(a),"Bak 2: " + String(b),
"Bak 3: " + String(c), "Bak 4: " + String(d), "Bak 5: " + String(e),
"Bak 6: " + String(f), "Bak 7: " + String(g), "Bak 8: " + String(h)};

// Delay constant variables
#define dt 800
#define dt2 10

std_msgs::String msg;                   // Message object
ros::NodeHandle nh;                     // Object to handle nodes 

bool go = false; // Declare the boolean variable 'go' outside the callback function

// Callback function to handle the received boolean for "ready"
void ReadyCallback(const std_msgs::Bool& msg) 
{
  bool Ready = msg.data;
  
  go = Ready; // makes go var true when ready is true in ROS
}

// Callback function to handle the received boolean for "ActuatorActive"
void ActuatorActiveCallback(const std_msgs::Bool& msg) 
{
  bool ActuatorActive = msg.data;
  
  if (ActuatorActive) 
  {
     digitalWrite(Led3, HIGH);
  }
  else
     digitalWrite(Led3, LOW);
}

// Callback function to handle the received msg for "Placed"
void PlacedCallback(const std_msgs::String& msg) 
{
  const char* receivedPlacedString = msg.data;

  // object 1 is deliverd
  if (strcmp(receivedPlacedString, "obj1") == 0) 
  {
    a++; delay(dt);
  } 

  // object 2 is deliverd
  else if (strcmp(receivedPlacedString, "obj2") == 0) 
  {
    b++; delay(dt); 
  }

  // object 3 is deliverd
  else if (strcmp(receivedPlacedString, "obj3") == 0) 
  {
    c++; delay(dt);
  }
  
  // object 4 is deliverd
  else if (strcmp(receivedPlacedString, "obj4") == 0) 
  {
    d++; delay(dt);
  }

  // object 5 is deliverd
  else if (strcmp(receivedPlacedString, "obj5") == 0) 
  {
    e++; delay(dt);
  }

  // object 6 is deliverd
  else if (strcmp(receivedPlacedString, "obj6") == 0)
  {
    f++; delay(dt);
  }

  // object 7 is deliverd
  else if (strcmp(receivedPlacedString, "obj7") == 0) 
  {
    g++; delay(dt);
  }

  // object 8 is deliverd
  else if (strcmp(receivedPlacedString, "obj8") == 0) 
  {
    h++; delay(dt);
  }

  // Update the strings with the increased values
  lines[0] = "Bak 1: " + String(a);
  lines[1] = "Bak 2: " + String(b);
  lines[2] = "Bak 3: " + String(c);
  lines[3] = "Bak 4: " + String(d);
  lines[4] = "Bak 5: " + String(e);
  lines[5] = "Bak 6: " + String(f);
  lines[6] = "Bak 7: " + String(g);
  lines[7] = "Bak 8: " + String(h);
}

// Callback function to handle the received msg for "Error"
void ErrorCallback(const std_msgs::String& msg) 
{
  const char* receivedErrorString = msg.data;

  // Orange/ Yellow needs to be high
  analogWrite(Red, 0);
  analogWrite(Orange, 255);
  analogWrite(Green, 0);

  // If topic error gets the message "error bak 1"
  if (strcmp(receivedErrorString, "error bak 1") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 1 vol");
    delay(dt2);
  } 
  
    // If topic error gets the message "error bak 2"
  else if (strcmp(receivedErrorString, "error bak 2") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 2 vol");
    delay(dt2);
  } 
  
    // If topic error gets the message "error bak 3"
  else if (strcmp(receivedErrorString, "error bak 3") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 3 vol");
    delay(dt2);
  }
  
    // If topic error gets the message "error bak 4" 
  else if (strcmp(receivedErrorString, "error bak 4") == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 4 vol");
    delay(dt2);
  } 
  
    // If topic error gets the message "error bak 5"
  else if (strcmp(receivedErrorString, "error bak 5") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 5 vol");
    delay(dt2);
  } 

    // If topic error gets the message "error bak 6"
  else if (strcmp(receivedErrorString, "error bak 6") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 6 vol");
    delay(dt2);
  } 

    // If topic error gets the message "error bak 7"
  else if (strcmp(receivedErrorString, "error bak 7") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 7 vol");
    delay(dt2);
  } 

    // If topic error gets the message "error bak 8"
  else if (strcmp(receivedErrorString, "error bak 8") == 0) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Bak 8 vol");
    delay(dt2);
  } 
  
    // If topic error gets a diffrent message from the 8
  else 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Error");
    lcd.setCursor(0, 1); lcd.print("Unknown");
    delay(dt2);
  }
}


 // Callback function to handle the received "Faulth" string
void FaulthCallback(const std_msgs::String& msg) {
  const char* receivedFaulthString = msg.data;

  // Red lamp needs to be high
  analogWrite(Red, 255);
  analogWrite(Orange, 0);
  analogWrite(Green, 0);

  // If topic Faulth gets msg "motor"
  if (strcmp(receivedFaulthString, "motor") == 0) 
   {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Faulth");
    lcd.setCursor(0, 1); lcd.print("motor");
    delay(dt2);
   } 
   
  // If topic Faulth gets msg "robot"
  else if (strcmp(receivedFaulthString, "robot") == 0) 
   {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Faulth");
    lcd.setCursor(0, 1); lcd.print("robot");
    delay(dt2);
   } 

  // If the msg is unkown to the topic
  else 
   {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Faulth");
    lcd.setCursor(0, 1); lcd.print("Unknown");
    delay(dt2);
   }
}

// Subscribers
ros::Subscriber<std_msgs::Bool> ReadySub("/HMI/Ready", &ReadyCallback);
ros::Subscriber<std_msgs::Bool> ActuatorActiveSub("/HMI/ActuatorActive", &ActuatorActiveCallback);
ros::Subscriber<std_msgs::String> PlacedSub("/HMI/Placed", &PlacedCallback);
ros::Subscriber<std_msgs::String> ErrorSub("/HMI/Error", &ErrorCallback);
ros::Subscriber<std_msgs::String> FaulthSub("/HMI/Faulth", &FaulthCallback);

// Published messages
std_msgs::Bool StartStop_msg;
std_msgs::Bool Reset_msg;
std_msgs::Bool Estop_msg;
std_msgs::Bool SingleCycle_msg;

// Publisher topics
ros::Publisher StartStopPub("/HMI/StartStop", &StartStop_msg);
ros::Publisher ResetPub("/HMI/Reset", &Reset_msg);
ros::Publisher EstopPub("/HMI/Emergency", &Estop_msg);
ros::Publisher SingleCyclePub("/HMI/SingleCycle", &SingleCycle_msg);

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

  // Init LCD en serial monitor
  lcd.begin();
  lcd.backlight();
  lcd.noBlink();
  Serial.begin(9600);                   // Initialize the serial communication

  // Initialize node message
  nh.initNode();

  // Start initialisation pubisher data
  StartStop_msg.data = false;
  Reset_msg.data = false;
  Estop_msg.data = false;
  SingleCycle_msg.data = false;
  
  // Advertise publishers
  nh.advertise(StartStopPub);
  nh.advertise(ResetPub);
  nh.advertise(EstopPub);
  nh.advertise(SingleCyclePub);
  
  // Subscribe to topics
  nh.subscribe(ReadySub);
  nh.subscribe(ActuatorActiveSub);
  nh.subscribe(PlacedSub);
  nh.subscribe(ErrorSub);
  nh.subscribe(FaulthSub);
  
  // Show that the third test will start
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Booting...");
  lcd.setCursor(0, 1); lcd.print("Installation");

  bool readyReceived = false;  // Flag to track if "Ready" message is received

  // While ready is not recieved the green light blinks
  while (!readyReceived) {
    
    nh.spinOnce(); // Process ROS callbacks

    // Blink Green light
    digitalWrite(Green, HIGH);
    delay(dt);
    digitalWrite(Green, LOW);
    delay(dt);

    // Set var readyRecieved to true
    if (go)
    {
      readyReceived = true;
    }
  }

  // While start is not pressed the green light is on and ready to start will be shown
  while (digitalRead(StartButton) == HIGH) 
  {  
  digitalWrite(Green, HIGH);
  lcd.setCursor(0, 0); lcd.print("Ready to start");
  }
  
  lcd.clear();
  
}

void loop() 
{
  // if Emergency is pressed
  if (digitalRead(Emergency) == HIGH)
   {
    
    // Print message on LCD
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Emergency button");
    lcd.setCursor(0, 1); lcd.print("is pressed!");
    
    Estop_msg.data = true;               // Communicates that E-stop is pressed
    EstopPub.publish(&Estop_msg);        // Publishes the data
    
    delay(dt);

    // Orange and green will be high
    analogWrite(Orange, 255);
    analogWrite(Green, 255);  

    // While emergencystop is active it keeps looping until it's safe
    while (digitalRead(Emergency) == HIGH) {delay(dt2);} 

    // While emergencystop is released
    if (digitalRead(Emergency) == LOW)
    {
      // Print message on LCD
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("No more");
      lcd.setCursor(0, 1); lcd.print("Emergencies");
      
      Estop_msg.data = false;              // Communicates that E-stop is released
      EstopPub.publish(&Estop_msg);        // Publishes the data
      delay(dt2); 
    }
 
    delay(dt);

    // Makes the lights low
    analogWrite(Orange, 0);
    analogWrite(Green, 0);

    delay(1000);

    // Turn green light on implementing that start can be pressed
    analogWrite(Green, 255);
    
  } 

  // If statement for Switch1 for Single/Cycle topic to change cycle state
  if (digitalRead(Switch1) == LOW) 
  {
     digitalWrite(Led1, HIGH);
     digitalWrite(Led2, LOW);                     
     SingleCycle_msg.data = false;                // Communicates that the system is in cycle mode
     SingleCyclePub.publish(&SingleCycle_msg);    // Publishes the data
     delay(dt2);
  } 

  // If statement for Switch2 for Single/Cycle topic to change Single state
  if (digitalRead(Switch2) == LOW) 
  {
     digitalWrite(Led1, LOW);
     digitalWrite(Led2, HIGH);
     SingleCycle_msg.data = true;                 // Communicates that the system is in single mode
     SingleCyclePub.publish(&SingleCycle_msg);    // Publishes the data
     delay(dt2);

       // If Ready is false and switch is in single mode
      if (!go && digitalRead(Switch2) == LOW)
    {
     analogWrite(Green, 255);                   // Turn on the green light
     analogWrite(Red, 0);                   // Turn on the green light
     analogWrite(Orange, 0);                   // Turn on the green light
     StartStop_msg.data = false;                  // Set StartStop to false
     StartStopPub.publish(&StartStop_msg);        // Publishes the data
     delay(dt2);
    }   
  }   

  // If statement for StopButton for Start/Stop topic to stop the system
  if (digitalRead(StopButton) == HIGH) 
  {
    // Red light on
    analogWrite(Green, 0);
    analogWrite(Orange, 0);
    analogWrite(Red, 255);
    
    StartStop_msg.data = false;              // Communicates that stop is pressed since FALSE
    StartStopPub.publish(&StartStop_msg);    // Publishes the data
    
    delay(dt2);
  }

  // If statement for StopButton for Reset topic to reset the LCD
  if (digitalRead(ResetButton) == LOW) 
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Values");
    lcd.setCursor(0, 1); lcd.print("Reset");
    delay(dt2);
  
    Reset_msg.data = true;              // Communicates that the reset is pressed since TRUE
    ResetPub.publish(&Reset_msg);       // Publishes the data
    
    // Reset values to 0
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    e = 0;
    f = 0;
    g = 0;
    h = 0;

    // Update the strings with the increased values
    lines[0] = "Bak 1: " + String(a);
    lines[1] = "Bak 2: " + String(b);
    lines[2] = "Bak 3: " + String(c);
    lines[3] = "Bak 4: " + String(d);
    lines[4] = "Bak 5: " + String(e);
    lines[5] = "Bak 6: " + String(f);
    lines[6] = "Bak 7: " + String(g);
    lines[7] = "Bak 8: " + String(h);

    delay(dt2);
 } 
  
 
 if (digitalRead(StartButton) == LOW) 
 {
   // Orange light on
    digitalWrite(Green, LOW);
    analogWrite(Orange, 255);
    analogWrite(Red, 0);
    
    StartStop_msg.data = true;             // Communicates that stop is pressed since TRUE
    StartStopPub.publish(&StartStop_msg);  // Publishes the data
    
    delay(dt2);
  }

  lcd.clear();

  // Print two lines simultaneously
  lcd.setCursor(0, 0);
  lcd.print(lines[lineCounter]);

  if (lineCounter + 1 < sizeof(lines) / sizeof(lines[0])){
    lcd.setCursor(0, 1);
    lcd.print(lines[lineCounter + 1]);
  }

  // Increase linecounter by two
  lineCounter += 2;

  // Reset the lines to repeat the print proces for the 8 object lines
  if (lineCounter >= sizeof(lines) / sizeof(lines[0])){
    lineCounter = 0;
  }

  delay(dt2);

  nh.spinOnce();

  delay(dt); // Delay to get time to see the printed text on the LCD
}

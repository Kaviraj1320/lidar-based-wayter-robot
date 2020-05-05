#define USE_STM32_HW_SERIAL
#include <ros.h>

#include <std_msgs/String.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

#include <WS2812B.h>
#define NUM_LEDS 13
WS2812B strip = WS2812B(NUM_LEDS);

StaticJsonBuffer<500> sendjson;
JsonObject& doc = sendjson.createObject();
char Send_data[300];

#define touch1 PB2                                             // BOTH TOUCH SENSOR IS DIGITAL
//#define touch2 PB3                                        //TO CALCULATE MOTOR2 RPM-- 3 PULSE 1 RPM --- ARDUINO INPUT PIN

// Power Bypass
#define BypassRelay        PB5                                  // BRK for M2 in PCB ----RASPBERRY PI SHUTDOWN RELAY

int onBootParking = 0;
//----------------------------------------------------------------//
//--------------System control devices definition-----------//

#define battery     PA0                                       //TO CALCULATE BATTERY VOLTAGE--- ARDUINO INPUT
                                   //

//---------------------------------------------------------//

//Json Variables
String Pi_Data;                                              //ARDUINO TO RASPBERRY PI COMMUNICATION
String Command;
String Param;


//

byte onObstacle = 0;
byte onTouch = 0;
byte onStart = 0;
byte onStop = 0;

// robot operation variables
byte ledFadeSpeed = 1;
byte ledInc = 0;

byte isAutomatic = 1 ;
int blu_count=0;

int motorNormal   = 0;  //int motorNormal   = 50; 

//pi boot variable
byte isbooted = 0;

//battery variable
long batteryTime = 0;
int pre_batteryVal = 0;
int batteryVal=0;

//parking tag variable
int isParking=1;
int touchcount = 0;
const int pingPin_1 = PA3; // Trigger Pin of Ultrasonic Sensor
const int echoPin_1 = PA2; // Echo Pin of Ultrasonic Sensor

const int pingPin_2 = PA5; // Trigger Pin of Ultrasonic Sensor
const int echoPin_2 = PA4; // Echo Pin of Ultrasonic Sensor

const int pingPin_3 = PA7; // Trigger Pin of Ultrasonic Sensor
const int echoPin_3 = PA6; // Echo Pin of Ultrasonic Sensor
byte R=0;
byte G=0;
byte B=255;

void messageCb( const std_msgs::String& toggle_msg)
{

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& Rcvdata = jsonBuffer.parseObject(toggle_msg.data);
  if(Rcvdata["command"] == "Robot")
    {
      if( Rcvdata["param"] == 1)
      {
       R=220;
       G=0;
       B=0;
        
      }
      else if( Rcvdata["param"] == 2)
      {
        R=0;
        G=220;
        B=0;
        
      }
      else if( Rcvdata["param"] == 3)
      {
        R=0;
        G=0;
        B=220;
        
      }
      else if( Rcvdata["param"] == 4)
      {
        R=220;
        G=220;
        B=220;
      }
      EEPROM.update(1,R);
      EEPROM.update(2,G);
      EEPROM.update(3,B);
      
    }
   if(Rcvdata["command"] == "BOOTED")                                               
    {
      isbooted=1;
      doc["reply"] = "CONNECTED";
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();
      
      
      
    }
    if (Rcvdata["command"] == "START")
    {
      doc["reply"] = "STARTED";
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();
      isAutomatic = 1;
      runLED(1);
    }
    else if (Rcvdata["command"] == "STOP")
    {
      doc["reply"] = "STOPPED";
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();
      batterylow(0);
      runLED(0);
      isParking=0;
    }
    else if (Rcvdata["command"] == "TABLE")
    {
     while (!touchSensor())                                        //AFTER FOOD DELIVERD AND WAIT FOR TOUC DETECTED            
      {
        deliveryLed(1);
      }

      doc["reply"] = "DELIVERED";
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();
      deliveryLed(0);
      runLED(1);
        
    }
    else if(Rcvdata["command"]=="PARKING")                                            //MANUALLY CONTROL ROBOT USING BLUETOOTH
    {
      isParking=1;
    }
  
    else if(Rcvdata["command"]=="MANUAL")                                            //MANUALLY CONTROL ROBOT USING BLUETOOTH
    {
      doc["reply"] = "MANUAL";
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();
      isAutomatic=0;
      runLED(0);
      bluetoothLed(1);
    }
    Command = "";
  
  
}

ros::Subscriber<std_msgs::String> sub("nanodata", &messageCb );

void setup()
{

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  strip.begin();// Sets up the SPI
  strip.show();
  pinMode(BypassRelay, OUTPUT);                                         //ARDUINO0002233946 ON SIGNAL
  pinMode(touch1, INPUT);
  //pinMode(touch2, INPUT);

  pinMode(battery, INPUT);
  
  digitalWrite(BypassRelay, LOW);

  EEPROM.write(1,R);
  EEPROM.write(2,G);
  EEPROM.write(3,B);

  batteryTime = (millis()/1000)-30;                                 //TO SEND BATTERY VALUE IN 30 SEC ONCE
}

int irSensor()
{
   long duration_1, inches_1, cm_1;
   pinMode(pingPin_1, OUTPUT);
   digitalWrite(pingPin_1, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin_1, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin_1, LOW);
   pinMode(echoPin_1, INPUT);
   duration_1 = pulseIn(echoPin_1, HIGH);
   cm_1 = duration_1*0.017;
   
   long duration_2, inches_2, cm_2;
   pinMode(pingPin_2, OUTPUT);
   digitalWrite(pingPin_2, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin_2, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin_2, LOW);
   pinMode(echoPin_2, INPUT);
   duration_2 = pulseIn(echoPin_2, HIGH);
   cm_2 = duration_2*0.017;
   
   long duration_3, inches_3, cm_3;
   pinMode(pingPin_3, OUTPUT);
   digitalWrite(pingPin_3, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin_3, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin_3, LOW);
   pinMode(echoPin_3, INPUT);
   duration_3 = pulseIn(echoPin_3, HIGH);
   cm_3 = duration_3*0.017;
  /* Serial.print(cm_1);
   Serial.print("  |  ");
   Serial.print(cm_2);
   Serial.print("  |  ");
   Serial.println(cm_3);
   delay(100);*/
  if(isParking == 1)
  {
    if ((cm_1<=30) || (cm_2<=30) )                                     //IF OBSTACLE DETECTED 
    {
      return 1;
    }
    else                                                                     // IF OBSTACLE NOT DETECTED
    {
      return 0;
    }
  }
  else
  {
    if ((cm_1<80) || (cm_2<80) || (cm_3<80) )             //IF OBSTACLE DETECTED 
    {
      return 1;
    }
    else                                                                     // IF OBSTACLE NOT DETECTED
    {
      return 0;
    }
  }
}

int touchSensor()
{
  if ((digitalRead(touch1)))// || digitalRead(touch2))                        // IF TOUCH SENSOR DETECTED
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int batteryMon()
{
  batteryVal = map(analogRead(battery), 740, 925, 0, 100);
  //int batteryVal = map(analogRead(battery), 800, 1018, 0, 100);        //  MAPPING VALUE BATTERY VOLT-23V INPUT VOLT-3.58V MAPPING VALUE-800
  //int batteryVal = map(analogRead(battery), 724, 908, 0, 100);       //BATTERY VOLT 30V INPUT VOLT-4.6V MAPPING VALUE-1018
  //Serial.println(analogRead(battery));
  //Serial.println(batteryVal);
  if (((millis()/1000) - batteryTime) >= 30)                           //TO SEND BATTERY VALUE IN 30 SEC ONCE
  {
    batteryVal = constrain(batteryVal,0,100);                          //BATTERY VALUE BETWEEN 0 TO 100                        
    {
      if(isParking == 1 )
      {
        doc["reply"] = "BATTERY";
        doc["param"]   = batteryVal;
        doc.printTo(Send_data, sizeof(Send_data));
        str_msg.data =Send_data ;
        chatter.publish( &str_msg );
        nh.spinOnce();
        
      }
      
    }
    batteryTime=millis()/1000;
    if (batteryVal <= 10)                                                //IF ROBOT IS OFF TO SEND THE SHUTDOWN SIGNAL TO RASPBERRY PI  
      {
        doc["reply"] = "SHUTDOWN";
        doc.printTo(Send_data, sizeof(Send_data));
        str_msg.data =Send_data ;
        chatter.publish( &str_msg );
        nh.spinOnce();
        delay(20000);                                                      //UNTILL ARDUINO IS ON
        digitalWrite(BypassRelay, HIGH);
      }
      else
      {
        digitalWrite(BypassRelay, LOW);
      }
  }
}


//---------------------------------------------------RING LIGHT CONTROL--------------------------//
void deliveryLed(int downUP)
{
  if (downUP == 1)
  {
    ledInc = ledInc + ledFadeSpeed;
  }
  else
  {
    ledInc = ledInc - ledFadeSpeed;
  }

  colorWipe(strip.Color(0, 0, ledInc), 0); 
  delay(10);
}


void obstacleLed(int downUP)
{
  if (downUP == 1)
  {
    ledInc = ledInc + ledFadeSpeed;
  }
  else
  {
    ledInc = ledInc - ledFadeSpeed;
  }  
  colorWipe(strip.Color(ledInc, 0, 0), 0); 
  delay(10);
}



void runLED(int offON)
{
  if (offON == 1)
  {
    for (ledInc = 0; ledInc < 200; ledInc = ledInc + ledFadeSpeed)
    { 
      colorWipe(strip.Color(ledInc, ledInc, ledInc), 0);
      delay(10);
    }
  }
  else
  {
    for (ledInc = 200; ledInc > 0; ledInc = ledInc - ledFadeSpeed)
    {     
       colorWipe(strip.Color(ledInc, ledInc, ledInc),0);      
      delay(10);
      
    }
  }
}

void batterylow(int offON)
{
  if (offON == 1)
  {
    for (ledInc = 0; ledInc < 204; ledInc = ledInc + ledFadeSpeed)
    { 
      colorWipe(strip.Color(ledInc, 0, ledInc), 0);
      delay(10);
    }
  }
  else
  {
    for (ledInc = 204; ledInc > 0; ledInc = ledInc - ledFadeSpeed)
    {     
       colorWipe(strip.Color(ledInc, 0, ledInc),0);      
      delay(10);
      
    }
  }
}


void piBoot(int boot,byte R,byte G,byte B)
{
  if(boot == 1)
  {
    byte time = millis() >> 2;
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
      byte x = time - 8*i;
      strip.setPixelColor(i, strip.Color(R, G, B - x));
      strip.show();
      
    }
     
    delay(10);
  }
  else if(boot == 0)
  {
    while(!(R <=0) && (G<=0) && (B <= 0))
    //for (ledInc = 170; ledInc > 0; ledInc = ledInc - ledFadeSpeed)
    {
      colorWipe(strip.Color(R, G, B),0); 
      R=R-1;
      G=G-1;
      B=B-1;
      delay(10);
    }
  }
}

void bluetoothLed(int blu)
{
  if(blu == 1)
  { 
    colorWipe(strip.Color(225, 112, 0),0); 
    
    delay(10);
  }
  else if(blu==0)
  {
    for (ledInc = 150; ledInc > 0; ledInc = ledInc - ledFadeSpeed)
    {
      colorWipe(strip.Color(ledInc, ledInc, 0),0); 
      delay(10);
      
    }
  }
}


void colorWipe(uint32_t c, uint8_t wait) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
      strip.setPixelColor(i, c);
      strip.show();
  }
}

void manOverRide()
{
  if ((digitalRead(touch1)))// && digitalRead(touch2))                        // IF TOUCH SENSOR DETECTED
  {
    touchcount=touchcount+1;
    Serial.print("touchcount=");
    Serial.println(touchcount);
    if(touchcount>300)
    {
      if (isAutomatic==1)
      {
        Command="Manual";
        doc["reply"] = "MANUAL";
        doc.printTo(Send_data, sizeof(Send_data));
        str_msg.data =Send_data ;
        chatter.publish( &str_msg );
        nh.spinOnce();
        isAutomatic=0;
        runLED(0);
        bluetoothLed(1);
        

      }
      else 
      {
        doc["reply"] = "NOMANUAL";
        doc.printTo(Send_data, sizeof(Send_data));
        str_msg.data =Send_data ;
        chatter.publish( &str_msg );
        nh.spinOnce();        
        bluetoothLed(0);
        runLED(1);
        isAutomatic = 1;
      }
      touchcount=0; 
    }
  }
  else
  {
    touchcount=0;
  }
}

void loop()
{
  nh.spinOnce();
  
  if(isbooted == 1)                                                 // IF RASPBERRY IS BOOTED                                          
  {
    piBoot(0);
    isbooted = 2;
  }
  else if(isbooted == 0)                                            //IF RASPBERRY PI IS NOT BOOTED
  {
    R=EEPROM.read(1);
    G=EEPROM.read(2);
    B=EEPROM.read(3);
    piBoot(1,R,G,B);
  }
  
  
  if(isbooted == 2)
  {
    
    manOverRide();
    batteryMon();
    
    
    if((isParking == 1) && touchSensor() == 1)
    {
      
      doc["reply"] = "MOTORSTOP"; 
      doc.printTo(Send_data, sizeof(Send_data));
      str_msg.data =Send_data ;
      chatter.publish( &str_msg );
      nh.spinOnce();//OBSTACLE DETECTED SEND TO RASPBERRY PI USING JSON
      
    }
    if((batteryVal<=40) && (isParking == 1))
    {
      runLED(0);
      batterylow(1);
    }
    
    if ( (irSensor() == 1) && (isAutomatic == 1) )                                      // IF OBSTACLE IS DETECTED
        {
          doc["reply"] = "OBSTACLE";                                   //OBSTACLE DETECTED SEND TO RASPBERRY PI USING JSON
          doc["param"]   = "true";
          doc.printTo(Send_data, sizeof(Send_data));
          str_msg.data =Send_data ;
          chatter.publish( &str_msg );
          nh.spinOnce();
          
          runLED(0);
          while (irSensor() == 1)                                                               
          {
            batteryMon();
            obstacleLed(1);
            manOverRide();
            if(Command == "MANUAL")
            {
              break;
            }
             
          }
          obstacleLed(0);
          runLED(1);
         doc["reply"] = "OBSTACLE";
          doc["param"]   = "false";
          doc.printTo(Send_data, sizeof(Send_data));
          str_msg.data =Send_data ;
          chatter.publish( &str_msg );
          nh.spinOnce();
        }
  }
}



// ---------------------------------------------------------------------------
// Écart par égard, mesure de la distance de dépassement.
// ---------------------------------------------------------------------------

#include <NewPing.h>            // Ultrasonic range finder
#include <Adafruit_NeoPixel.h>  // for LED 7 segment

#include <QuickStats.h>
float readings[5];
QuickStats stats; //initialize an instance of this class


// Settings for ultrasonic range finder
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define DISTANCE_KO     100 // Up to this distance (cm) it's too close --> RED
#define DISTANCE_ERR    150 // Up to this distance (cm) it's acceptable --> ORANGE
// DISTANCE_OK = MAX_DISTANCE, over this distance no overtaking is detected, MAX_DISTANCE define the limit.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

const int BatteryVoltagePin = A0;  // Analog input pin that the battery/2 voltage is attached
const int LuminosityPin = A1;     // Luminosity
int Luminosity = 0; 
int BatteryVoltage = 0;        // value read from the pot
int BatteryCharge = 0;

int Distance_measured = 0;
int Distance_offset = 0;  // Distance (cm) to the border of the bike handle

// NeoPixel 7 segment display
#define NUMPIXELS 45 // Number of leds in the full strip
#define NUMPIXSEGMENT 3
#define LED_DATAPIN 8 // Pin
#define LED_BRIGHTNESS 50
 // NeoPixels 7 segment display contructor
Adafruit_NeoPixel LedStrip = Adafruit_NeoPixel(NUMPIXELS, LED_DATAPIN, NEO_GRB + NEO_KHZ800);

void setup() 
{
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  delay(50); //pause a moment to let capacitors on board settle
 // Serial.begin(9600); // Open serial monitor at xxxx baud to see ping results.

 // NeoPixels 7 segment display
  LedStrip.begin(); // Initialize pins for output

  analogReference(DEFAULT); // Luminosity is Ratiometric, take the PS as ref
  Luminosity = analogRead(LuminosityPin);
  // map it to the range of the LEDs:
  Luminosity = map(Luminosity, 0, 1023, 5, 255);
  LedStrip.setBrightness(Luminosity);

  // Test led on startup
  int color[3]={0, 0, 255};
  SetAllLED(color);
  LedStrip.show();

  delay(1000);
  
  Distance_offset = sonar.ping_cm();
  if (Distance_offset>50) Distance_offset = 50;
}

void loop() {
  int color[3]={0, 255, 0};
  int DistUnit, DistDecimal;
  
  // Sample n distances to filter out bad measurements
  for (int i=0; i<5; i++)
  {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    readings[i]=sonar.ping_cm()-Distance_offset;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on when a measeure is processed
    delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  }

  Distance_measured = stats.median(readings, 5);
  DistUnit = Distance_measured/100;
  DistDecimal = (Distance_measured/10)%10;
  
  // Distance more that just the bike handle
  if (Distance_measured > 10)
  {
    // Too close?
    if (Distance_measured < (DISTANCE_KO))
    {
      //display.println("close");
      color[0]=255; // Red
      color[1]=0;
      color[2]=0;
    }
    else if (Distance_measured < (DISTANCE_ERR))
    {
      //display.println("accept.");
      color[0]=255;  // Orange
      color[1]=127;
      color[2]=0;
    }
    else
    {
      //display.println("ok");
      color[0]=0; //green
      color[1]=255;
      color[2]=0;
    }
    Luminosity = analogRead(LuminosityPin);
    Luminosity = map(Luminosity, 0, 1023, 5, 255);
    if(Luminosity<127)
    {
      Luminosity = Luminosity<<1;
    }
    else
    {
      Luminosity = 255;
    }
    Serial.print("Luminosity = ");
    Serial.print(Luminosity);
    LedStrip.setBrightness(Luminosity);
    
    digitWrite(0, DistUnit, color);
    digitWrite(1, DistDecimal, color);
    LedStrip.show();
    delay(1000);
  }
  else
  {
    color[0]=0; // OFF
    color[1]=0;
    color[2]=0;
    SetAllLED(color);
    LedStrip.show();
  }
  // Check battery
  analogReference(INTERNAL); // 1.1V
  BatteryVoltage = analogRead(BatteryVoltagePin); // Dummy measurement to bypass bug
  delay(20);
  BatteryVoltage = analogRead(BatteryVoltagePin);
  if (BatteryVoltage>35) // check if battery button pressed
  {
    BatteryVoltage = map(BatteryVoltage,0,1023,0,44);  // 1/10 of volt
    
    Luminosity = analogRead(LuminosityPin);
    Luminosity = map(Luminosity, 0, 1023, 5, 255);
    LedStrip.setBrightness(Luminosity);
    color[0]=0; 
    color[1]=0;
    color[2]=0;
    digitWrite(0, 0, color);
    color[1]=255;
    digitWrite(1, BatteryVoltage - 35, color);
    LedStrip.show();
    delay(1000);
  }
  
}

void Blink()
{
  for (int i=0; i<6; i++)
  {
    LedStrip.setBrightness(10); // It's lossy, try to keep values!
    LedStrip.show();
    delay(500);
    LedStrip.setBrightness(LED_BRIGHTNESS);
    LedStrip.show();
    delay(500);
  }
}

void SetAllLED(int* color)
{
  for (int i =0; i< NUMPIXELS; i++)
  {
    LedStrip.setPixelColor(i,color[0],color[1],color[2]);
  }
}

void JingleLED(void)
{
  for (int i =0; i< NUMPIXELS; i++)
  {
    LedStrip.setPixelColor(i,i*30,i*30+120,i*30+240);
  }
}

void digitWrite(int digit, int DigitValue, int* color)
{
  /*
  // Letters are the standard segment naming, as seen from the front,
  // numbers are based upon the wiring sequence
  
            A 2     
       ----------
      |          |
      |          |
  F 1 |          | B 3
      |          |
      |     G 7  |
       ----------
      |          |
      |          |
  E 6 |          | C 4
      |          |
      |     D 5  |
       ----------    dp 8
  
  */
  // Convert the value to the segment order
  unsigned char SegmentVal = 0;
  switch (DigitValue)
  {
  case 0:
    SegmentVal = 0b10111111;
  break;
  case 1:
    SegmentVal = 0b10001100;
  break;
  case 2:
    SegmentVal = 0b11110110;
  break;
  case 3:
    SegmentVal = 0b11011110;
  break;
  case 4:
    SegmentVal = 0b11001101;
  break;
  case 5:
    SegmentVal = 0b11011011;
  break;
  case 6:
    SegmentVal = 0b11111011;
  break;
  case 7:
    SegmentVal = 0b10001110;
  break;
  case 8:
    SegmentVal = 0b11111111;
  break;
  case 9:
    SegmentVal = 0b11011111;
  break;
  }
  
  // Program the segments depending on the value and the number of pixel per segment
  for (int segment=0; segment <8; segment ++)  //segments F,A,B,C,D,E,G,dp
  {
    if(SegmentVal&(1<<segment))   // Check if the segment should be ON
    {
      for (int segmentpixel=0; segmentpixel<NUMPIXSEGMENT; segmentpixel++)
      {
        LedStrip.setPixelColor(NUMPIXSEGMENT*8*digit+segment*NUMPIXSEGMENT+segmentpixel,color[0],color[1],color[2]);
      }
    }
    else    // segment is OFF
    {
      for (int segmentpixel=0; segmentpixel<NUMPIXSEGMENT; segmentpixel++)
      {
        LedStrip.setPixelColor(NUMPIXSEGMENT*8*digit+segment*NUMPIXSEGMENT+segmentpixel,0,0,0);
      }
    }
  }
}

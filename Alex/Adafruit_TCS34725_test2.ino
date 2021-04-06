#include <Adafruit_TCS34725.h>
#include <Wire.h>
#define LEDPIN 8

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

int colourCode = 0;
int incomingByte = 0;
int16_t r, g, b, c;

int16_t red[3] = {360, 210, 181};
int16_t green[3] = {280, 249, 196};
int16_t reading1[3] = {0};
int16_t reading2[3] = {0};
int16_t reading3[3] = {0};
int16_t readingFinal[3] = {0};


void getColor() 
{
  digitalWrite(LEDPIN, HIGH);
  delay(1000);
  tcs.getRawData(&r, &g, &b, &c);
  reading1[0] = r;
  reading1[1] = g;
  reading1[2] = b;
  Serial.print("Reading 1: ");
  Serial.print(reading1[0]);
  Serial.print(" ");
  Serial.print(reading1[1]);
  Serial.print(" ");
  Serial.print(reading1[2]);
  Serial.println(" ");
  
  delay(500);
  tcs.getRawData(&r, &g, &b, &c);
  reading2[0] = r;
  reading2[1] = g;
  reading2[2] = b;
  Serial.print("Reading 2: ");
  Serial.print(reading2[0]);
  Serial.print(" ");
  Serial.print(reading2[1]);
  Serial.print(" ");
  Serial.print(reading2[2]);
  Serial.println(" ");
  delay(500);
  tcs.getRawData(&r, &g, &b, &c);
  reading3[0] = r;
  reading3[1] = g;
  reading3[2] = b;
  Serial.print("Reading 3: ");
  Serial.print(reading3[0]);
  Serial.print(" ");
  Serial.print(reading3[1]);
  Serial.print(" ");
  Serial.print(reading3[2]);
  Serial.println(" ");
  readingFinal[0] = (reading1[0] + reading2[0] + reading3[0])/3;
  readingFinal[1] = (reading1[1] + reading2[1] + reading3[1])/3;
  readingFinal[2] = (reading1[2] + reading2[2] + reading3[2])/3;
  Serial.print("Reading Final: ");
  Serial.print(readingFinal[0]);
  Serial.print(" ");
  Serial.print(readingFinal[1]);
  Serial.print(" ");
  Serial.print(readingFinal[2]);
  Serial.println(" ");
  digitalWrite(LEDPIN, LOW);

  r = readingFinal[0];
  g = readingFinal[1];
  b = readingFinal[2]; 
  colourCode = processColor(r, g, b);
}

int processColor(int r, int g, int b)
{
  double redDist = square(r-red[0]) + square(g - red[1]) + square(b - red[2]);
  Serial.print("r: ");Serial.print(r);
  Serial.print("red[0]: ");Serial.print(red[0]);
  Serial.print("diff: ");Serial.print(r-red[0]);
  Serial.print("redDist = "); Serial.println(redDist);
  double greenDist = square(r - green[0]) + square(g - green[1]) + square(b - green[2]);
  Serial.print("greenDist = "); Serial.println(greenDist);

  if (greenDist > redDist)
  {
    return 0;
  }
  else 
  {
    return 1;
  }
}

void setup(void) {
  Serial.begin(9600);

  pinMode(8, OUTPUT); //LED
  digitalWrite(8, LOW);
  
  if (tcs.begin()) { //initialise sensor, returns true if initialisation is successful
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop(void) {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }

  if (incomingByte == '1') {
    getColor();
    Serial.println(colourCode); //0 is red, 1 is green
  }
  incomingByte = 0;
}

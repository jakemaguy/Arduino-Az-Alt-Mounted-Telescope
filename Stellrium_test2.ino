#include <RTClib.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055();
RTC_DS3231 rtc;

float jd(int y, int m, int d);
float gst(float jd, float ut);

/* Global Variable Definitions */
char txAR[10];
char txDEC[11];
char input[20];
char SIGNtel;
double arHH, arMM, arSS;
double decDEG, decMM, decSS; 
float dec, ra;

/* Lattiude/Longitude - edit the number to correspond with your location */
float lat = 42.65176250;
float lon = -71.315/15;        //longitude is west so this value is negative
int TIMEZONE = -5;  // eastern time GMT: -5
int DST = 0;                // Daylight Savings Time (1 = summer, 0 = winter)

/* prototype functions */
void convertEQ();
void transmitAR();
void transmitDEC();
double returnLST(int y, int m, int d, int h, int mn, int s);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!bno.begin())                         // Attempt communication with sensor
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  rtc.begin();
  //Wire.begin();
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){         // ottiene byte in entrata
    communication();
  }
  delay(100);
}

//-----------------------------------------------------------------------------------------------------------------------

void communication(){
  
  int i=0;
  convertEQ();
  input[i++] = Serial.read();
  delay(5);
  while((input[i++] = Serial.read()) != '#'){
    delay(5);
  }
  input[i]='\0';
  
  if(input[1]==':' && input[2]=='G' && input[3]=='R' && input[4]=='#'){             // con il comando #:GR# stellarium chiede l'invio della coordinata di AR
    transmitHA();
  }


  if(input[1]==':' && input[2]=='G' && input[3]=='D' && input[4]=='#'){             // con il comando #:GD# stellarium chiede l'invio della coordinata di AR
    transmitDEC();
  }
}

//-----------------------------------------------------------------------------------------------------------------------

void convertEQ(){
  double alt, az, dec_sin, ha_cos, ha, ha_prime, sin_A;
  /* Get a new sensor event */ 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  DateTime now = rtc.now();
  
  az = radians(euler.x());
  alt = radians(euler.y());
  
  dec_sin=((sin(alt)*sin(lat))+(cos(alt)*cos(lat)*cos(az)));
  dec=asin(dec_sin);

  ha_cos=((sin(alt)-(sin(lat)*sin(dec)))/(cos(lat)*cos(dec)));
  ha_prime=acos(ha_cos);
  
  dec=degrees(dec);
  ha_prime=degrees(ha_prime);

  sin_A = sin(az);
  sin_A = degrees(sin_A);
  if (sin_A > 0){
    ha = 360 - ha_prime; 
  }
  if (sin_A < 0){
    ha = ha_prime;
  }
  // calculate lst here
  ha = ha / 15;
  float seconds = now.second();
  float minutes = now.minute();
  float hours = now.hour();
  float local_time = (seconds / 60);
  local_time = ((local_time + minutes) / 60);
  local_time = (local_time + hours);          // hours are returned in 24 hour format via DS3231
  local_time = (local_time - DST);            // zone time
  float UT = (local_time - TIMEZONE);
  float JD = jd(now.year(), now.month(), now.day());
  float GST = gst(JD, UT);
  float LST = GST + lon;
  ra = LST - ha;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------   

void transmitHA(){                                                         // trasmissione dati nella forma   HH:MM:SS#
     //convertEQ();
     ra = modf(ra, &arHH);                                                 // ricava le ore della coordinata AR del telescopio
     ra = ra * 60;
     ra = modf(ra, &arMM);
     arSS = ra * 60;                                                       // ricava i secondi della coordinata AR del telescopio
     sprintf(txAR, "%02d:%02d:%02d#", int(arHH), int(arMM), int(arSS));
     Serial.print(txAR);
    }
    
//--------------------------------------------------------------------------------------------------------------------------------------------------------   

void transmitDEC(){                                                             // trasmissione dati nella forma  sDDßMM:SS#  (dove s è il segno + o -)
     //convertEQ();
     dec = modf(dec, &decDEG);
     (decDEG < 0) ? SIGNtel = 45: SIGNtel = 43;                                 // controllo del segno della coordinata DEC del telescopio
     dec = dec * 60;
     dec = modf(dec, &decMM);
     decSS = dec * 60;
     sprintf(txDEC, "%c%02d%c%02d:%02d#", SIGNtel, int(decDEG), 223, int(decMM), int(decSS));
     Serial.print(txDEC);
  
    }
    
//-------------------------------------------------------------------------------------------------------------------------------------

float jd(int y, int m, int d)
{
  if (m < 3) { m += 12; y -= 1; }
  double A, B, C, E, F;
  A = (long)y / 100;
  B = (long)A / 4;
  C = (long)2-A+B;
  E = (long)(365.25*(y+4716));
  F = (long)(30.6001*(m+1));
  return (C + d + E + F - 1524.5);
}

float gst(float jd, float ut)
{
  float S, T, T0, A;
  
  S = jd - 2451545.0;
  T = (S / 36525.0);
  T0 = (6.697374558 + (2400.051336 * T) + (0.000025862 * (T * T)));
  T0 = T0 - (24 * (T0 / 24));
  ut = ut * 1.002637909;
  A = ut + T0;
  if (A < 0)
    A = A - (A * (A / 24));
  return A;
}











/*
double returnLST(int y, int m, int d, int h, int mn, int s)
{
}
*/

/*
void convertDEC(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  degDEC = (euler.y());
  degDEC = modf(degDEC, &degree);
  if (degree < 0){
    Serial.print("-");
    if (degree > -10){
      Serial.print("0");
    }
  }
  else if (degree >= 0){
    Serial.print("+");
    if (degree < 10){
      Serial.print("0");
    }
  }
  Serial.print(degree, 0);
  Serial.print ((char)223);
  degDEC = degDEC * 60;
  degDEC = modf(degDEC, &deg_DECmin);
  if (deg_DECmin < 10){
    Serial.print("0");
  }
  Serial.print(deg_DECmin, 0);
  Serial.print(":");
  deg_DECsec = degDEC * 60;
  if (deg_DECsec < 10){
    Serial.print("0");
  }
  Serial.print(deg_DECsec, 0);
}
*/


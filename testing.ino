#include <RTClib.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055();
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (500)
RTC_DS3231 rtc;

void LST_get();

double dec, ra;
double arHH, arMM, arSS;
double decDEG, decMM, decSS; 

/* Lattiude/Longitude - edit the number to correspond with your location */
float lat = 42.65176250*PI/180;
float lon = -71.315;        //longitude is west so this value is negative
int TIMEZONE = -5;               // eastern time GMT: -5
int DST = 0;                     // Daylight Savings Time (1 = summer, 0 = winter)
double LST, ALT, AZ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bno.begin();
  //delay(1000);
  rtc.begin();
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float dec_sin, ha_cos, ha, ha_prime, sin_A;

  Serial.print("Alt: ");
  Serial.print(euler.y(), DEC);
  Serial.print(" \n");
  Serial.print(" Az: ");
  Serial.print(euler.x(), DEC);
  Serial.print(" \n\n");

  //converts AZ and ALT to radians becuase Arduino math take in radians as args
#if 0
  AZ = euler.x()*PI/180;
  ALT = euler.y()*PI/180;
#endif
  AZ = 0.0;
  ALT = abs(euler.y())*PI/180;

  dec_sin = ((sin(ALT) * sin(lat)) + (cos(ALT) * cos(lat) * cos(AZ)));
  dec = asin(dec_sin);

  ha_cos = ((sin(ALT) - (sin(lat) * sin(dec))) / (cos(lat) * cos(dec)));
  ha_prime = acos(ha_cos);

  dec = dec*180.0/PI;
  decDEG = (int)dec;
  decMM = (int)((dec-decDEG)*60.0);
  decSS = (dec-decDEG-decMM/60.0)*3600.0; 
  Serial.print("DegDEG: ");
  Serial.print(decDEG);
  Serial.print("\n\n");
  Serial.print("DegMM: ");
  Serial.print(decMM);
  Serial.print("\n\n");
   Serial.print("DegMM: ");
  Serial.print(decSS);
  Serial.print("\n\n");

  
  ha_prime = ha_prime*180.0/PI;

  sin_A = sin(AZ);
  sin_A = sin_A*180.0/PI;
  if (sin_A >= 0) {
    ha = 360.0 - ha_prime;
  }
  else {
    ha = ha_prime;
  }
  ha = ha / 15.0;
  LST_get();
  ra = LST - ha;
  if (ra < 0) ra += 24.0;
  /*
    local_time += now.hour();
    Serial.print(local_time, DEC);
    Serial.print("\n\n");
  */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void LST_get()
{
  DateTime now = rtc.now();
  int S = now.second();
  int MN = now.minute();
  int H = now.hour();
  int D = now.day();
  int M = now.month();
  int Y = now.year();
#if 0
  if (M < 3) { M += 12; Y -= 1; }
  double HH = H + ((float)MN/60.00) + ((float)S/3600.00);
  float AA = (int)(365.25*(Y+4716)); 
  float BB = (int)(30.6001*(M+1));
  double CurrentJDN = AA + BB + D - 15375.5 + (HH - TIMEZONE)/24;
  float CurrentDay = CurrentJDN - 2451543.5;

  double MJD = CurrentJDN - 2400000.5;
  int MDJ0 = (int)MJD;
  float ut = (MJD - MDJ0)*24.0;
  double t = (MDJ0-51544.5)/36525.0;
  double GMST = 6.697374558 + 1.0027379093*ut + (8640184.812866 + (0.093104 - 0.0000062*t)*t)*t/3600.0;
#endif
  double GMST = 18.697374558 + 24.06570982441908*D;
  int GMSTint = (int)GMST;
  GMSTint/=24;
  GMST = GMST - (double)GMSTint* 24;

  LST = GMST + lon/15;

  int LSTint = (int)LST;
  LSTint/=24;
  LST = LST - (double)LSTint *24; 
}


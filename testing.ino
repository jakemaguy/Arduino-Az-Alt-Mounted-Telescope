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

float jd(int y, int m, int d);
float gst(float jd, float ut);

float dec, ra;

/* Lattiude/Longitude - edit the number to correspond with your location */
float lat = radians(42.65176250);
float lon = (-71.315/15);        //longitude is west so this value is negative
int TIMEZONE = -5;  // eastern time GMT: -5
int DST = 0;                // Daylight Savings Time (1 = summer, 0 = winter)

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
  DateTime now = rtc.now();
  float alt, az, dec_sin, ha_cos, ha, ha_prime, sin_A, localTime;

  Serial.print("Alt: ");
  Serial.print(euler.y(), DEC);
  Serial.print(" \n");
  Serial.print(" Az: ");
  Serial.print(euler.x(), DEC);
  Serial.print(" \n\n");

  az = radians(euler.x());
  alt = radians(euler.y());

  dec_sin = ((sin(alt) * sin(lat)) + (cos(alt) * cos(lat) * cos(az)));
  dec = asin(dec_sin);

  ha_cos = ((sin(alt) - (sin(lat) * sin(dec))) / (cos(lat) * cos(dec)));
  ha_prime = acos(ha_cos);

  dec = degrees(dec);
  ha_prime = degrees(ha_prime);

  sin_A = sin(az);
  sin_A = degrees(sin_A);
  if (sin_A > 0) {
    ha = 360 - ha_prime;
  }
  if (sin_A < 0) {
    ha = ha_prime;
  }
  ha = ha / 15;
  //ra = ha;
  
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
  if (LST < 0)
    LST=LST-(24*(LST/24));
  ra = LST - ha;
  if (ra < 0)
    ra = ra + 24;
  Serial.print("Universal Time: ");
  Serial.print(UT, DEC);
  Serial.print("\n\n");

  Serial.print("Julian Time: ");
  Serial.print(JD);
  Serial.print("\n\n");

  Serial.print("GST: ");
  Serial.print(GST, DEC);
  Serial.print("\n\n");

  Serial.print("declination: ");
  Serial.print(dec, DEC);
  Serial.print("\n\n");

  Serial.print("Right Asencion: ");
  Serial.print(ra, DEC);
  Serial.print("\n\n");
  /*
    local_time += now.hour();
    Serial.print(local_time, DEC);
    Serial.print("\n\n");
  */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

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


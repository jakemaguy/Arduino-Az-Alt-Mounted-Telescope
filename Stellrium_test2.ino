#include <RTClib.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055();
RTC_DS3231 rtc;

// polaris Equitorial Coordinates
float poleAR = 2.9203;
float poleHA = 21.3362;

/* Global Variable Definitions */
char txAR[10];
char txDEC[11];
char input[20];
char SIGNtel;
double arHH, arMM, arSS;
double decDEG, decMM, decSS; 
double dec, ra;

/* Lattiude/Longitude - edit the number to correspond with your location */
float lat = 42.507416*PI/180;
float lon = -71.81309;        //longitude is west so this value is negative
int TIMEZONE = -5;  // eastern time GMT: -5
int DST = 0;                // Daylight Savings Time (1 = summer, 0 = winter)
double LST, ALT, AZ;

/* prototype functions */
void convertEQ();
void transmitAR();
void transmitDEC();
//void LST_get();

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
  LST = poleAR + poleHA;
  while (LST >= 24) LST -= 24;
}

void loop() {
  // put your main code here, to run repeatedly:
  convertEQ();
  if (Serial.available()>0){         // ottiene byte in entrata
    communication();
  }
  delay(100);
}

//-----------------------------------------------------------------------------------------------------------------------

void communication(){
  
  int i=0;
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
  /* Local Variable Definitons */
  float dec_sin, ha_cos, ha, ha_prime, sin_A;
  /* Get a new sensor event */ 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  DateTime now = rtc.now();

  //converts AZ and ALT to radians becuase Arduino math take in radians as args
  AZ = euler.x()*PI/180.0;
  ALT = euler.y()*PI/180.0;
  //ALT = -47.0;

  dec_sin = ((sin(ALT) * sin(lat)) + (cos(ALT) * cos(lat) * cos(AZ)));
  dec = asin(dec_sin);

  ha_cos = ((sin(ALT) - (sin(lat) * sin(dec))) / (cos(lat) * cos(dec)));
  ha_prime = acos(ha_cos);

  dec = dec*180.0/PI;
  //dec = abs(dec); 
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
  
  //LST_get();
  ra = LST - ha;
  while (ra >= 24) ra -= 24;
  while (ra < 0) ra += 24.0;
  
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
#if 0
  char sDEC_tel;
  sprintf(buffer, "%lf", dec);
  sscanf(buffer, "%lf.%lf", decDEG, dec);// controllo del segno della coordinata DEC del telescopio
  (decDEG < 0) ? SIGNtel = 45: SIGNtel = 43;
  if (dec < 0) dec *= -1;
  dec = dec * 60;
  memset(buffer, 0, sizeof(buffer));
  sprintf(buffer, "%lf", dec);
  sscanf(buffer, "%lf.%lf", decMM, dec);
  decSS = dec * 60;
  memset(buffer, 0, sizeof(buffer));
#endif
  char sDEC_tel;
  decDEG = (int)dec;
  decMM = (int)((dec-decDEG)*60.0);
  decSS = (dec-decDEG-decMM/60.0)*3600.0;
  (decDEG < 0) ? sDEC_tel = 45 : sDEC_tel = 43;
  sprintf(txDEC, "%c%02d%c%02d:%02d#", sDEC_tel, (int)abs(decDEG), 223, (int)decMM, (int)decSS);
  Serial.print(txDEC);
}
    
//-------------------------------------------------------------------------------------------------------------------------------------
#if 0
void LST_get()
{
  DateTime now = rtc.now();
  int S = now.second();
  int MN = now.minute();
  int H = now.hour();
  int D = now.day();
  int M = now.month();
  int Y = now.year();

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

  double GMST = 18.697374558 + 24.06570982441908*D;
  int GMSTint = (int)GMST;
  GMSTint/=24;
  GMST = GMST - (double)GMSTint* 24;
  
  LST = GMST + lon/15;
  
  int LSTint = (int)LST;
  LSTint/=24;
  LST = LST - (double)LSTint *24; 
}
#endif


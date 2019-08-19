#include <Adafruit_FONA.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

//digital pins
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#define PUMP_RELAY 5
#define NO_RELAY 6
#define NC_RELAY 7

//analog pins
#define MAIN_P 0
#define HOUSE_P 1

#define PHONE_NO "6504651751"
#define SAMP_SIZE 300 // milliseconds
#define LEAK_SIZE 0.1 // liters / minute

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

//uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

// FONA globals
uint8_t type;
//char replybuffer[255];

// fxn globals
AlarmID_t ID_CORR;
uint8_t CORR;
uint8_t DATLEN;


// ##################### SETUP #####################
void setup() {

  // initializaion from FONAtest
  while (!Serial); Serial.begin(115200); Serial.println(F("FONA Water Pressure Testing")); 
  Serial.println(F("Initializing....(May take 3 seconds)")); 
  fonaSerial->begin(4800); if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  } type = fona.type(); Serial.println(F("FONA is OK")); Serial.print(F("Found ")); switch (type) {
    case FONA800L: Serial.println(F("FONA 800L"));
      break;
    case FONA800H: Serial.println(F("FONA 800H"));
      break;
    case FONA808_V1: Serial.println(F("FONA 808 (v1)"));
      break;
    case FONA808_V2: Serial.println(F("FONA 808 (v2)"));
      break;
    case FONA3G_A: Serial.println(F("FONA 3G (American)"));
      break;
    case FONA3G_E: Serial.println(F("FONA 3G (European)"));
      break;
    default: Serial.println(F("???"));
      break;
  } char imei[16] = {0}; uint8_t imeiLen = fona.getIMEI(imei); if (imeiLen > 0) {
    Serial.print(F("Module IMEI: "));
    Serial.println(imei);
  }

  // hold for network, sync time
  // important to ensure proper test timing
  while(fona.getNetworkStatus()!=1){Serial.println(F("Waiting for network..."));delay(3000);}
  syncTime();

  // set pinmodes and default (initial) states, with all relays off
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(NO_RELAY, OUTPUT);
  pinMode(NC_RELAY, OUTPUT);
  digitalWrite(PUMP_RELAY,LOW);
  digitalWrite(NO_RELAY,LOW);
  digitalWrite(NC_RELAY,LOW);

  // repeat the leaktest at 3am every day
  Alarm.alarmRepeat(3,0,0,leakTestCall);
  CORR = 0; // set global main correction flag off

  // call the function with a callback to the main correction at the nearest half hour
  // so as to prevent overlap of main correction testing and leak testing. The callback function
  // then sets an hourly repeating call of the mainCorr function, now synced with every :30.
  Alarm.alarmOnce((minute()<29)?hour():(hour()+1),30,0,mainCorrCall);

  //fona.sendSMS(PHONE_NO,"FONA Online");
}


// ##################### LOOP #####################
void loop() {
  /*
    // code from FONAtest for menu selection / FONA interaction
    // removed for brevity from printout as commented out for production regardless.
    // calls some of commented FONATest functions below, but most code inline,
    // taking up two+ full pages even when minified
  */
  
  // necessary for TimeAlarms to work
  Alarm.delay(1000);
}

// ##################### FUNCTIONS #####################
void syncTime(void) {
  char buff[23];
  String buff_str; 
  // apparently use of the "String" type is bad form as it isn't a "string", but this code functions well, can always revise to be cleaner
  int hh, mi, ss, yy, mo, dd;

  //Serial.println("syncTime");
  fona.getTime(buff, 23);
  buff_str = String(buff);

  Serial.print(F("FONA Time = ")); Serial.println(buff_str.substring(10, 18));

  // parse and convert from string to integers
  yy = buff_str.substring(1, 3).toInt();
  mo = buff_str.substring(4, 6).toInt();
  dd = buff_str.substring(7, 9).toInt();
  hh = buff_str.substring(10, 12).toInt();
  mi = buff_str.substring(13, 15).toInt();
  ss = buff_str.substring(16, 18).toInt();

  // set time_t for FONA / Arduino based on network time
  setTime(hh, mi, ss, dd, mo, yy); // not at present setting millis()
}

void mainCorrCall() {
  //Serial.println(F("mainCorrCall"));

  // call once immediately (first half-hour mark)
  mainCorr(); 
  // set hourly repeating call, with ID to remotely disable while leak testing
  ID_CORR = Alarm.timerRepeat(3600, mainCorr); 
  // safety measure for when mainCorrCall is re-called 
  Alarm.enable(ID_CORR); 
}

void mainCorr(void) {
  // determine main pressure
  float avg = 0;
  avg = avgP(MAIN_P, 10000); // average over 10s

  if (avg < 45 && avg > 25) {
    // in correctable range, notify owner and run correction
    char txt[32],
         buff[10];
    dtostrf(avg, 4, 2, buff);
    
    sprintf(txt,"Main %s psi, pumping.",buff);
    fona.sendSMS(PHONE_NO, txt);
    
    runCorr();
  } else {
    digitalWrite(NO_RELAY, LOW);
    digitalWrite(NC_RELAY, LOW);
    digitalWrite(PUMP_RELAY, LOW);
    
    if (avg <= 25) {
      // out of correctable range, notify owner
      Serial.println(F("Low pressure, out of pumping range"));

      char txt[35],
           buff[10];
      dtostrf(avg>0?avg:0, 4, 2, buff);
    
      sprintf(txt,"Main %s psi, cannot pump.",buff);
      fona.sendSMS(PHONE_NO, txt);
      
    } else {
      // in nominal range, no action required
      Serial.println(F("Pressure above 45psi"));
    }
  }

  CORR = 0; // global flag off
}

void runCorr() {
  Serial.println(F("Low pressure, pumping"));
  CORR = 1; // global flag on

  // close NO valve, open NC valve, enable pump
  digitalWrite(NO_RELAY, HIGH);
  digitalWrite(NC_RELAY, HIGH);
  digitalWrite(PUMP_RELAY, HIGH);

  // find current pressures
  float avgm = avgP(MAIN_P, 10000),
        avgh = avgP(HOUSE_P, 10000);

  // turn off mainCorr alarm temporarily
  // while loop will continue to correct pressure while 
  Alarm.disable(ID_CORR);

  while (avgm < 45 && avgm > 25) {
    Alarm.delay(3600000); // 60*60*1000 for hour delay, keep in time with half hour mark
    avgm = avgP(MAIN_P, 10000);
    avgh = avgP(HOUSE_P, 10000);
    Serial.println(F("Still correcting pressure..."));
    Serial.print(F("House pressure ")); Serial.print(avgh); Serial.println(F(" psi"));
    Serial.print(F("Main pressure ")); Serial.print(avgm); Serial.println(F(" psi"));
  }

  Serial.print(F("Pressure correction turning off, main pressure ")); Serial.print(avgm); Serial.println(F(" psi"));
  digitalWrite(NO_RELAY, LOW);
  digitalWrite(NC_RELAY, LOW);
  digitalWrite(PUMP_RELAY, LOW);
  
  CORR = 0; // global flag off
  Alarm.alarmOnce((minute() < 29) ? hour() : (hour() + 1), 30, 0, mainCorrCall); // re-start at :30
}

float readP(uint8_t pin) {
  int a = 0;
  float v, p;

  a = analogRead(pin);
  v = a * 0.00488;
  // constants determined by linear regression, data below 10psi not trusted but still included / calculated
  p = (v < 1) ? ((v * 19.0476) - 9.6191) : ((v * 44.436) - 34.579); //(v*34.809)-offset;

  return p;
}

float avgP(uint8_t pin, int t) {
// average function implemented so as to account for pressure variations over time
  
  float runsum = 0;
  long int start = millis(),
           i = 0;

  // get avg pressure over time period to account for variations
  while ((millis() - start) < t) {
    i++;
    runsum += readP(pin);
    delay(100);
  }

  return (float) runsum / i;
}

void leakTestCall(){
  if (CORR == 1){
    // disable pressure correction if at present running
    digitalWrite(NO_RELAY, LOW);
    digitalWrite(NC_RELAY, LOW);
    digitalWrite(PUMP_RELAY, LOW);

    Serial.println(F("Pressure adjustment paused for leakTest"));
    // wait for pressure to equilibrate / stop oscillating mostly
    delay(10000);
  }
  
  float *p,
        tau,
        leak;

  // get pressure data over monitoring period
  p = leakTest();

  // find time constant
  tau = curveFit(p);
  Serial.print(F("Tau: "));Serial.println(tau,8);
  
  // if not requiring correction (nominal pressure) find actual pressure and current leak rate, 
  // else, current main pressure not representative, so calculate worst case leak at pressure of 55 psi
  leak = calcLeak(tau,(!CORR)?avgP(HOUSE_P,10000):55); 
  Serial.print(F("Leak: "));Serial.println(leak,10);

  // if calculated leak is greater than specified allowable leak size, notify owner
  if (leak>LEAK_SIZE){
    char txt[32],
         buff[10];
    dtostrf(leak*60, 4, 2, buff);
    
    sprintf(txt,"Leak of %s liters/hr.",buff);
    fona.sendSMS(PHONE_NO, txt);
  }

  if (CORR == 1){
    // resume pressure correction if running before leak testing
    digitalWrite(NO_RELAY, HIGH);
    digitalWrite(NC_RELAY, HIGH);
    digitalWrite(PUMP_RELAY, HIGH);

    Serial.println(F("Leak test finished, pressure adjustment rebooted"));
  }  
}


float * leakTest(void) {
  digitalWrite(PUMP_RELAY, LOW); // ensure pump is off and not correcting for main pressure
  digitalWrite(NC_RELAY, LOW); // should already be low
  digitalWrite(NO_RELAY, HIGH); // closes the NO valve

  // array only needs to be number of times sample size fits into 60s long
  static float p[60000 / SAMP_SIZE];
  long int i = 0,
           lstart = millis();

  // run while under 60s, wait sample size duration between iterations, place pressures in array
  while ((millis() - lstart) <= 60000) {
    p[i] = readP(HOUSE_P);
    Serial.print(i*SAMP_SIZE);
    Serial.print(",");
    Serial.println(p[i]);
    i++;
    
    delay(SAMP_SIZE);
  }

  // global var for length of array
  // bad coding practice, but easier than working through proper passing of pointers for project timeline
  DATLEN = i; 
  
  //test over, reopen
  digitalWrite(NO_RELAY, LOW);

  return p; // returns pointer to first element of array, to iterate through
}

double curveFit(float p[]) {
  // of form P = Po * e^(B * t)
  // b approx B by least squares
  // Tau = -1/b
  // eqn for least squares sol'n from http://mathworld.wolfram.com/LeastSquaresFittingExponential.html

  // running sums
  float Ey = 0,
        Eylny = 0,
        Exy = 0,
        Exylny = 0,
        Exxy = 0;

  float x, xy, lny;
  // sum of "y" vals (p[])
  long int i; // else overflows and goes negative
  for (i = 0; i < DATLEN; i++) {
    // used multiple times each iteration, more efficent to calc once per
    x = (i * SAMP_SIZE) / 1000.;

    xy = x * p[i];
    lny = p[i] > 0 ? log(p[i]) : log(.01);

    // running sums, only calc if pressure above 10psi as don't trust data otherwise at present
    if (p[i]>10){
      Ey += p[i];
      Eylny += p[i] * lny;
      Exy += xy;
      Exylny += xy * lny;
      Exxy += xy * x;
    }

    // for easy visualization / confirmation via serial monitor during testing
    Serial.print(x);Serial.print(F(":"));Serial.print(Ey);Serial.print(F(":"));
    Serial.print(Eylny);Serial.print(F(":"));Serial.print(Exy);
    Serial.print(F(":"));Serial.print(Exylny);Serial.print(F(":"));Serial.println(Exxy);
  }

/*
  //**********ERROR CHECKING**********
  // compare calculated coeffiticent (pressure at t=0) to actual main pressure to check curve fitting eqns
  // po (initial main pressure) should be passed to function or measured by avgP

  float a = exp(((Exxy * Eylny) - (Exy * Exylny)) / ((Ey * Exxy) - (Exy * Exy))); // coeff of e
  int err = 5; // percentage + or -
  //Serial.println((float)((100-err)/100.));
  if (((po * ((100 - err) / 100.)) < a) && ((po * ((100 + err) / 100.)) > a)) {
    Serial.print("Measured initial pressure within ");
    Serial.print(err); Serial.println("% of curve-fit");
  } else {
    Serial.print("WARNING: Measured initial pressure outside ");
    Serial.print(err); Serial.println("% of curve-fit");
  }

  Serial.print("A: "); Serial.print(a); Serial.println(" psi");
  Serial.print("Po: "); Serial.print(po); Serial.println(" psi");

  //********END ERROR CHECKING********
*/
  //
  Serial.println(((Ey * Exylny) - (Exy * Eylny)) / ((Ey * Exxy) - (Exy * Exy)),8); //b = ((Ey*Eylny)-(Exy*Exylny))/((Ey*Exxy)-(Exy*Exy));
  return ((Exy * Exy) - (Ey * Exxy)) / ((Ey * Exylny) - (Exy * Eylny)); // Tau: equal to -1/b
}

float calcLeak(float tau,float po) {
  // leak rate ("current") is equal to ("Capacitance" * "Voltage")/Tau
  // Capacitance being the change in volume of the house pipes under pressure
  // Voltage being the pressure in Pascals (kg/(m*s^2))
  
  float i,
        v,
        c = 1.5837; // constant determined from leakdown testing exponential regression and known leak size
  int c_pow = -10; // capacitance = c * 10^c_pow

  v = po * 6894.76; // psi to Pascals, Pascals the unit of modeling "voltage"
  
  // i is in m^3/s here, to avoid some floating pt error converting to Liters / min
  //i = (v * c * pow(10, c_pow)) / tau; // in m^3/s
  i = (v * c * pow(10, c_pow+3) * 60) / tau;
  
  return i; // in liters per minute
}

/*
// ##################### TESTING FUNCTIONS #####################
int exLow(void) {
  Serial.println("Low pressure, pumping");
  digitalWrite(NO_RELAY, HIGH);
  digitalWrite(NC_RELAY, HIGH);
  digitalWrite(PUMP_RELAY, HIGH);

  return 1;
}

void txtData(float d) {
  char buff[10];
  dtostrf(d, 4, 6, buff);  //4 is mininum width, 6 is precision
  fona.sendSMS(PHONE_NO, buff);
}




// ##################### TIMEALARMS FUNCTIONS #####################
void digitalClockDisplay() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println();
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}



// ##################### FONATEST FUNCTIONS #####################
void printMenu(void) {
  Serial.println(F("-------------------------------------"));
  Serial.println(F("[?] Print this menu"));
  Serial.println(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
  Serial.println(F("[b] read the Battery V and % charged"));
  Serial.println(F("[C] read the SIM CCID"));
  Serial.println(F("[U] Unlock SIM with PIN code"));
  Serial.println(F("[i] read RSSI"));
  Serial.println(F("[n] get Network status"));
  Serial.println(F("[v] set audio Volume"));
  Serial.println(F("[V] get Volume"));
  Serial.println(F("[H] set Headphone audio (FONA800 & 808)"));
  Serial.println(F("[e] set External audio (FONA800 & 808)"));
  Serial.println(F("[T] play audio Tone"));
  Serial.println(F("[P] PWM/Buzzer out (FONA800 & 808)"));
  Serial.println(F("[f] tune FM radio (FONA800)"));
  Serial.println(F("[F] turn off FM (FONA800)"));
  Serial.println(F("[m] set FM volume (FONA800)"));
  Serial.println(F("[M] get FM volume (FONA800)"));
  Serial.println(F("[q] get FM station signal level (FONA800)"));
  Serial.println(F("[c] make phone Call"));
  Serial.println(F("[A] get call status"));
  Serial.println(F("[h] Hang up phone"));
  Serial.println(F("[p] Pick up phone"));
  Serial.println(F("[N] Number of SMSs"));
  Serial.println(F("[r] Read SMS #"));
  Serial.println(F("[R] Read All SMS"));
  Serial.println(F("[d] Delete SMS #"));
  Serial.println(F("[s] Send SMS"));
  Serial.println(F("[u] Send USSD"));
  Serial.println(F("[y] Enable network time sync (FONA 800 & 808)"));
  Serial.println(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
  Serial.println(F("[t] Get network time"));
  Serial.println(F("[G] Enable GPRS"));
  Serial.println(F("[g] Disable GPRS"));
  Serial.println(F("[l] Query GSMLOC (GPRS)"));
  Serial.println(F("[w] Read webpage (GPRS)"));
  Serial.println(F("[W] Post to website (GPRS)"));
  Serial.println(F("[S] create Serial passthru tunnel"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));
}

void flushSerial() {
  while (Serial.available())Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
} uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {} Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  } return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;
  while (true) {
    if (buffidx > maxbuff) {
      break;
    } while (Serial.available()) {
      char c =  Serial.read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)continue;
        timeout = 0;
        timeoutvalid = true;
        break;
      } buff[buffidx] = c;
      buffidx++;
    } if (timeoutvalid && timeout == 0) {
      break;
    } delay(1);
  } buff[buffidx] = 0;
  return buffidx;
}
*/
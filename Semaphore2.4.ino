#define Version "Semaphore 2.4 2021/08/02 14:30"
//   -- documentation re LoopDly
//   -- documentation re BounceArc set to 0
// "Semaphore 2.2 2021/05/02 12:00"
//   -- Add Bounce
// "Semaphore 2.1 2021/04/18 22:00"
//
// Controls Dual Semaphore Controller board from MRCS
// Color    Resist          analog        button       digital      index
// Red2      R1                A0           S4            D4          3
// Yel2      R2                A1           S5            D3          4
// Grn2      R3                A2           S6            D2          5
// Red1      R4                A3           S1            D7          0
// Yel1      R5                A6           S2            D6          1
// Grn1      R6                A7           S3            D5          2
#include <Servo.h>

// How many signals/servos to process?
#define NumSigs 2
// FullArc - the maximum degrees of arc to travel
#define FullArc 90
// Chg 2.2 vvvvvvvvvvvvv
// BounceArc - factor for arc of bounce 
//    - (green-red)/BounceArc
//    - 0 means no bounce
#define BounceArc 3
//    BounceDly - delay between moves
#define BounceDly 300
// Chg 2.2 ^^^^^^^^^^^^
// MinPotChg - the minimum ohm change we will act on
#define MinPotChg 10
// HoldMS - time before looking for a new signal 
#define HoldMS 10000
// latch = true - hold last active button
#define Latch false
// mirror = true - upper goes counter-clockwise, lower goes clockwise
#define Mirror true

// main loop delay - sample rate
#define LoopDly 1000

// debugging
#define Trace 0
// if debugging, make the LoopDly large

// note ordering:    1R 1Y 1G 2R 2Y 2G
int const PotPins[6] = {A3,A6,A7,A0,A1,A2};
int const ButPins[6] = { 7, 6, 5, 4, 3, 2};
#define RED 0
#define YEL 1
#define GRN 2
int const LEDs [2] = {9,10};// note: high = off

// servos and pins
Servo SemiP[2];
int const ServPins[2] = {12,11};
int       ServState [2]; // red/yellow/green
int       Posn [2];      // pot position
long int  PosnTM [2];    // last position time change


// #################################################
void LEDlert(int Patrn, int Repet) {
  // specific for the Dual Semaphore Controller
  // Patrn
  //   0 = slow blink - active
  //   1 = rapid blink - working
  //   2 = fast blink - problem
  int TurnOn = LOW;
  int TurnOff = HIGH;

  int ii,jj, Iner;
  Iner = (3-Patrn)*100;
  
  for (jj = 0; jj < Repet; ++jj){  
  for (ii = 0; ii < Patrn+1; ++ii){  
    digitalWrite(LEDs[0], TurnOn);
    delay(Iner);
    digitalWrite(LEDs[1], TurnOn);
    digitalWrite(LEDs[0], TurnOff);
    delay(Iner);
    digitalWrite(LEDs[1], TurnOff);
    if (jj != Repet-1){delay(Iner);}
  }// for ii
    if (jj != Repet-1){delay(Iner*5);}
  }// for jj
}// LEDlert

// Chg 2.2 vvvvvvvvvvvvv
// #################################################
void BounceRed(int Iset, int Redval) {
int Ioff, nxtpos, Grnval;
  // return to Red causes bounce
  #if (BounceArc > 0) 
  Ioff = Iset * 3;
  Grnval = analogRead(PotPins[Ioff+GRN]);
  if (Ioff == 0 && Mirror){Grnval = 1023 - Grnval;}
  Grnval = map(Grnval, 0, 1023, 0, FullArc);
  nxtpos = (Grnval - Redval)/BounceArc;
    if (Trace > 0){
      Serial.print("Bounce G: ");
      Serial.print(Grnval);
      Serial.print(" R:");
      Serial.print(Redval);
      Serial.print(" nxt: ");
      Serial.println(nxtpos);
    }// Trace
  do {
    if (Trace > 0){
      Serial.print("Bounce to ");
      Serial.println(Redval + nxtpos);
    }// Trace
  delay (BounceDly);
    SemiP[Iset].write(Redval + nxtpos);
  delay (BounceDly);
    SemiP[Iset].write(Redval);
    nxtpos /= BounceArc;
  } while (nxtpos != 0);
  #endif
}// BounceRed
// Chg 2.2 ^^^^^^^^^^^^


  // #################################################
void setup() {
  long int TuneTM;
  int PotPos[6];
  // Tuning - time watching for a pot change
  #define Tuning 15000
  
  int ii, jj, Cnt, MPin, PotVal;
  if (Trace > 0){
      Serial.begin(9600);
      Serial.println(Version);
  }// Trace
  // setup servos and variables
  for (ii = 0; ii < 6; ++ii){
    pinMode(PotPins[ii], INPUT);
    pinMode(ButPins[ii], INPUT_PULLUP);
  }// for ii
  for (ii = 0; ii < NumSigs; ++ii){
    SemiP[ii].attach(ServPins[ii]);
  // read red pot & position
    Posn[ii] = analogRead(PotPins[ii*3]);
    if (ii == 0 && Mirror){Posn[ii] = 1023 - Posn[ii];}
    SemiP[ii].write(map(Posn[ii], 0, 1023, 0, FullArc));
    PosnTM[ii] = millis();  
    ServState[ii] = ii * 3; 
    if (Trace > 0){
      Serial.print("Servo ");
      Serial.print(ii);
      Serial.print(" pot ");
      Serial.println(Posn[ii]);
    }
  }// for ii
  for (ii = 0; ii < 2; ++ii){
    pinMode(LEDs[ii], OUTPUT);
    digitalWrite(LEDs[ii], HIGH);
  }// for ii
  if (Trace > 0){// light both for startup
    LEDlert(0,1); // 
  }// Trace
  // initialize pot position
  for (jj = 0; jj < 6; ++jj){
    PotPos[jj] = analogRead(PotPins[jj]);
  }// jj
  TuneTM = millis();
  do {
    for (jj = 0; jj < 6; ++jj){
      PotVal = analogRead(PotPins[jj]);
      if (abs(PotVal - PotPos[jj]) > MinPotChg){
        TuneTM = millis();
        PotPos[jj] = PotVal;
        ii = jj / 3;
        if (ii == 0 && Mirror){PotVal = 1023 - PotVal;}
        SemiP[ii].write(map(PotVal, 0, 1023, 0, FullArc));
        }// if pot moved
    }// jj
    LEDlert(1,1); // notify tuning time
  } while (TuneTM + Tuning > millis());
  } // setup
  // #################################################

  // #################################################
void loop() {
  int iset, ioff, ii, jj, ibutn, potval, mapval;

  for (iset = 0; iset < NumSigs; ++iset){
    // check timer and skip if not yet expired
    if (PosnTM[iset] + HoldMS > millis()){continue;}
    
    ibutn = ioff = iset * 3; // default to red
    jj = 0; // # of buttons pushed
    // loop through buttons
    for (ii = ioff; ii < ioff +3; ++ii){
      potval = digitalRead(ButPins[ii]);
        if (Trace > 1){
          Serial.print("Button ");
          Serial.print(ii);
          Serial.print(" pin ");
          Serial.print(ButPins[ii]);
          Serial.print(" value ");
          Serial.println(potval);
        }// Trace
      if (potval == LOW){
        ++jj;
        ibutn = ii;
        potval = analogRead(PotPins[ibutn]);
        if (iset == 0 && Mirror){potval = 1023 - potval;}
        mapval = map(potval, 0, 1023, 0, FullArc);
        if (Trace > 0){
          Serial.print("Semaphore ");
          Serial.print(iset);
          Serial.print(" Button ");
          Serial.print(ii);
          Serial.println(" active");
         }// Trace
      }
    }// for ii
    // done scanning buttons
    if (jj > 1){LEDlert(2,10);}// if jj
    if (Trace > 0){
          Serial.print("Set ");
          Serial.print(iset);
          Serial.print(" actives ");
          Serial.print(jj);
          Serial.print(" ibutn ");
          Serial.print(ibutn);
          Serial.print(" state ");
          Serial.println(ServState[iset]);
      } // Trace

    if (jj == 0 && Latch) {ibutn = ServState[iset];}
    potval = analogRead(PotPins[ibutn]);
    if (iset == 0 && Mirror){potval = 1023 - potval;}
    mapval = map(potval, 0, 1023, 0, FullArc);
// button pushed or latch false
  if (ibutn != ServState[iset]
         && 
       (abs(potval - Posn[iset]) > MinPotChg)
       )
     {
      if (Trace > 0){
          Serial.print("Set ");
          Serial.print(iset);
          Serial.print(" state ");
          Serial.print(ServState[iset]);
          Serial.print(" button ");
          Serial.print(ibutn);
          Serial.print(" pot ");
          Serial.print(potval);
          Serial.print(" moved to ");
          Serial.print(mapval);
          Serial.println(" angle ");        
       } // Trace
     // new button state
      ServState[iset] = ibutn;
      Posn[iset]      = potval;
      PosnTM[iset]    = millis();
      SemiP[iset].write(mapval);
    
// Chg 2.2 vvvvvvvvvvvvv
    if (ibutn == ioff) // RED - call BounceRed
    {BounceRed(iset, mapval);}
// Chg 2.2 ^^^^^^^^^^^^^
     }// conditions
  }// for iset
  
  if (LoopDly > 0){
    LEDlert(0,1); // heartbeat  
    delay(LoopDly);}
  } // loop

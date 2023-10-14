// This sketch controls the dual semaphore controller available from:
// https://www.modelrailroadcontrolsystems.com/dual-semaphore-servo-controller/
//
// Actions:
// The sketch starts in Setup mode, indicated by moderately alternately flashing LEDs.
// During this time the user may adjust the board pots which correspond to the servo positions 
// for Red, Yellow, and Green positions of the servo arm.  The user may continue to change 
// the pots for as long as is needed.  after 15 seconds of no pot change, the sketch will move
// the blades to Red, Yellow + overrun, Yellow, Green, and then the selected position
// (default Red). If you need to reenter Setup, push the reset on the Nano.
//
// After Setup mode finishes, the sketch will go into normal mode, indicated by a heartbeat
// flashing rate of the LEDs.
// If the LEDs flash rapidly it indicates that more than one signal is grounded at the same time.
//
// Dual Semaphore Controller board from MRCS
// Color    Resist          analog        button       digital      index
// Red2      R1                A0           S4            D4          3
// Yel2      R2                A1           S5            D3          4
// Grn2      R3                A2           S6            D2          5
// Red1      R4                A3           S1            D7          0
// Yel1      R5                A6           S2            D6          1
// Grn1      R6                A7           S3            D5          2
//
#define Version "Semaphore 3.2a 2023/10/14"
// fix go to red if bounce = 0
//
// #define Version "Semaphore 3.2 2023/08/26 17:00"
// fix definition of FullArc to 190 degrees for best granulation
// fix bounce to use last position either yellow or green
// add YelOverun for moving past yellow position & return
// add YelOverunDlyMS to allow time for the overrun move
//
// #define Version "Semaphore 3.1a 2023/02/12 16:00"
//   -- fix BounceRed - refix 'a' to allow slow movement
// #define Version "Semaphore 3.0 2022/06/05 10:00"
//   -- fix BounceRed - refix 'a' to allow slow movement
//   -- optionally use VarSpeedServo library
//   more use of #if
//   cosmetic cleanup, tracing cleanup
//   adjustments to BounceRed and FullArc
//   UseMidYellow if true, force Yellow position to be equidistant from Red/Green
//   -- ignores the Yellow pot
//
// #define Version "Semaphore 2.4 2021/08/02 14:30"
//   -- documentation re LoopDly
//   -- documentation re BounceArc set to 0
// "Semaphore 2.2 2021/05/02 12:00"
//   -- Add Bounce
// "Semaphore 2.1 2021/04/18 22:00"
//
// optionally use the 
// VarSpeedServo library - https://github.com/netlabtoolkit/VarSpeedServo
#define VSServo true 
#if VSServo
  #include <VarSpeedServo.h>
  #define MoveFast   35
  #define MoveSlow   10
#else
  #include <Servo.h>
#endif
// How many signals/servos to process?
#define NumSigs 2
// FullArc - the maximum degrees of arc to travel
#define FullArc 180
// Chg 2.2 vvvvvvvvvvvvv
// BounceArc - factor for arc of bounce 
//    - (last-red)/BounceArc
//    - 0 means no bounce
#define BounceArc 0
//    BounceDly - delay between moves
#define BounceDly 150
// Chg 2.2 ^^^^^^^^^^^^
// MinPotChg - the minimum ohm change we will act on
#define MinPotChg 5
// HoldMS - time before looking for a new signal 
#define HoldMS 3000
// UseMidYellow = true - yellow is midway between red & green
#define UseMidYellow true
// YelOverun <> 0 - overrun the yellow position (degrees) and return (+-0)
#define YelOverun 25
// YelOverunDlyMS - time for overrun to take effect
#define YelOverunDlyMS 300
// latch = true - hold last active button
#define Latch false
// mirror = true - upper goes counter-clockwise, lower goes clockwise
#define Mirror true 
//
// DEBUGGING
#define Trace 0 
// if debugging, make the LoopDly large
// main loop delay - sample rate
#define LoopDly 1000
//
// note ordering:       1R 1Y 1G 2R 2Y 2G
int const PotPins[6] = {A3,A6,A7,A0,A1,A2};
int const ButPins[6] = { 7, 6, 5, 4, 3, 2};
#define RED 0
#define YEL 1
#define GRN 2
int const LEDs [2] = {9,10};// note: high = off
String Colors[3] = {"RED","YEL","GRN"};

// servos and pins
#if VSServo
  VarSpeedServo SemiP[NumSigs];
#else
  Servo SemiP[NumSigs];
#endif
int const ServPins  [NumSigs] = {12,11};
int       ServState [NumSigs]; // red/yellow/green
int       LastState [NumSigs]; // red/yellow/green
int       Posn      [NumSigs]; // pot position
long int  PosnTM    [NumSigs]; // last position time change
int       LastPosn  [NumSigs]; // pot position


// #################################################
void LEDlert(int Patrn, int Repet) {
  // specific for the Dual Semaphore Controller
  // Patrn
  //   0 = slow blink  - active
  //   1 = rapid blink - working
  //   2 = fast blink  - problem
  int TurnOn  = LOW;
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
void BounceRed(int Iset) {
int Ioff, nxtpos, Frmval, Redval;
  // return to Red causes bounce
  Ioff = Iset * 3;
  if (LastState[Iset] == Ioff + YEL)
    {Frmval = analogRead(PotPins[Ioff+YEL]);}
  else
    {Frmval = analogRead(PotPins[Ioff+GRN]);}
  Redval = analogRead(PotPins[Ioff+RED]);
  if (Ioff == 0 && Mirror){
    Frmval = 1023 - Frmval;
    Redval = 1023 - Redval;}
  Frmval = map(Frmval, 0, 1023, 0, FullArc);
  Redval = map(Redval, 0, 1023, 0, FullArc);
  #if BounceArc > 0 
  nxtpos = (Frmval - Redval)/BounceArc;
    #if Trace > 0
      Serial.print("Bounce Frm: ");
      Serial.print(Frmval);
      Serial.print(" Red:");
      Serial.print(Redval);
      Serial.print(" nxt: ");
      Serial.println(nxtpos);
    #endif // Trace
  do {
    #if Trace > 0
      Serial.print("Bounce to ");
      Serial.println(Redval + nxtpos);
    #endif // Trace
   #if VSServo
    SemiP[Iset].write(Redval, MoveFast, true);
    delay (BounceDly);
    SemiP[Iset].write(Redval + nxtpos, MoveFast, true);
    delay (BounceDly);
   #else
    SemiP[Iset].write(Redval);
    delay (BounceDly);
    SemiP[Iset].write(Redval + nxtpos);
    delay (BounceDly);
   #endif
    nxtpos /= BounceArc;
  } while (nxtpos != 0);
    SemiP[Iset].write(Redval);
  #endif // if BounceArc > 0
  SemiP[Iset].write(Redval);
}// BounceRed
// Chg 2.2 ^^^^^^^^^^^^


  // #################################################
void setup() {
  long int TuneTM;
  int PotPos[6];
  // Tuning - time watching for a pot change
  #define Tuning 15000
  
  int ii, jj, PotVal;
  #if Trace > 0
      Serial.begin(9600);
      Serial.println(Version);
  #endif  // Trace
  // setup servos and variables
  for (ii = 0; ii < 6; ++ii){
    pinMode(PotPins[ii], INPUT);
    pinMode(ButPins[ii], INPUT_PULLUP);
  }// for ii
  for (ii = 0; ii < NumSigs; ++ii){
    SemiP[ii].attach(ServPins[ii]);
  // read red pot & position servo
    PotVal = analogRead(PotPins[ii*3]);
    if (ii == 0 && Mirror){PotVal = 1023 - PotVal;}
    Posn[ii] = PotVal;
    SemiP[ii].write(map(PotVal, 0, 1023, 0, FullArc));
    PosnTM[ii] = millis();  
    ServState[ii] = ii * 3; 
    LastState[ii] = ii * 3; 
    #if Trace > 0
      Serial.print("Servo ");
      Serial.print(ii);
      Serial.print(" RED pot ");
      Serial.println(Posn[ii]);
    #endif
  }// for ii
  for (ii = 0; ii < 2; ++ii){
    pinMode(LEDs[ii], OUTPUT);
    digitalWrite(LEDs[ii], HIGH);
  }// for ii
  #if Trace > 0  // light both for startup
    LEDlert(0,1); // 
  #endif // Trace
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
        #if Trace > 0  
          Serial.print("Servo ");
          Serial.print(ii);
          Serial.print(" " + Colors[jj%3] + " pot changed to ");
          Serial.print(PotPos[jj]);
          Serial.print(" map ");
          Serial.println(map(PotVal, 0, 1023, 0, FullArc));
        #endif // Trace
        }// if pot moved
    }// jj
    LEDlert(1,1); // notify tuning time
  } while (TuneTM + Tuning > millis());
  #if UseMidYellow
    PotPos[YEL]   = (PotPos[RED]   + PotPos[GRN])   / 2;
    PotPos[YEL+3] = (PotPos[RED+3] + PotPos[GRN+3]) / 2;
  #endif // UseMidYellow
  // now show the positions selected
    for (jj = 0; jj < 6; ++jj){
        ii = jj / 3;
        PotVal = PotPos[jj];
        if (ii == 0 && Mirror){PotVal = 1023 - PotVal;}
		if (YelOverun != 0 && jj%3 == YEL){
          SemiP[ii].write(map(PotVal, 0, 1023, 0, FullArc)+YelOverun);
          delay(YelOverunDlyMS);}
        SemiP[ii].write(map(PotVal, 0, 1023, 0, FullArc));
        #if Trace > 0  
          Serial.print("Servo ");
          Serial.print(ii);
          Serial.print(" moving to " + Colors[jj%3] + " pot ");
          Serial.print(PotVal);
          Serial.print(" map ");
          Serial.println(map(PotVal, 0, 1023, 0, FullArc));
        #endif // Trace
        delay(2000);
        }// for jj
        BounceRed(0);
        BounceRed(1);
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
    // loop through buttons for set
    for (ii = ioff; ii < ioff +3; ++ii){
      potval = digitalRead(ButPins[ii]);
        #if Trace > 5 
          Serial.print("Button ");
          Serial.print(ii);
          Serial.print(" pin ");
          Serial.print(ButPins[ii]);
          Serial.print(" value ");
          Serial.println(potval);
        #endif // Trace
      if (potval == LOW){
        ++jj;
        ibutn = ii;
        #if Trace > 5
          Serial.print("Semaphore ");
          Serial.print(iset);
          Serial.print(" " + Colors[ii%3]+ " Button ");
          Serial.print(ii);
          Serial.println(" active");
         #endif // Trace
      }
    }// for ii
    // done scanning buttons
    if (jj > 1){LEDlert(2,10);}// if multipins hit
    #if Trace > 5
          Serial.print("Set ");
          Serial.print(iset);
          Serial.print(" actives ");
          Serial.print(jj);
          Serial.print(" ibutn ");
          Serial.print(ibutn);
          Serial.print(" state ");
          Serial.println(ServState[iset]);
      #endif // Trace

    if (jj == 0 && Latch) {ibutn = ServState[iset];}
    if (UseMidYellow && (ibutn%3 == YEL)) 
      {potval = (analogRead(PotPins[ioff+RED]) + analogRead(PotPins[ioff+GRN])) / 2;}
	  else 
      {potval = analogRead(PotPins[ibutn]);} // UseMidYellow
    if (iset == 0 && Mirror){potval = 1023 - potval;}
    mapval = map(potval, 0, 1023, 0, FullArc);
// button pushed or latch false
  if (ibutn != ServState[iset]
         && 
       (abs(potval - Posn[iset]) > MinPotChg))
     {
      #if Trace > 0
          Serial.print("Set ");
          Serial.print(iset);
          Serial.print(" state ");
          Serial.print(ServState[iset]);
          Serial.print(" " + Colors[ibutn%3] + " button ");
          Serial.print(ibutn);
          Serial.print(" pot ");
          Serial.print(potval);
          Serial.print(" moved to ");
          Serial.print(mapval);
          Serial.println(" angle ");        
       #endif // Trace
     // new button state
      ServState[iset] = ibutn;
      Posn  [iset]    = potval;
      PosnTM[iset]    = millis();
// Chg 2.2 vvvvvvvvvvvvv
    if (ibutn == ioff && BounceArc > 0) // RED - call BounceRed
      {BounceRed(iset);}
	else // not RED
	{
	 if (ibutn%3 == YEL && YelOverun != 0)
	 { // do overrun first and delay
      #if Trace > 0
          Serial.print("Set ");
          Serial.print(iset);
          Serial.print(" " + Colors[ibutn%3] + " button ");
          Serial.print(ibutn);
          Serial.print(" overrun ");
          Serial.print(YelOverun);
          Serial.print(" moved to ");
          Serial.print(mapval+YelOverun);
          Serial.println(" angle ");        
       #endif // Trace
     #if VSServo
      SemiP[iset].write(mapval+YelOverun,MoveSlow,true);
     #else
      SemiP[iset].write(mapval+YelOverun);
     #endif
     delay(YelOverunDlyMS);
	 }
// Chg 2.2 ^^^^^^^^^^^^^
     #if VSServo
      SemiP[iset].write(mapval,MoveSlow,false);
     #else
      SemiP[iset].write(mapval);
     #endif
    } // if RED
    LastState[iset] = ibutn;
    }// new state and min chg
  }// for iset
  
  if (LoopDly > 0){
    LEDlert(0,1); // heartbeat  
    delay(LoopDly);}
  } // loop

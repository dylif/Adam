    
    
    #include <Servo.h>
    
    Servo RHip;
    Servo RThigh;
    Servo RKnee;
    Servo RAnkle;
    
    Servo LHip;
    Servo LThigh;
    Servo LKnee;
    Servo LAnkle;
    
    // Lean back
    //LThigh.write(LThigh.read()-5);
    //  RThigh.write(RThigh.read()+5);
    
    int RHipCenter = 90;  
    int RThighCenter = 70;
    int RKneeCenter = 164;
    int RAnkleCenter = 90;
    
    int LHipCenter = 90;
    int LThighCenter = 110;
    int LKneeCenter = 26;//lthigh
    int LAnkleCenter = 104;
    
    
    int walking = 0;
    
    int d = 100;
    int data = 0;
    void setup()
    {
    
    Serial.begin(9600);
    
    RHip.attach(13);
    RThigh.attach(12);
    RKnee.attach(11);
    RAnkle.attach(10);
    LHip.attach(2);
    LThigh.attach(3);
    LKnee.attach(4);
    LAnkle.attach(5);
    
    }
    
    void loop()
    {
    if (Serial.available() > 0) {

    data = Serial.read();	 // read the incoming byte:
    
    switch(data)
    {
	case 's' : stand();  break;
	case 'a' : crouch();  break ;
	case ' ' : LeanRight();  break;
	case 'd' : LeanLeft();  break;
	case 'f' : walk();  break ;

case '1' : RightAnkleForwards();  break ;
case '2' : RightAnkleBackwards();  break ;
case '3' : LeftAnkleForwards();  break ;
case '4' : LeftAnkleBackwards();  break ;
case '5' : RightKneeForwards();  break ;
case '6' : RightKneeBackwards();  break ;
case '7' : LeftKneeForwards();  break ;
case '8' : LeftKneeBackwards();  break ;
case '9' : RightThighForwards();  break ;
case '0' : RightThighBackwards();  break ;
case 'q' : LeftThighForwards();  break ;
case 'w' : LeftThighBackwards();  break ;
case 'e' : RightHipForwards();  break ;
case 'r' : RightHipBackwards();  break ;
case 't' : LeftHipForwards();  break ;
case 'y' : LeftHipBackwards();  break ;


    }
    }
    
   // walk();
   // else
   // {
     // stand();
   // }
    //stand();
    //crouch();
    //delay(15000);  //  if (walking == 0) {
    //StandRightForward();
    //delay(15000);       //StandLeftForward();
    //delay(15000);       //stand();
    //      delay(5000);  //  }
    
    //swayright();
    //swayleft();
    //StandRightForward();
    //delay(5000);   stepleft();
    //delay(5000);  //  walk();
    
    }
    
      void LeanLeft() {
    Serial.println("LeanLeft");
    LHip.write(LHip.read()-5);
    RHip.write(RHip.read()-5);
    RAnkle.write(RAnkle.read()+5); 
    LAnkle.write(LAnkle.read()+5);
    delay(d);                       }
    
    void LeanRight() {
    Serial.println("LeanRight");
    LHip.write(LHip.read()+5);
    RHip.write(RHip.read()+5);
    RAnkle.write(RAnkle.read()-5);
    LAnkle.write(LAnkle.read()-5);
    delay(d);                       }
    
    void walk()
    {
     LeanLeft();
     delay(2000);
     LeanRight();
     delay(2000);
    }
    
    
    void stepleft()
    {
    StandRightForward();
    LeanBack();
    LeanRight();
    LiftRightHip();
    SpreadLegs();
    SpreadLegs();
    LeanRight();
    LeanRight();
    LeanRight();
    LiftLeftFoot();
    LiftLeftHip();
    LiftLeftHip();
    SpreadLegs();
    LiftLeftFoot();
    LeftLegForward();
    LeftLegForward();
    LeftLegForward();
    StretchRightKnee();
    DownLeftFoot();
    LeanBack();
    DownLeftFoot(); 
    LeanLeft();    
    LeanLeft();
    LiftRightHip();
    LiftRightHip();
    DownLeftFoot();
    LeanBack();    
    StandLeftForward();
    //LeanBack();
    //LeanBack();    
    LeanLeft();
    LiftLeftHip();
    //Low battery, turn on this:
    //SpreadLegs();
    SpreadLegs();
    LeanLeft();
    LeanLeft();    
    LiftRightFoot();
    LiftRightHip();
    LiftRightHip();
    SpreadLegs();    
    LiftRightFoot();
    LeanBack();
    LeanBack();   
    RightLegForward();
    RightLegForward();
    RightLegForward();
    RightLegForward();
    StretchLeftKnee();
    DownRightFoot();
    LeanBack();
    DownRightFoot();
    LeanRight();
    LeanRight();
    LiftLeftHip();
    LiftLeftHip();
    DownRightFoot();
    LeanBack();
    StandRightForward();
    
    }
    
    /*
    void swayright()
    {
    
    LeanRight();
    LiftRightHip();
    SpreadLegs();
    LeanRight();
    //LeanRight();
    LeanRight();
    LeanRight();
    
    delay(5000);
    LeanLeft();
    LeanLeft();
    LiftLeftHip();
    
    }
    
    void swayleft()
    {
    
    LeanLeft();
    LiftLeftHip();
    SpreadLegs();
    LeanLeft();
    //LeanLeft();
    LeanLeft();
    LeanLeft();
    
    delay(5000);
    LeanRight();
    LeanRight();
    LiftRightHip();   stand();
    
    }
    */
    
    void stand()
    {

    Serial.println("STAND");
    RHip.write(RHipCenter);
    RThigh.write(RThighCenter);
    RKnee.write(RKneeCenter);
    RAnkle.write(RAnkleCenter);
    LHip.write(LHipCenter);
    LThigh.write(LThighCenter);
    LKnee.write(LKneeCenter);
    LAnkle.write(LAnkleCenter);

    delay(d);
  
    }
    
    
    void crouch()
    {
    
    Serial.println("crouch");
    RThigh.write(RThigh.read()-10);
    RKnee.write(RKnee.read()-20);
    //RAnkle.write(RAnkle.read()-10);
    LThigh.write(LThigh.read()+10);
    LKnee.write(LKnee.read()+20);
    //LAnkle.write(LAnkle.read()+10);
    delay(d);
    }
    
    void StandRightForward()
    {
    Serial.println("StandRightForward");
    RHip.write(RHipCenter);
    RThigh.write(RThighCenter);
    RKnee.write(RKneeCenter);
    RAnkle.write(RAnkleCenter);
    LHip.write(LHipCenter);
    LThigh.write(LThighCenter-20);
    LKnee.write(LKneeCenter);
    LAnkle.write(LAnkleCenter+20);
    delay(d);  }
    
    void StandLeftForward()
    {
    Serial.println("StandLeftForward");
    RHip.write(RHipCenter);
    RThigh.write(RThighCenter+20);
    RKnee.write(RKneeCenter);
    RAnkle.write(RAnkleCenter-20);
    LHip.write(LHipCenter);
    LThigh.write(LThighCenter);
    LKnee.write(LKneeCenter);
    LAnkle.write(LAnkleCenter);
    delay(d);  }
    
    void ResetHip()
    {
    Serial.println("ResetHip");
    LHip.write(LHip.read()+5);
    RHip.write(RHip.read()+5);
    delay(d);                       }
    
    void LiftLeftHip()
    {
    Serial.println("LiftLeftHip");
    LHip.write(RHipCenter);
    delay(d);                       }
    
    void LiftRightHip()
    {
    Serial.println("LiftRightHip");
    RHip.write(RHip.read()-5);
    delay(d);                       }
    
    void SpreadLegs()
    {
    Serial.println("SpreadLegs");
    LHip.write(LHip.read()-5);
    RHip.write(RHip.read()+5);
    delay(d);                       }
    
    void CollectLegs()
    {
    Serial.println("CollectLegs");
    LHip.write(LHip.read()+5);
    RHip.write(RHip.read()-5);
    delay(d);                       }
    
  
    
    void LeftLegForward() {
    Serial.println("LeftFootForward");
    LThigh.write(LThigh.read()+10);
    LKnee.write(LKnee.read()-10);
    LAnkle.write(LAnkle.read()-10);
    delay(d);                       }
    
    void LeftFootForward() {
    Serial.println("LeftFootForward");
    //LThigh.write(LThigh.read()+10);
    LKnee.write(LKnee.read()-10);
    LAnkle.write(LAnkle.read()-5);
    delay(d);                       }
    
    void LeanBack()
    {
    Serial.println("LeanBack");
    LThigh.write(LThigh.read()-5);
    RThigh.write(RThigh.read()+5);
    delay(d);  }
    
    void LeanForward()
    {
    Serial.println("LeanForward");
    LThigh.write(LThigh.read()+10);
    RThigh.write(RThigh.read()-10);
    delay(d);  }
    
    void LeftThighBackward() {
    Serial.println("LeftFootBack");
    LThigh.write(LThigh.read()-10);
    delay(d);                       }
    
    void LeftThighForward() {
    Serial.println("LeftThighForward");
    LThigh.write(LThigh.read()+10);
    delay(d);                       }
    
    void StretchLeftKnee() {
    Serial.println("StretchLeftKnee");
    LKnee.write(LKnee.read()-5);
    delay(d);                       }
    
    void LiftLeftFoot() {
    Serial.println("LiftLeftFoot");
    LThigh.write(LThigh.read()+10);
    LKnee.write(LKnee.read()+10);
    LAnkle.write(LAnkle.read()-10);
    delay(d);                       }
    
    void DownLeftFoot() {
    Serial.println("DownLeftFoot");
    LThigh.write(LThigh.read()-10);
    LKnee.write(LKnee.read()-5);
    LAnkle.write(LAnkle.read()-5);
    delay(d);                       }
    
    void RightLegForward() {
    Serial.println("RightLegForward");
    RThigh.write(RThigh.read()-10);
    RKnee.write(RKnee.read()+10);
    RAnkle.write(RAnkle.read()+10);
    delay(d);                       }
    
    void RightFootForward() {
    Serial.println("RightFootForward");
    //RThigh.write(RThigh.read()-10);
    RKnee.write(RKnee.read()+10);
    RAnkle.write(RAnkle.read()+5);
    delay(d);                       }
    
    void RightThighBackward()
    {
    Serial.println("RightFootForward");
    RThigh.write(RThigh.read()+10);
    delay(d);    }
    
    void RightThighForward()
    {
    Serial.println("RightFootForward");
    RThigh.write(RThigh.read()-10);
    delay(d);  }
    
    /*void RightToeUp()
    {
    Serial.println("RightAnkleForward");
    RAnkle1.write(RAnkle1.read()-5);
    delay(d);                       }
    
    void RightToeDown()
    {
    Serial.println("RightAnkleBackward");
    RAnkle1.write(RAnkle1.read()+5);
    delay(d);                       }
    
    
    void LeftToeUp()
    {
    Serial.println("LeftAnkleForward");
    LAnkle1.write(LAnkle1.read()+5);
    delay(d);                       }
    
    void LeftToeDown()
    {
    Serial.println("LeftAnkleBackward");
    LAnkle1.write(LAnkle1.read()-5);
    delay(d);                       }
    */
    
    void StretchRightKnee() {
    Serial.println("StretchRightKnee");
    RKnee.write(RKnee.read()+5);
    delay(d);                       }
    
    void LiftRightFoot() {
    Serial.println("LiftRightFoot");
    RThigh.write(RThigh.read()-10);
    RKnee.write(RKnee.read()-10);
    RAnkle.write(RAnkle.read()+10);
    delay(d);                       }
    
    void DownRightFoot() {
    Serial.println("DownRightFoot");
    RThigh.write(RThigh.read()+10);
    RKnee.write(RKnee.read()+5);
    RAnkle.write(RAnkle.read()+5);
    delay(d);                       }
    
    void ResetLeftAnkle()
    {
    Serial.println("ResetLeftAnkle");
    LAnkle.write(RAnkleCenter);
    delay(d);                       }
    
    void ResetRightAnkle()
    {
    Serial.println("ResetLeftAnkle");
    LAnkle.write(RAnkleCenter);
    delay(d);                       }


void RightThighForwards()
{
  Serial.println("RTP");
  RThigh.write(RThigh.read()-2);
  
}
void RightThighBackwards()
{
  Serial.println("RTN");
  RThigh.write(RThigh.read()+2);
 
}
void LeftThighForwards()
{
  Serial.println("LTP");
  LThigh.write(LThigh.read()+2);
}
void LeftThighBackwards()
{
  Serial.println("LTN");
  LThigh.write(LThigh.read()-2);
}
void RightHipForwards()
{
  Serial.println("RHP");
  RHip.write(RHip.read()-2);
}
void RightHipBackwards()
{
  Serial.println("RHN");
   RHip.write(RHip.read()+2);
}
void LeftHipForwards()
{
  Serial.println("LHP");
   LHip.write(LHip.read()-2);
}
void LeftHipBackwards()
{
  Serial.println("LHN");
  LHip.write(LHip.read()+2);
}
void RightKneeForwards()
{
  Serial.println("RKP");
  RKnee.write(RKnee.read()+2);
}
void RightKneeBackwards()
{
  Serial.println("RKN");
  RKnee.write(RKnee.read()-2);
}
void LeftKneeForwards()
{
  Serial.println("RKP");
  LKnee.write(LKnee.read()-2);
}
void LeftKneeBackwards()
{
  Serial.println("LKN");
  LKnee.write(LKnee.read()+2);
}

void RightAnkleForwards()
{
  Serial.println("RAP");
   RAnkle.write(RAnkle.read()-2);
}
void RightAnkleBackwards()
{
  Serial.println("RAN");
   RAnkle.write(RAnkle.read()+2);
}

void LeftAnkleForwards()
{
  Serial.println("LAP");
  LAnkle.write(LAnkle.read()-2);
}
void LeftAnkleBackwards()
{
  Serial.println("LAN");
  LAnkle.write(LAnkle.read()+2);
}

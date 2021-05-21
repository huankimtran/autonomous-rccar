#include <Servo.h>

#define LED 13

class SteerServo
{
  public:
    
    int pin;             // This is the pin that servo is connected to
    double steerValue;   // This is in the range [-100.0, 100.0] percent. 100% is max right turn and -100% is max left turn
    Servo servo;

    // These constans are tuned with sg90 running at ~5V
    const int CENTER=104;
    const int MAXR=165;
    const int MAXL=36;

    /**
     * THis function set the steer value
     * vl is between [-100, 100] percent
     * -100 is max left turn, and 100 is max right turn
     */ 
    void steer(double vl)
    {
      if(vl > 100.0)
      {
        vl = 100.0;
      }
      if(vl < -100.0)
      {
        vl = -100.0;
      }
      this->steerValue = vl;
      // Calculate real pwm value
      int pwm;
      if(vl > 0.0 )
      {
        pwm = CENTER + (MAXR - CENTER) * (vl/100.0);
      }
      else
      {
        pwm = CENTER - (CENTER - MAXL) * (-vl/100.0);        
      }
      // Write the actual value to the PWM
      this->servo.write(pwm);
    }
      
    SteerServo()
    {
      // Weird runtime error if we set up the servo and attach it in this constructor
      // If we create a global SteerServo variable, and attach the serve to the pin in the constructor
      // THis program would not work
      // However, if we create a global SteerServo, and only attach to servo, in the setup funciton, or later on, it works
      // So in this funciton, just initialize steerValue to dummy value
      // The actual work will be in the attach function below, which should be called in the setup() function or below the setup()
    }

    void attach(int ctrlPin)
    {
      this->pin = ctrlPin;
      this->servo.attach(ctrlPin);
      this->steerValue = 0;
      this->steer(0);
    }
};

class ThrottleMotor
{
  public:
  
  int fwdPin;
  int bwdPin;
  double throttleAmount;

  /**
   * This function set the throttle amount
   * vl is between [-100, 100] percent
   * with 100 as moving forward as full power
   * and -100 as moving backward as full power
   */ 
  void throttle(double vl)
  {
    if(vl > 100.0)
    {
      vl = 100.0;
    }
    if(vl < -100.0)
    {
      vl = -100.0;
    }
    
    this->throttleAmount = vl;
    // Calculate real pwm value
    int pwm;
    if(vl > 0.0 )
    {
      // Move forward
      pwm = 255 * vl/100.0;
      analogWrite(this->bwdPin, 0);
      analogWrite(this->fwdPin, pwm);
    }
    else
    {
      // Move backward
      pwm = -255 * vl/100.0;
      analogWrite(this->bwdPin, pwm);
      analogWrite(this->fwdPin, 0);
    }
  }
  
  ThrottleMotor(int forwardPin, int backwardPin)
  {
    // pin setup
    this->fwdPin = forwardPin;
    this->bwdPin = backwardPin;
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    // Initilize value
    this->throttleAmount = 0;
    this->throttle(0.0);
  }
};

// Program: Control DC motors using BTS7960 H Bridge Driver.
//==========================================================================
// Connection to the BTS7960 board:
// BTS7960 Pin 1 (RPWM) to Arduino pin 5(PWM)
// BTS7960 Pin 2 (LPWM) to Arduino pin 6(PWM)
// BTS7960 Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
// BTS7960 Pin 8 (GND) to Arduino GND
// BTS7960 Pin 5 (R_IS) and 6 (L_IS) not connected

ThrottleMotor throttleMotor(5,6);

// The setup control the steering using an sg90 connected to pin 3

SteerServo steerMotor;


// Command parsing
// There are 2 command:
//  + 's steer_amount;'
//  + 't throttle_amount;'
//  commands always terminate with ;

char parsingBuff[100]={'\0'};
char rdOut, cmd;
int len = 0;
int amount;       // This is supposed to be double but Arduino sscanf does not seem to work with double and float so let just use integer
int itemParsed;

void setup()
{
  // Serial to get command from host
  Serial.begin(9600);
  // Set up LED indicator for debug purposes
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  // Attach the pin to the sg90 here, if this is done is the SteerServo class's constructor, the servo would not work properly, why? I don't know
  steerMotor.attach(3);
}

void loop()
{
  while(Serial.available())
  {
    rdOut = Serial.read();
    if(rdOut == ';')
    {
      // Command terminated
      // Terminate the parsing buff
      parsingBuff[len] = '\0';
      // Parse the command
      itemParsed = sscanf(parsingBuff, "%c %d",&cmd, &amount);
      if(itemParsed == 2)
      {
        // should only has 2 parsed items, otherwise, something wrong happened, don't parse
        // Let parse
        if(cmd == 's')
        {
          // Steering 
          steerMotor.steer(amount);
        }
        else if(cmd == 't')
        {
          // Thorttle
          throttleMotor.throttle(amount);
        }
      }
      // Reset parsing mechanism
      parsingBuff[0]='\0';
      len=0;
    }
    else
    {
      parsingBuff[len++] = rdOut; 
    }

    // DEBUG
    if( cmd == 't')
    {
      if(amount > 50 || amount < -50)
      {
        digitalWrite(LED, HIGH);
      }
      else
      {
        digitalWrite(LED, LOW);      
      }      
    }
    else if(cmd == 's')
    {
      if(amount < 50 && amount > -50)
      {
        digitalWrite(LED, HIGH);
      }
      else
      {
        digitalWrite(LED, LOW);      
      } 
    }
    else
    {
      digitalWrite(LED, LOW);
    }

  }
}

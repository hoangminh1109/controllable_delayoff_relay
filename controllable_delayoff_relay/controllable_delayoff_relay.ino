/**** HEADER INCLUDE ****/
#include <timer.h>

#include <ooPinChangeInt.h> // necessary otherwise we get undefined reference errors.
#include <AdaEncoder.h>

#include "LedControl.h"

/**** INTERNAL DEFINITIONS ****/

#define BUZZER 1
#define SERIAL 0

#define STATE_IDLE      0
#define STATE_RUNNING   1
#define STATE_SLEEPING  2

#define DEFAULT_TIME    30     /* 1 minutes */
#define MIN_TIME        5     /* 10 seconds */
#define MAX_TIME        300    /* 5 minutes */
#define SLEEP_TIME      120    /* 2 minutes */
#define notifyState_INTERVAL 30    /* notifyState every 30sec */

/**** PIN DEFINITION ****/
#define PIN_LED           13      // the number of the LED pin
#define PIN_ENCODER_CLK   5        //the clk attach to pin 2
#define PIN_ENCODER_DT    6         //the dt pin attach to pin 3
#define PIN_ENCODER_SW    7        //the sw pin attach to pin 4

/*
 Now we need a LedControl to work with.
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 */
#define PIN_SPI_DIN       12
#define PIN_SPI_CLK       11
#define PIN_SPI_CS        10
#define PIN_CLOCK_DOT1    8
#define PIN_CLOCK_DOT2    9

#define PIN_RELAY         4
#define PIN_BUZZER         2

/**** GLOBAL VARIABLES ****/

int set_time = DEFAULT_TIME;
int elapsed_time = 0;

int state = STATE_IDLE;
int pointState = HIGH;

/**** PERIPHERAL DEFINITIONS ****/
AdaEncoder encoderA = AdaEncoder('a', PIN_ENCODER_CLK, PIN_ENCODER_DT);
LedControl lc=LedControl(PIN_SPI_DIN, PIN_SPI_CLK, PIN_SPI_CS, 2);

/**** PERIPHERAL CONTROL SPECIFIC FUNCTIONS ****/
int getEncoderTurn() 
{
  int turn = 0;
  int clicks;
  AdaEncoder *thisEncoder=NULL;
  thisEncoder=AdaEncoder::genie();
  if (thisEncoder != NULL) {
    clicks=thisEncoder->query();
    if (clicks > 0) {
      turn = 1;
    }
    if (clicks < 0) {
       turn = -1;
    }
  }
  return turn;
}


int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
boolean switchPressed()
{
  boolean pressed = false;
  
  // read the state of the switch into a local variable:
  int reading = digitalRead(PIN_ENCODER_SW);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == LOW) {
        pressed = true;
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  // return
  return pressed;
}

void displayTime(int t)
{
  int minutes = t/60;
  int seconds = t%60;
  lc.setDigit(0,0,minutes/10,false);
  lc.setDigit(0,1,minutes%10,false);
  lc.setDigit(1,0,seconds/10,false);
  lc.setDigit(1,1,seconds%10,false);  
  digitalWrite(PIN_CLOCK_DOT1, pointState);
  digitalWrite(PIN_CLOCK_DOT2, pointState);
}

void clockDotOn()
{
  digitalWrite(PIN_CLOCK_DOT1, HIGH);
  digitalWrite(PIN_CLOCK_DOT2, HIGH);    
}

void clockDotOff()
{
  digitalWrite(PIN_CLOCK_DOT1, LOW);
  digitalWrite(PIN_CLOCK_DOT2, LOW);  
}

void closeRelay()
{
  digitalWrite(PIN_RELAY, HIGH);
}

void openRelay()
{
  digitalWrite(PIN_RELAY, LOW);
}

void buzzerOn()
{
#if(BUZZER)
  digitalWrite(PIN_BUZZER, HIGH);
#endif
}

void buzzerOff()
{
#if(BUZZER)
  digitalWrite(PIN_BUZZER, LOW);
#endif
}

/**** TIMER DEFINITINOS FUNCTIONS ****/
auto timer = timer_create_default();
auto timer2 = timer_create_default();
auto timer3 = timer_create_default();

bool tick_1sec()
{
  elapsed_time++;
  //Serial.println(elapsed_time);
  return true;
}

bool tick_500msec()
{
  if (state == STATE_RUNNING)
  {
    pointState = !pointState;
  }
  return true;
}

int notifyState = LOW;
int notifyCount = 0;
bool tick_100msec()
{
  if (state == STATE_RUNNING)
  {
    int remaining_time;
    remaining_time = set_time - elapsed_time;

    if (remaining_time < set_time)
    {
      if (notifyState == HIGH)
      {
        notifyState = LOW;
        buzzerOff();
      }
      else
      {
        if (remaining_time%notifyState_INTERVAL == 0)
        {
          if (notifyCount < 2)
          {
            notifyState = HIGH;
            buzzerOn();
            notifyCount++;
          }
        }
        else
        {
          notifyCount = 0;
        }
      }
    }
  }
  return true;
}


/**** STATE MACHINE PROCESSING FUNCTIONS ****/
void processIDLE()
{
  if (switchPressed())
  {
    digitalWrite(PIN_LED, HIGH);
    buzzerOn();
    delay(100);  
    digitalWrite(PIN_LED, LOW);
    buzzerOff();
    Serial.println("Switch pressed.");

    elapsed_time = 0;
    pointState = LOW;
    notifyState = LOW;
    notifyCount = 0;
    state = STATE_RUNNING;
    closeRelay();
    Serial.println("state = STATE_RUNNING");
  }

  int turn = getEncoderTurn();
  if (turn != 0)
  {
    Serial.print("set_time = ");Serial.println(set_time);
    Serial.print("turn = ");Serial.println(turn);
    set_time = set_time + 5*turn;
    if (set_time < MIN_TIME) { set_time = MIN_TIME; } // min time
    if (set_time > MAX_TIME) { set_time = MAX_TIME; } // max time
    elapsed_time = 0;
    Serial.print("Set time = "); Serial.println(set_time);
  }

  if (elapsed_time < SLEEP_TIME)
  {
    displayTime(set_time);
  }
  else
  {
    lc.shutdown(0,true);
    lc.shutdown(1,true);
    clockDotOff();

    buzzerOn();
    delay(1000);  
    buzzerOff();
    elapsed_time = 0;
    pointState = LOW;    
    state = STATE_SLEEPING;
    Serial.println("state = STATE_SLEEPING");
  }
}


void processRUNNING()
{
  int remaining_time;

  if (switchPressed())
  {
    digitalWrite(PIN_LED, HIGH);
    delay(100);  
    digitalWrite(PIN_LED, LOW);
    Serial.println("Switch pressed.");

    elapsed_time = set_time; /* stop running */
  }

  remaining_time = set_time - elapsed_time;
  displayTime(remaining_time);

  if (remaining_time == 0)
  {
    pointState = HIGH;
    state = STATE_IDLE;
    elapsed_time = 0;
    openRelay();

    while (getEncoderTurn()!=0){ } //ignore all encoder turns

    /* flashing */
    for (int i = 0; i < 5; i++)
    {
      lc.shutdown(0,false);
      lc.shutdown(1,false);
      clockDotOn();
      buzzerOn();
      //lc.setIntensity(0,8);
      //lc.setIntensity(1,8);
      delay(200);
      lc.shutdown(0,true);
      lc.shutdown(1,true);
      clockDotOff();
      buzzerOff();
      delay(200);
    }
    lc.shutdown(0,false);
    lc.shutdown(1,false);
    clockDotOn();
    
    Serial.println("state = STATE_IDLE");
    
  }
}

void processSLEEPING()
{
  if (switchPressed())
  {
    digitalWrite(PIN_LED, HIGH);
    buzzerOn();
    delay(100);  
    digitalWrite(PIN_LED, LOW);
    buzzerOff();
    Serial.println("Switch pressed.");
    elapsed_time = 0;
    set_time = DEFAULT_TIME;
    pointState = HIGH;
    state = STATE_IDLE;
    while (getEncoderTurn()!=0){ } //ignore all encoder turns
    Serial.println("state = STATE_IDLE");

    //wake up display
    lc.shutdown(0,false);
    lc.setIntensity(0,8);
    lc.clearDisplay(0);
    lc.shutdown(1,false);
    lc.setIntensity(1,8);
    lc.clearDisplay(1);
    clockDotOn();
    
  }
  
}

/**** INITIAL SETUP ****/
void setup() {

  /* pin mode */
  pinMode(PIN_ENCODER_CLK, INPUT);
  pinMode(PIN_ENCODER_DT, INPUT);
  pinMode(PIN_ENCODER_SW, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_CLOCK_DOT1, OUTPUT);
  pinMode(PIN_CLOCK_DOT2, OUTPUT);

  /* Serial */
#if (SERIAL)
  Serial.begin(115200);
#endif
  Serial.println("Hello Rotary Encoder");  

  // set initial LED state
  digitalWrite(PIN_LED, LOW);


  /* display on */
  lc.shutdown(0,false);
  lc.setIntensity(0,8);
  lc.clearDisplay(0);
  lc.shutdown(1,false);
  lc.setIntensity(1,8);
  lc.clearDisplay(1);
  clockDotOn();

  // set initial relay state
  openRelay();

  /* timer */
  timer.every(1000, tick_1sec);
  timer2.every(500, tick_500msec);
  timer3.every(100, tick_100msec);

  // internal variables
  state = STATE_IDLE;
  set_time = DEFAULT_TIME;
  elapsed_time = 0;
  Serial.println("state = STATE_IDLE");

  //buzzer on
  buzzerOn();
  delay(100);
  buzzerOff();
}

/**** LOOP ****/
void loop()
{
  switch(state)
  {
    case STATE_RUNNING:
      processRUNNING();
      break;
    case STATE_SLEEPING:
      processSLEEPING();
      break;
    default:
    case STATE_IDLE:
      processIDLE();
      break;
  }

  /* tick the timer */
  timer.tick(); // tick the timer
  timer2.tick(); // tick the timer
  timer3.tick(); // tick the timer
}

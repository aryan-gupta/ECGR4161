//*******************************************************************
// Blink - verify my MSP432 board works by blinking two LEDs
// Aryan Gupta, 2020-06-08
//*******************************************************************

/* virtual pins 73-78 */
// These are already defined in 
// ~/.energia15/packages/energia/hardware/msp432r/5.29.0/cores/msp432r/ti/runtime/wiring/../../../../../variants/MSP_EXP432P401R/pins_energia.h:40:22
//#define RED_LED   75   /*  75 - P2.0 RED_LED */
//#define GREEN_LED 76   /*  76 - P2.1 GREEN_LED */
//#define BLUE_LED  77   /*  77 - P2.2 BLUE_LED */
//#define LED       78   /*  78 - P1.0 LED1 (red) */

constexpr int delay_led = 500;

void setup() {    // put your setup code here, to run once:
  // initialize two digital pins as outputs.
  pinMode(      78, OUTPUT);  //RED LED
  pinMode(  RED_LED, OUTPUT);  //RGB LED - blue color
  pinMode(GREEN_LED, OUTPUT);
  pinMode( BLUE_LED, OUTPUT);
}

void loop() {    // put your main code here, to run repeatedly:  
  digitalWrite(RED_LED, HIGH);   // turn the RED LED on (HIGH is the voltage level)
  delay(delay_led);
  digitalWrite(RED_LED, LOW);
  
  digitalWrite(BLUE_LED, HIGH);
  delay(delay_led);
  digitalWrite(BLUE_LED, LOW);
  
  digitalWrite(GREEN_LED, HIGH);
  delay(delay_led);
  digitalWrite(GREEN_LED, LOW);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(delay_led);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  delay(delay_led);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  delay(delay_led);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  delay(delay_led);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}

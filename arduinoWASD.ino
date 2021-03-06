#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;
volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21.5

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin


//power-saving
#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

//#define ALEX_LENGTH 8
//#define ALEX_BREADTH 3

//Alex Diagonal
//float AlexDiagonal = 0.0;
//float AlexCirc = 0.0;

/*
      Alex's State Variables
*/
//
// Store the ticks from Alex's left and
// right encoders for moving forward/backward
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;


//ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;


//variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//variables to keep track of turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);


  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket; //TPacket is the data type. okPacket is the variable. so okPacket will contains the members of the struct
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.

  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.

void leftISR()
{

  switch (dir)
  {
    case FORWARD:
      leftForwardTicks++;
      break;
    case BACKWARD:
      leftReverseTicks++;
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }

  if (dir == FORWARD)
    forwardDist = ((float) leftForwardTicks / COUNTS_PER_REV) * WHEEL_CIRC;
  if (dir == BACKWARD)
    reverseDist = ((float) leftReverseTicks / COUNTS_PER_REV) * WHEEL_CIRC;

}

  //  if (dir == FORWARD) {
  //    leftForwardTicks++;
  //  //Serial.print("LEFTFORWARD: ");
  //  //Serial.println(leftForwardTicks);
  //    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); //how much its moving forward
  //  }
  //  else if (dir == BACKWARD) {
  //    leftReverseTicks++;
  //    reverseDist = (unsigned long) ((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  //  }
  //  else if (dir == LEFT) {
  //    leftReverseTicksTurns++; //based on Hint given
  //  }
  //  else if (dir == RIGHT) {
  //    leftForwardTicksTurns++;
  //  }


void rightISR()
{

  switch (dir)
  {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break;
  }
}

  //  if (dir == FORWARD) {
  //    rightForwardTicks++;
  //  }
  //  else if (dir == BACKWARD) {
  //    rightReverseTicks++;
  //  }
  //  else if (dir == RIGHT) {
  //    rightReverseTicksTurns++;
  //  }
  //  else if (dir == LEFT) {
  //    rightForwardTicksTurns++;
  //  }


// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{

  // Set the INT0 interrupt to respond on the Falling edge.
  // We need to set bits 0 and 1
  EICRA = 0b00001010;

  // Activate INT0 and INT 1
  EIMSK = 0b00000011;
  // Ensure that interrupts are turned on.
  sei();

  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
}

ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}
// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.


/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */

//      DDRD |= 0b01100000; //PD5 & PD6
//      DDRB |= 0b00000110; //PB2 & PB3
}


// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

//
//// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  digitalWrite(LF, 0);
  digitalWrite(LR, 0);
  digitalWrite(RF, 0);
  digitalWrite(RR, 0);

}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;

}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

int stayON_W_S = 300; // KEY W AND KEY S
int stayON_A_D = 200; // KEY A AND KEY D
//int stayON_T_G = 600; // KEY T AND KEY G
void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:  // FOR W
      //sendOK();
      analogWrite(LF, 255);
      analogWrite(RF, 248);
      digitalWrite(LR, LOW);
      digitalWrite(RR, LOW);
      delay(stayON_W_S);
      stop();
      sendOK();
      putArduinoToIdle(); //extra 
      //forward((float) command->params[0], (float) command->params[1]);
      break;
      
   /*  case COMMAND_FORWARD_2: // FOR T
      analogWrite(LF, 255);
      analogWrite(RF, 248);
      digitalWrite(LR, LOW);
      digitalWrite(RR, LOW);
      delay(stayON_T_G);
      stop();
      sendOK();
      break; */
      
    case COMMAND_REVERSE: // FOR S 
      //sendOK();
      digitalWrite(LF, LOW);
      digitalWrite(RF, LOW);
      analogWrite(LR, 255);
      analogWrite(RR, 252);
      delay(stayON_W_S);
      stop();
      sendOK();
      putArduinoToIdle(); //extra 
      //reverse((float) command->params[0], (float) command->params[1]);
      break;
      
     /* case COMMAND_REVERSE_2: // FOR G
      digitalWrite(LF, LOW);
      digitalWrite(RF, LOW);
      analogWrite(LR, 255);
      analogWrite(RR, 252);
      delay(stayON_T_G);
      stop();
      sendOK();
      break; */
      
    case COMMAND_TURN_LEFT:
      //sendOK();
      digitalWrite(LF, LOW);
      analogWrite(RF, 200);
      analogWrite(LR, 230);
      digitalWrite(RR, LOW);
      //delayMicroseconds(stayON);
      delay(stayON_A_D);
      stop();
      sendOK();
      putArduinoToIdle(); //extra 
      //left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      //sendOK();
      analogWrite(LF, 230);
      digitalWrite(RF, LOW);
      digitalWrite(LR, LOW);
      analogWrite(RR, 230); 
      delay(stayON_A_D);
      stop();
      sendOK();
      putArduinoToIdle(); //extra 
      //right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      //sendOK();
      digitalWrite(LF, LOW);
      digitalWrite(LR, LOW);
      digitalWrite(RF, LOW);
      digitalWrite(RR, LOW);
      sendOK();
      //stop((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;
    /*
       Implement code for other commands here.

    */

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}


//turn off watchdog timer (POWER MANAGEMENT)
void WDT_off()
{
  /* Global interrupt should be turned OFF here if not already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require turning off global interrupt */
  
}

//(POWER MANAGEMENT)
void setupPowerSaving() 
{
  // Turn off the Watchdog Timer
  WDT_off();
  // Modify PRR to shut down TWI
  PRR|=PRR_TWI_MASK;
  // Modify PRR to shut down SPI
  PRR|=PRR_SPI_MASK; 
  // Modify ADCSRA to disable ADC,
  // then modify PRR to shut down ADC
  ADCSRA&= (~ADCSRA_ADC_MASK);   //ADCSRA_ADC_MASK 0b10000000
  PRR|=PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF
  DDRB|=0b00100000;
  PORTB&=0b11011111;
}

//(POWER MANAGEMENT)
void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR|=PRR_TIMER2_MASK;
  PRR|=PRR_TIMER1_MASK;
  PRR|=PRR_TIMER0_MASK;
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR|=SMCR_SLEEP_ENABLE_MASK;
  // The following function puts ATmega328P???s MCU into sleep;
  // it wakes up from sleep when USART serial data arrives
  sleep_cpu();  //included inside "sleep.h"
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR&=(~SMCR_SLEEP_ENABLE_MASK);
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR&= ~PRR_TIMER2_MASK;
  PRR&= ~PRR_TIMER1_MASK;
  PRR&= ~PRR_TIMER0_MASK;
}

void setup() {
  // put your setup code here, to run once:
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);

  cli(); //disable interrupts
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();

  sei();


}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  putArduinoToIdle(); // put your main code here, to run repeatedly:

  TPacket recvPacket; // This holds commands from the Pi

  TResult result  = readPacket(&recvPacket); // this reads the received packet from PI

  if (result == PACKET_OK){ //if packet received properly from Pi to arduino
    handlePacket(&recvPacket);
   // putArduinoToIdle(); //power saving part
  }
  else if (result == PACKET_BAD)
  {
    sendBadPacket(); // if bad packet, arduino will send this response back to PI
    putArduinoToIdle(); //power saving part
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum(); // if bad checksum, arduino will send this response back to PI 
    putArduinoToIdle(); //power saving part
  }
}

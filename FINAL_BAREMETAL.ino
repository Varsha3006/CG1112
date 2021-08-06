#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;
volatile TDirection dir = STOP;

//Timer 0
static volatile int OCR0B_val = 0;  //LR (PD5/PIN 5)
static volatile int OCR0A_val = 0; // LF (PD6/PIN 6)

//Timer 1
static volatile int OCR1BL_val = 0; // RF (PB2/PIN 10)
static volatile int OCR1AL_val = 0; // RR (PB1/PIN 9)


// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
/*#define RR                  11  // Right reverse pin*/
#define RR                  9  // Right reverse pin


/////////////////////////////////////////////////////
//ISR for motors
ISR(TIMER0_COMPA_vect)
{
  OCR0A = OCR0A_val;
}

ISR(TIMER0_COMPB_vect)
{
  OCR0B = OCR0B_val;
}

ISR(TIMER1_COMPA_vect)
{
  OCR1AL = OCR1AL_val;
  OCR1AH = 0; // not in use
}

ISR(TIMER1_COMPB_vect)
{
  OCR1BL = OCR1BL_val;
  OCR1BH = 0; // not in use
}

///////////////////////////////////////////////////////

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
        A1IN - Pin 6, PD6, OC0A LF
        A2IN - Pin 5, PD5, OC0B LR
        B1IN - Pin 10, PB2, OC1BL/H RF
        B2In - Pin 9, PB1, OC1AL/H RR
  */
  cli();

  //Timer 0 for left motor
  DDRD |= 0b01100000;
  TCNT0 = 0;
  TIMSK0 |= 0b110;
  OCR0A = 0;
  OCR0B = 0;
  TCCR0B = 0b00000011; // pre-scalar: 64

  //Timer 1 for right motor
  DDRB |= 0b00000110;
  TCNT1L = 0;
  TCNT1H = 0;
  TIMSK1 |= 0b110;
  OCR1AL = 0;
  OCR1AH = 0;
  OCR1BL = 0;
  OCR1BH = 0;
  TCCR1B = 0b00000011; // pre-scalar: 64  ( phase correct)

  sei();

}

// Stop Alex. To replace with bare-metal code later.
void stop()
{

  OCR0A_val = 0;
  OCR0B_val = 0;
  OCR1AL_val = 0;
  OCR1BL_val = 0;
}

void left_forward() {

  TCCR0A = 0b10000001; //for LEFT MOTOR FORWARD (phase correct PWM: top = 0xFF)
  PORTD &= 0b01000000;
}

void right_forward() {

  TCCR1A = 0b00100001; // for RIGHT MOTOR FORWARD  (phase correct, 8-bit PWM: top = 0xFF)
  PORTB &= 0b00000100;

}

void left_reverse() {

  TCCR0A = 0b00100001; //for LEFT MOTOR REVERSE
  PORTD &= 0b00100000;

}
void right_reverse() {

  TCCR1A = 0b10000001; // for RIGHT MOTOR REVERSE
  PORTB &= 0b00000010;

}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands
    case COMMAND_FORWARD:

      OCR0A_val = 255; // LF (PD6/PIN 6)
      OCR1BL_val = 248; // RF (PB2/PIN 10)

      OCR0B_val = 0;  //LR (PD5/PIN 5)
      OCR1AL_val = 0; // RR (PB1/PIN 9)

      for (int i = 0; i < 10; i++) {

        left_forward();
        right_forward();

        delay(5000);

      }
      stop();
      sendOK();
      break;

    case COMMAND_REVERSE:

      OCR0B_val = 255;  //LR (PD5/PIN 5)
      OCR1AL_val = 252; // RR (PB1/PIN 9)

      OCR0A_val = 0; // LF (PD6/PIN 6)
      OCR1BL_val = 0; // RF (PB2/PIN 10)

      for (int i = 0; i < 10; i++) {

        left_reverse();
        right_reverse();

        delay(5000);
      }
      stop();
      sendOK();
      break;


    case COMMAND_TURN_LEFT:

      OCR0B_val = 230;  //LR (PD5/PIN 5)
      OCR1BL_val = 200; // RF (PB2/PIN 10)

      OCR0A_val = 0; // LF (PD6/PIN 6)
      OCR1AL_val = 0; // RR (PB1/PIN 9)

      for (int i = 0; i < 10; i++) {

        left_reverse();
        right_forward();

        delay(5000);
      }
      stop();
      sendOK();
      break;

    case COMMAND_TURN_RIGHT:

      OCR1AL_val = 230; // RR (PB1/PIN 9)
      OCR0A_val = 230; // LF (PD6/PIN 6)

      OCR1BL_val = 0; // RF (PB2/PIN 10)
      OCR0B_val = 0;  //LR (PD5/PIN 5)

      for (int i = 0; i < 10; i++) {

        right_reverse();
        left_forward();

        delay(5000);

      }
      stop();
      sendOK();
      break;


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

void setup() {
  // put your setup code here, to run once:

  cli(); //disable interrupts

  setupSerial();
  startSerial();
  setupMotors();
  initializeState();

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

  // put your main code here, to run repeatedly:

  TPacket recvPacket; // This holds commands from the Pi

  TResult result  = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
}

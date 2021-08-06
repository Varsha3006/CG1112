#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
//#include <curses.h>
//#include <conio.h> 

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
sem_t _xmitSema;

void handleError(TResult error) //data type TResult. error is the argument
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet) //response from Arduino into PI! 
{
	switch(packet->command) //This has to be command. follow in Arduino code too
	{
		case RESP_OK:
			//printf("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)  //This has to be command. follow in Arduino code too 
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
			
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
			
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
	
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");

		break;

		default:
			printf("Arduino reports a weird error\n");

	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)  // THIS IS THE MAIN! WHEN INCOMING PACKETS COME IN FROM ARDUINO. THIS FUNCTION DECIDES WHERE IT GOES NEXT
{
	switch(packet->packetType) // This has to be packetType. follow in Arduino code too 
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet) // sends the packet(command) from PI to arduino (handle 
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len); 
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

//void getParams(TPacket *commandPacket)
//{
//	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
//	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
//	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
//	flushInput();
//}

void sendCommand(char command) // from PI to Arduino (handleCommand)
{
	TPacket commandPacket; //commandPacket is a variable of data type TPacket 

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'W':
		case 'w':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'S':
		case 's':
			//getParams(&commandPacket); //get the parameters (distance/angle and speed) 
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket); //send the commandPacket which includes Direction / Distance(angle) / Speed to Arduino 
			break;

		case 'A':
		case 'a':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'D':
		case 'd':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

	/*	case 't':
		case 'T':
			commandPacket.command = COMMAND_FORWARD_2;
			sendPacket(&commandPacket);
			break;
			
		case 'g':
		case 'G':
			commandPacket.command = COMMAND_REVERSE_2;
			sendPacket(&commandPacket); //send the commandPacket which includes Direction / Distance(angle) / Speed to Arduino 
			break; */

		default:
			printf("Bad command\n");

	}
}


int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for ONE seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);
	
	int c; 
	
	printf("Command (W = FORWARD, S = REVERSE, D = RIGHT,  A = LEFT, q=exit)\n");
	
	
	system("/bin/stty raw");
	//system("/bin/stty -echo"); // To hide input  
	while((c=getchar()) !='q')
	{		
		if(c == 'w' || c == 'a' || c == 's' || c == 'd'){
			
			sendCommand(c);
			//sleep(1);	
		}
		/*else if (c == 't' || c == 'g'){
			
			sendCommand(c); 
		} */
			
		
	}
	system ("/bin/stty cooked");

	printf("Closing connection to Arduino.\n");
	//endSerial();
}

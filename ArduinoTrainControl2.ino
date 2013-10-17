#include "OutDel.h"
#include <Streaming.h>
#include <Button.h>
#include <PinChangeInt.h>

#include <PString.h>
#include <Timer.h>
#include <Event.h>
#include <Metro.h>
#include "Turnout.h"
#include <arduino.h>
#include <QueueArray.h>

//Vars:
//String buffers:
char turnoutBuffer[50];
char blockBuffer[50];
char tsMsgBfr[24];
char bsMsgBfr[24];
PString turnoutStr(turnoutBuffer,sizeof turnoutBuffer);
PString blockStr(blockBuffer,sizeof blockBuffer);
PString bsMsg = PString(bsMsgBfr, sizeof bsMsgBfr);
PString tsMsg =  PString(tsMsgBfr, sizeof tsMsgBfr);
//Mux Shield Communicator:
OutDel theShield;

//struct for the blocks:
typedef struct Block
{
  int bank2pin;
  int bank3pin;
  int powerLevel;
  bool isCCW;
  bool isReversed;

}
Block;
Block blocks[9];

//message constants:
const char* is90 = "90";
const char* is180 = "180";
const char* tnMsgB = "*T";
const char* ccwPol = "CCW";
const char* cwPol = "CW";
const char* okCmd = "*M{OK}";
const char* errorMsg = "*M{E}";
const char* sensorMSG = "*S{";
//// More External Variables:
Block tempBlock;
int x;
//Arrays for turnouts:
int numOfTurnouts = 6;
//Turnout turnouts[] = {Turnout(1,theShield), Turnout(2,theShield), Turnout(3,theShield), Turnout(4, theShield), Turnout(5, theShield), Turnout(6, theShield)};
//Turnout* turnoutArray = static_cast<Turnout*>(::operator new (sizeof Turnout * 6));
Turnout *turnouts[6];
//Timers: 
Timer tt;

//***************************
//Typedef for serial commands:
typedef struct blockCmd
{
  int blockNum;
  int power;
  bool isCCW;
} 
BlockCmd;
typedef struct turnoutCmd
{
  int turnoutNum;
  bool is90;
} 
TurnoutCmd;
//Serial communication array:********************QUEUES********************
//Queue of block commands comming in from the serial port.
QueueArray<BlockCmd> blockCommands;
//Queue of turnout commands comming in from the serial port.
QueueArray<TurnoutCmd> turnoutCommands;
//queue of strings waiting to be pushed out out to the serial out port.
QueueArray<String> outStrings;

//QueueArray<char> blockSerialChars;
//QueueArray<char> turnoutSerialChars;
//*******************SERIAL INTERFACE EXTERNS ***********************
char blockSerChar[8]; //chars stored here from serial port.
char turnSerChar[8]; //chars stored here from serial port.
bool blockCMDReady = false; //if there is a block cmd that has been parsed.
boolean turnoutCMDReady = false; //if there is a turnout cmd that has been parsed.
//These are the command variables - placed here so that we don't have to pass around the numbers.
//however, this means that we can only parse ONE block command per loop!!!!
int blkNumT;
int blkPowerT;
bool isCCWT;
//Same as above, but for turnouts:s
int turnoutNumT;
boolean is90T;

//Serial Data Flags **
boolean ser1Rdy = false;
boolean ser2Rdy = false;
char cmdEndChar = '|'; //this char is the end of a data stream.
boolean cmdNumsRdy = false;
//Represent the serial port data:

int num1 = -999; //invalid!
int num2,num3; //unknown!!


///*******************************************************
//****************************SENSOR READING ARRAY ****************************/
int sensorData[16];
QueueArray<uint8_t> sensorDataQ;
// this array has the pin assignments:
const uint8_t sensorPins[] = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45};
const uint8_t sensorEquiv[] ={
  0,0,0,0,0,0,0,0,0,0,00,00,00,00,00,00,00,00,00,00,00,00, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,00,00,00,00,00,00,00,00};
const uint8_t activeSensorPins[] = {
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37};
const uint8_t shSensorEquiv[] = {
  1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
bool sensorHits[16] = {
  false};
int sensorQTY = 16;
uint8_t sensorHit[16] = {
  0};
uint8_t prevSensorVal[16] = {
  0};

//sensor message:
///Pin Change Variables 
uint8_t latest_interrupted_pin;
uint8_t interrupt_count[64]={
  0};
Button *sensorBtns[16];
//***************SENSOR DATA DONE ***************************************8/
//ready pin:
const uint8_t readyPin = 14;
//refresh mux shield timer. If this dosent work, we're going to ditch the mux shield!!!
Metro refreshOK(1000);

/// Possibly nneeded code from this point until next star comment ************************************************************8
//char cleanup from serial -- make sure the command is okay.
// { B , B , B } \n == 8 Bytes.
// 0 1 2 3 4 5 6 7
//  num,isCCW,PWR -- 1,3,5
bool checkBlockCMD()
{

  bool isValid = false;
  blkNumT = blockSerChar[1] - 0;
  blkPowerT = blockSerChar[5] - 0;
  int isCCW = blockSerChar[3] - 0;
  if(blockSerChar[0] == '{' && blockSerChar[6] == '}')
  {
    //check to see if the block is within the range we want:
    if (blkNumT > 0 && blkNumT < 10 && blkPowerT < 7 && blkPowerT > -1)
    {
      if (isCCW == 0)
      {
        isCCWT = false;
        isValid = true;
      }
      else if(isCCW == 1)
      {
        isCCWT = true;
        isValid = true;
      }
    }

  }
  return isValid;
}
//TurnoutCMD verification & var placement.
bool checkTurnCMD()
{
  bool isValid = false;
  turnoutNumT = turnSerChar[1] - 0;
  if(turnSerChar[0] == '{' && turnSerChar[5] == '}')
  {
    if (turnoutNumT > 0 && turnoutNumT < 7) //valid mui,bers 
      if(turnSerChar[4] - 0 == 1)
      {
        is90T = true;
        isValid = true;
      }
      else if(turnSerChar[4] - 0 == 0)
      {
        is90T = false;
        isValid = false;
      }
  }
  return isValid;
}
void queueErrorMessage()
{
  outStrings.push(errorMsg);
}
void queueBlockCmd()
{
  BlockCmd cmd;
  cmd.blockNum =blkNumT;
  cmd.isCCW = isCCWT;
  cmd.power = blkPowerT;
  blockCommands.push(cmd);
  blockCMDReady = false;
}
void queueTurnoutCmd()
{
  TurnoutCmd cmd;
  cmd.is90 = is90T;
  cmd.turnoutNum = turnoutNumT;
  turnoutCommands.push(cmd);
  turnoutCMDReady = false;
}
/*
//Serial 1 is the block command port.
 //Serial 2 is the turnout command port.
 //Serial 3 is the outgoing message port.
 //to simplify the arduino's processing, the netduino will send block messages in the following format:
 //{Byte,Bool,Byte}\n
 
 //for:
 //block number, isCCW, PowerLevel.
 //this allows for easier checking.
 //void serialEvent1()
 //{
 //	//blocking serial port event.
 //	Serial.println("Got Serial 1 Data in!");
 //	if(Serial1.available() >= 8 && !blockCMDReady)
 //	{
 //		Serial.println("Serial1 - 8 Bytes Recd. & no command waiting.");
 //		//fill up our char buffer:
 //		Serial1.readBytesUntil('\n',blockSerChar,8);
 //		if (checkBlockCMD())
 //		{
 //			blockCMDReady = true;
 //		}
 //		else
 //		{
 //			queueErrorMessage();
 //		}
 //	}
 //}
 
 ////serial event 2 - Turnout Commands:
 ////turnout cmds are in the form of:
 ////{Byte,Byte}\n
 //void serialEvent2()
 //{
 //	Serial.println("Got Serial 2 Data In!");
 //	if(Serial2.available() >= 6 && !turnoutCMDReady)
 //	{
 //		Serial.println("Serial2 - 6 bytes RCD - No Command Waiting.");
 //		Serial2.readBytesUntil('\n',turnSerChar,6);
 //		if(checkTurnCMD())
 //		{
 //			turnoutCMDReady = true;
 //		}
 //		else
 //		{
 //			queueErrorMessage();
 //		}
 //	}
 //}
 //*********************************************************************************New code:****************************************
 //This is debug code to test the system out on a PC:*/
char demoBuffer1[8];
char demoBuffer2[6];

//Serial Port 0 is a debug serial port, shared with the USB. We do not use it normally, so it has a special command system:
void serialEvent()
{
	if (Serial.available() > 5)
	{
    //ser1Rdy = true;
	    Serial.println("Serial Port 0 - Debug Input Detected.");
	    num1 = Serial.parseInt();
	    num2 = Serial.parseInt();
	    num3 = Serial.parseInt();
	    cmdNumsRdy = true;
	    
	    //clear read buffer?
	}
}
//when the events are thrown, check to see if there is enough data to warrant reading an input.
void serialEvent2() // should be input from the netduino.
{
//  if (Serial2.available() > 5)
//    ser2Rdy = true;
	if (Serial2.available() > 5)
	{
    //ser1Rdy = true;
	    Serial.println("Serial Port 2 - Non-Debug Input Detected.");
	    num1 = Serial2.parseInt();
	    num2 = Serial2.parseInt();
	    num3 = Serial2.parseInt();
		Serial.print("Port 2 commands are: ");
		Serial.print(num1);
		Serial.print(num2);
		Serial.print(num3);
	    
	    	cmdNumsRdy = true;
			Serial.println("Cmd is ready!");
	    
	    //clear read buffer?
	}
}

//parser method:
void parseSerialCommand(int num1, int num2, int num3)
{
	if(cmdNumsRdy)
		{
		Serial.println("Num 1:");
		Serial.println(num1);
		//anyway, we parse the message out and turn it into a command:
		if (num1 == 0 && (num2 > 0 && num2 < 7)){ // turnout message (and verification);
		TurnoutCmd tc;
		bool isn = num3; //QAD -> 0 is false.
		tc.turnoutNum = num2;
		tc.is90 = isn;
		turnoutCommands.push(tc);
		Serial.println("Got Trnout Cmd");


		}
		else if(num1 > 0 && num1 < 10) // QAD validation
		{
		BlockCmd bc;
		bc.blockNum = num1;
		bc.isCCW = num2;
		bc.power = num3;
		blockCommands.push(bc);
		Serial.println("Got Block Cmd");
		}
		else if(num1 > 50) //debug message;
		{
		String message2 = "Turnout Queue Size: ";
		message2 += turnoutCommands.count();
		message2 += "\n Block Queue Size: ";
		message2 += turnoutCommands.count();
		Serial.println(message2);

		}
		else if(num1 == 42)
		{
		theShield.digitalWriteOD(num2,num3,LOW);
		}
		else if(num1 == 43)
		{
		theShield.digitalWriteOD(num2,num3,HIGH);
		}


		//finally, fix the flag:
		cmdNumsRdy = false;
	}
}

//this uses the readBytesUntil method to manage serial input data. ReadBytesUntil was used to
//ensure that the commands are not read in a run-together fasion.
char* readSerial(int serNum)
{
  char serialReader[5];
  int read = 0;
  if (serNum == 2){
    if(Serial2.available() > 5)
    {
      ///Final def: Int,Int,Int,| 
      //|is the break character
      read = Serial2.readBytesUntil(cmdEndChar, serialReader,6);
      //mark the serial port done, if out of datA:
      if(Serial2.available() < 6)
      {
        ser2Rdy = false;
      }

    }
  }
  else
  {
    if (Serial.available() > 5)
    {
      read = Serial.readBytesUntil(cmdEndChar, serialReader,6);
      if(Serial.available() < 6)
      {
        ser1Rdy = false;
      }
    }
  }
  return serialReader;  
}

int* getNumberCmds(char* stream)
{
  int nums[3];
  char f1 = stream[0];
  char f2 = stream[2];
  char f3 = stream[4];

  nums[0] = atoi(&f1);
  nums[1] = atoi(&f2);
  nums[3] = atoi(&f3);
  return nums;
}

//perhaps we don't need this after all.
int* readSerialNumeric(int serNum)
{
  	int serialReader[3];
  	int read = 0;
 	if (serNum == 2){
	    if(Serial2.available() > 5)
	    {
	      ///Final def: Int,Int,Int,| 
	      //|is the break character
	      //read = Serial2.readBytesUntil(cmdEndChar, serialReader,6);
	      read = Serial2.parseInt();
	      serialReader[0] = read;
	      read = Serial2.parseInt();
	      serialReader[1] = read;
	      read = Serial2.parseInt();
	      serialReader[2] = read;
	      //mark the serial port done, if out of datA:
	    	
	    	
				ser2Rdy = false;
	   	

		}
  	}
  
    if (Serial.available() > 5)
    {
      //read = Serial.readBytesUntil(cmdEndChar, serialReader,6);
      read = Serial.parseInt();
      serialReader[0] = read;
      read = Serial.parseInt();
      serialReader[1] = read;
      read = Serial.parseInt();
      serialReader[2] = read;

      Serial.println("This is the numeric reader method."); 
	   
	   
	    	ser1Rdy = false;
	}
  
  return serialReader;  
}

void readSerialCommand()
{
  //Reads the serial port, then parses the data!
  //new format serial command: Int,int,int.
  // first int determines block number, or if zero, is a turnout command.
  //second int is a 0-7 for power, or is the switch number.
  //third int is 0 or 1 for CCW or CW or 90 or 180.
  //for debug purporses, we check this for serial and  serial 2!!! 
}/* ////old code --------------------------------------------------------for oldReadSerialCommand()
  num1 = -999;
  num2 = 0;
  num3 = 0; //set these up to invalid numbers to catch garbage.
  int serNum = -1; // no serial port by default.
  if(ser1Rdy)
    serNum = 0; // port zero is "interactive" debug through the arduino serial port monitor.
  else if (ser2Rdy)
  {
    serNum = 2; //port two is the primary port.
    Serial.print("Serial 2 Reading.");
}
  int* parsedNums;
  //set up the serial numbers!!
  //what serial ports do we use?
  if (serNum != -1) // serNum is not invalid, so we are ok.
  {
  	Serial.print("Serial reading now, from port ");
  	Serial.println(serNum);
  	//TODO: Investigate CRASH HERE!!!!
  	//TODO: Possibly do not init the relays to known states here,
  	//TODO: do it on the netduino!!!
    //parsedNums = getNumberCmds(readSerial(serNum));
    parsedNums = readSerialNumeric(serNum);
    Serial.print("number array is: ");
    Serial.print(parsedNums[1]);
    Serial.print(parsedNums[2]);
    Serial.println(parsedNums[3]);
  }
  //DEBUG PRINTOUT CODE:
  char debugMessage[200];

  // the fact that this is in here twice is because of debugging!! == we can delete this once serial port 0 testing is complete

  num1 = parsedNums[0];
  num2 = parsedNums[1];
  num3 = parsedNums[3];
    PString debug(debugMessage, sizeof debugMessage);
  debug.print("Part One: ");
  debug.print(num1);
  debug.print("\n\t Part 2: ");
  debug.print(num2);
  debug.print("\n\t Part 3: ");
  debug.print(num3);
  Serial.println(debug);

}*/


void setBlockPower(int blockNum, int powerLevel)
{
  blockNum--;
  blocks[blockNum].powerLevel = powerLevel;
}
void setBlockPolarity(int blockNum, bool isCCW)
{
  blockNum --;
  blocks[blockNum].isCCW = isCCW;
}


void updateBlocks(int i)
{
  Serial.println("UpdateBlocks Called.");
  //for(int i = 0; i < 9; i ++)
  //{
  tempBlock = blocks[i];
  int bank2PinValue = HIGH;
  int bank3PinValue = HIGH;

  if(tempBlock.powerLevel == 0 ) //block is off, make sure the shield reflects this.
  {
    //theShield.digitalWriteOD(2,tempBlock.bank2pin,HIGH);
    //theShield.digitalWriteOD(3,tempBlock.bank3pin,HIGH);
  }
  else if(tempBlock.isCCW)
  {
    bank2PinValue = HIGH;
    bank3PinValue = LOW;
  }
  else 
  {
    bank2PinValue = LOW;
    bank3PinValue = HIGH;
  }

  if (tempBlock.isReversed)
  {
    x = bank3PinValue;
    bank3PinValue = bank2PinValue;
    bank2PinValue = x;
  }

  Serial << "Output to matrix from block " << i <<" is bank 2, pin " << tempBlock.bank2pin << ", value: " << bank2PinValue << "\n";
  Serial << "Bank 3, pin "<< tempBlock.bank3pin <<", Value: "<< bank3PinValue << "\n";

  //theShield.digitalWriteOD(2,tempBlock.bank2pin,bank2PinValue);
  writeShield(2,tempBlock.bank2pin,bank2PinValue); //50/50 refresh?
  theShield.digitalWriteOD(3,tempBlock.bank3pin,bank3PinValue);

  Serial.println("Matrix update complete. Block Update Complete.");

  //}
}
void doBlockCmds()
{
  while(blockCommands.count() > 0)
  {
    BlockCmd cmd = blockCommands.pop();
    //we have a command, let's update the state:
    Serial << "Block Command: " << cmd.blockNum << ", CCW: " << cmd.isCCW;
    Serial.println();
    cmd.blockNum --; // sub by one
    blocks[cmd.blockNum].powerLevel = cmd.power;
    blocks[cmd.blockNum].isCCW = cmd.isCCW;
    updateBlocks(cmd.blockNum); // one at at time!!!!!

  }
  //we are going to do some crazy thing here:
  //for (int i = 0; i < 8; i ++)
  //{ //if this dosent work, throw the updateblocks into the main loop with an outside counter.
  //		updateBlocks(i);
  //}
}


//void turnoutDemo()
//{
//	turnout = turnout % 6;
//	if (turnout == 0)
//		is180 = !is180;
//	turnouts[turnout]->setTurnout(is180,theShield);
//	Serial.print("Turnout ");
//	Serial.print(turnout);
//	Serial.println(" was changed by the demo program.");
//        turnout ++;
//
//}

void doTurnoutCmds()
{
  if (turnoutCommands.count() > 0)  //debating between an if and a while here.
  {
    TurnoutCmd tc = turnoutCommands.pop();
    turnouts[tc.turnoutNum]->setTurnout(tc.is90,theShield);
  }
}

//update the turnouts:
void updateTurnouts()
{
}
void checkTurnout()
{
  for (int i = 0; i < numOfTurnouts; i ++)
  {
    turnouts[i]->Tick(theShield);
  }
}

//turnout state reporting -- debug function.
void turnoutStates()
{
  turnoutStr.begin();
  for(int i =0; i < numOfTurnouts; i ++)
  {
    turnoutStr.format("Turnout %i is currently $b 90.", i, turnouts[i]->is90);
    Serial.println(turnoutStr);
  }
}

//the setTurnStateBuffer function, and th e
void setTurnStateBuffer(int turnoutNum)
{
  //turnout state request.
  //first, reduce turnout number by one:
  turnoutNum --;
  //clear buffer:
  tsMsg.begin();
  if(turnouts[turnoutNum]->is90)
  {
    tsMsg.format("%c%i{%c}",tnMsgB,turnoutNum,is90);
  }
  else
  {
    tsMsg.format("%c%i{%c}",tnMsgB,turnoutNum,is180);
  }
  //return tsMsg;
}

void setBlockStateBuffer(int blockNum)
{
  //sub by one block num
  blockNum--;
  bsMsg.begin();
  if(blocks[blockNum].isCCW)
  {
    bsMsg.format("*B%i{%c,%i}",blockNum,ccwPol,blocks[blockNum].powerLevel);
  }
  else
  {
    bsMsg.format("*B%i{%c,%i}",blockNum,cwPol,blocks[blockNum].powerLevel);
  }

}
int sens1 = -1;
int sens2 = -1;
int cSens = 0;
void readSensCaller()
{
  bool sensHit = false;
  sens1 = -1;
  sens2 = -1;
  String shm = "*S{";
  if(sensorBtns[cSens]->getPressCount())
  {
    sens1 = cSens;
    cSens ++;
  }
  if(sensorBtns[cSens]->getPressCount())
  {
    sens2 = cSens;
    cSens ++;
  }
  if(sens1 != -1)
  {
    if (sens2 != -1)
    {
      //two sensors hit at the same time.
      shm += sens2;
      shm += ",";
      sensorBtns[cSens]->clearPressCount();
    }
    sensorBtns[cSens - 1]->clearPressCount();
    shm += sens1;
    shm += "}\n";

  }
  if(sensHit)
  {
    outStrings.push(shm);
  }
  cSens = cSens % 16;
}
//polls the sensors, and if they have turned low, queues a message to send to the receiver. NOTE: This currently does not support *S{X,X} format, only one sensor at a time will be reported.
void readSensors()
{
  bool detectedSensor = false;
  int sensorHits[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  };
  int hitCount = 0;
  String shm = "*S{";
  //I think 2 loops works. This is instead of using interrupts, since I don't have enough hardware interrupts.
  for(int i = 0; i < 16; i ++)
  {

    //I am using the "Button" library here,. This library helps automatically debounce inputs. Treating the sensors like
    //buttons makes for easier to read code. The code gets 
    //if (sensorBtns[i]->onPress())
    if(sensorBtns[i]->getPressCount()) // 
    {
      //Serial.println(shm);
      hitCount ++;
      sensorHits[i] = 1;
      sensorBtns[i]->clearPressCount();

    }
  }
  if(hitCount > 0) //run the assembly loop if one or more sensors were hit.
  {

    for(int i = 0; i < 16; i ++)
    {
      if(sensorHits[i]) 
      {
        hitCount --;
        shm += i;
        if (hitCount > 0)
        {
          shm += ",";
        }
        else //HitCount is zero, so no more sensor readings.
        {
          shm += "}\n";
          outStrings.push(shm);
          break;
        }
      }		
    }

  }



}

//for(int i = 0; i < sensorQTY; i ++) //poll all of the sensor pins.
//{
//	uint8_t senVal = digitalRead(activeSensorPins[i]);
//	//is this a new read?
//	//removed bitwise operations for pure logic operations

//	if (senVal != sensorHit[i] )
//	{
//		// new value!!
//		//prevSensorVal[i] = sensorHit[i];
//		if (sensorHit[i] == LOW) // high? low? 
//		{//build a message - since the sensor has been hit.
//			String message = "*S{";
//			message += shSensorEquiv[i];
//			message += "}";
//			outStrings.push(message);
//		}
//	}
//}

//for (int i = 0; i < sensorQTY; i ++)
//{
//	String sensor = sensorMSG;
//	//hope that these correspond somewhat to the real thing
//	//sensorData[i] = digitalRead(sensorPins[i]);
//	if (sensorData[i] > 0) //while a sensor has been hit:
//	{
//		sensor += sensorEquiv[i];
//		sensor += "}";
//	}
//	
//}
////Queue version:
//if(sensorDataQ.count() > 0)
//{
//String sensorMessage = sensorMSG; // start the message with "*S{
//while (sensorDataQ.count() > 0)
//{
//	sensorMessage += sensorDataQ.pop();
//	if (sensorDataQ.count() > 0)
//	{
//		sensorMessage += ",";
//	}
//}
//sensorMessage += "}";
//outStrings.push(sensorMessage);
//Serial.print("Sensor hit. Message will be:");
//Serial.println(sensorMessage);
//}



void sensorHitFunction()
{
  /*latest_interrupted_pin=PCintPort::arduinoPin;
   	sensorData[sensorEquiv[latest_interrupted_pin]] ++;
   	interrupt_count[latest_interrupted_pin]++;
   	sensorDataQ.push(latest_interrupted_pin);*/
  Serial.print("Sensor Hit: ");
  Serial.print(PCintPort::arduinoPin);
  Serial.println(" - DEBUG - Sensor Hit");


}

void printSerial()
{
  if (outStrings.count() > 0)
  {
    //Serial3.println(outStrings.pop());	
    //TODO: Remove this debug code:
    String msg = outStrings.pop();
    Serial.print(msg);
    Serial2.print(msg);
    //Serial3.print(msg);
  }
}
//shotgun debug!!!
void writeShield(int bank, int pin, int pwr)
{
  //refresh the shield.
  //theShield = OutDel();
  theShield.digitalWriteOD(bank,pin,pwr);
  //maybe this will help?
}

//this setup code starts the arduino up. It inits the serial ports and sets the state of the train to a known state.
void setup()
{
  pinMode(readyPin, OUTPUT); // ready the output pin!
  digitalWrite(readyPin, LOW);

  //refreshOK.interval(1000); //every 1000 seconds?
  //todo: resize queues for initial size:
  //blockSerialChars.resize(SIZE OF MESSAGES);
  //theShield.setMode(1,DIGITAL_OUT);  
  //theShield.setMode(2,DIGITAL_OUT);
  //theShield.setMode(3,DIGITAL_OUT);
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(57600,SERIAL_8O1);
  //Serial3.begin(9600); // NO SERIAL 3!

  Serial.println("Output Pins - setup");
  Serial.println("Removed for testing. No Sensor Pins Active!");
  for(int i = 0; i < 16; i ++)
  {
    sensorBtns[i] = new Button(activeSensorPins[i],false);
    /*pinMode(activeSensorPins[i],INPUT);
     		digitalWrite(activeSensorPins[i],HIGH);
     		Serial.println("Done with sensor intput. NOTE: Added a pullup resistor to the pins - check up or down on final product.");
     		Serial.println("PinChangeInt - Attaching interrupt to update method");*/

    //PCintPort::attachInterrupt(activeSensorPins[i],&sensorHitFunction,CHANGE);
  }

  Serial.println("init sensor arrays to low.");
  for(int i = 0; i < 16; i ++)
  {
    sensorData[i] = LOW;
  }
  Serial.println("Hi there - serial should be done. Now init the shield.");
  //init turnouts
  Serial.println("init turnouts first");
  for(int i = 0; i < 6; i ++)
  {
    turnouts[i] = new Turnout(i + 1, theShield);
  }
  //init shield
  Serial.println("Shield init");
  for(int i = 0; i < 16; i ++)
  {
    theShield.digitalWriteOD(1, i, HIGH);
    theShield.digitalWriteOD(2, i, HIGH);
    theShield.digitalWriteOD(3, i, HIGH);
    delay(100);
  }
  Serial.println("Block init");
  for(int i = 0; i < 9; i ++)
  {
    blocks[i].bank2pin = i;
    blocks[i].bank3pin = i; //the bank 2 and bank 3 pins are currently the same.
    blocks[i].isCCW = false; //default everyone is CW.
    switch(i)
    {
    case 1:
    case 2:
    case 3:
    case 4:
      blocks[i].isReversed = true;
      break;
    default:
      blocks[i].isReversed = false;
    }
    Serial.println("blocks are configured.");


  }

  //Timer for turnout test
  //	tt.every(3000,turnoutDemo);
  Serial.println("Shield done. Timer Done");
  //testing the mux shield:

  //
  //for(int i = 0; i < 16; i ++)
  //{
  // theShield.digitalWriteOD(2,i,HIGH);
  // theShield.digitalWriteOD(3,i,HIGH);
  //}
  //theShield =  OutDel();

  Serial.println("Setup Complete -- delay for debug");
  delay(1000);

  //set the ready pin to HIGH to signal the netduino.
  digitalWrite(readyPin, HIGH);

}
bool firstRun = true;

void loop()
{
  if(firstRun)
  {
    Serial.println("first loop run!!");
    firstRun = false;
  }
  tt.update(); // update timer
  //Serial.println("TT Update Done");
  checkTurnout(); // tick the turnouts so that they don't burn.
  //Serial.println("Turnout Check Done");
  for(int i = 0; i < 16; i ++)
  {
    sensorBtns[i]->listen();
  }
  if (blockCMDReady)
  {
    //we have a command waiting for the blocks:
    queueBlockCmd();
    Serial.println("queueBlockCmd done");
  }
  if (turnoutCMDReady)
  {
    queueTurnoutCmd();
    Serial.println("turnoutCMD ready");
  }
  //parse any waiting commands from the blocks:
  doBlockCmds();
  //Serial.println("done block commands");

  //parse the turnouts:
  doTurnoutCmds();

  //update the turnouts:
  updateTurnouts();
  //Serial.println("updateBlocks done.");
  //read any signal data from sensors
  readSensors();
  //Serial.println("Sensors read");
  //read Serial data -- how about using events?
  //todo: move this to main.
  if(ser1Rdy || ser2Rdy)
  {
    readSerialCommand();
  }
  if (cmdNumsRdy){
  parseSerialCommand( num1,  num2,  num3);
	}
  //update the state
  //send out serial data
  printSerial();
  //Serial.println("Serial printed");
}


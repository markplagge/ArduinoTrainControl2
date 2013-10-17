// 
// 
// 

#include "Turnout.h"

void Turnout::init()
{
	

}

Turnout::Turnout(int turnoutNum, OutDel initShield, unsigned long maxBrn, unsigned long burnStartTime, bool on, bool is9)
{
  
	 static int swNums[] = {14,13,12,11,10,9};
	 turnoutNum --;
	 //that array is in switch number -1, so switch 1 is swnums[0]. Pins are the same number across blocks.
	 this->bank2Pin = swNums[turnoutNum];
	 this->bank3Pin = swNums[turnoutNum];
	 //default max burn is 500 MS.
	 maxBurn = maxBrn;
	 isOn = on;
	 is90 = is9;
	 this->burnStartTime = burnStartTime;
	 this->processID = 0;    
	 //initShield.digitalWriteOD(2,bank2Pin,HIGH);
	// initShield.digitalWriteOD(3,bank3Pin,HIGH); //init the shield OFF! (just to make sure.)
	 this->setTurnout(true, initShield);
	 
	 init();
}
//Timer functions for the relays - so that we don't melt any of the turnout switche.
// first, inline function for abstraction. We're using millis here, but maybe we will need a 
// hardware switch of some sort.
static inline unsigned long elapsed() {return millis();}

void Turnout::Tick(OutDel theShield)
{
  if(isOn) //only if we are on!
  {	
  currentTime = elapsed();
	 //Serial.print("Current Time from Tick:");
	 //Serial.println(currentTime);
	 //Serial.print("burnStartTime is:");
	 //Serial.println(burnStartTime);
	 //Serial.print("MaxBurn is:");
	 //Serial.println(maxBurn);
	 {
		 if(currentTime - burnStartTime >= maxBurn)
		 {

			 Serial.println("Relay Done - Timer hit. Shutting down the relay.");
			 theShield.digitalWriteOD(2,bank2Pin,HIGH);
			 delay(100);
			 theShield.digitalWriteOD(3,bank3Pin,HIGH);
			 delay(100);
			 isOn = false;
		 }
	 }
  }

  
}
bool Turnout::startSwitching()
{
	if(isOn == false)
	{
		
		return true;
	}
	return false;
}

bool Turnout::setTurnout(bool is9, OutDel theShield)
{
	bool completedChange = false;
		//set up the turnout!!
		//TODO: This needs to be tested - we are including a redundant change check.
	Serial.print("setTurnout called. current status: ");
	Serial.print("Is90: ");
	Serial.print(this->is90); 
	Serial.print("IsOn: ");
	Serial.print(this->isOn);
	Serial.print("bank2Pin: ");
	Serial.print(this->bank2Pin);
	Serial.print("bank3Pin: ");
	Serial.print(this->bank3Pin);
	Serial.println();
		//if (this->is90 != is9 && this->isOn == false) // only if we are changing direction though, and not currently switching!
	if(!isOn) //make sure we are off before switch.
		{
			Serial.println("setTurnout changing turnout!!");
			this->is90 = is9;
			//then call the start of the process:
			if (startSwitching())
			{
				if(is90)
				{
					theShield.digitalWriteOD(2,bank2Pin, LOW);
				delay(100);
				}
				else
				{
					theShield.digitalWriteOD(3,bank3Pin, LOW);
				delay(100);
				}
				
				completedChange = true;
				isOn = true;
				burnStartTime = millis();
				
			}
		}
	return completedChange;
}



Turnout TURNOUT;


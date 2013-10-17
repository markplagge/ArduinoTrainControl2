// Turnout.h

#ifndef _TURNOUT_h
#define _TURNOUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "OutDel.h"
class Turnout
{
 private:
	 int turnoutNumber;
	 int bank2Pin;
	 int bank3Pin;
	 unsigned long maxBurn;
	 unsigned long currentTime;
	 unsigned long burnStartTime;
	 bool isOn;

	 int processID;

 public:
	 	 bool is90;
	void init();
	Turnout(){};
	Turnout(int turnoutNum, OutDel initShield, unsigned long maxBrn = 800, unsigned long burnStartTime = 0, bool on = false, bool is9 = false);
	//bool setTurnout(bool is9);
	bool setTurnout(bool is9, OutDel theShield);
	bool startSwitching();

	void Tick(OutDel theShield);
};

extern Turnout TURNOUT;

#endif


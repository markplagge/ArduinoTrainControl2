// OutDel.h

#ifndef _OUTDEL_h
#define _OUTDEL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class OutDel
{
 private:


 public:
	 OutDel();
	void init();
	void digitalWriteOD(int mux, int chan, int val);
	void digitalWriteOD(int mux, int chan, uint8_t val);
};

extern OutDel OUTDEL;

#endif


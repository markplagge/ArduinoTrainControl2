// 
// 
// 

#include "OutDel.h"
int _shiftReg1[16]={0};
int _shiftReg2[16]={0};
int _shiftReg3[16]={0};
uint8_t outputBank1[16]={6,7,8,9,10,11,12,13,A8,A9,A10,A11,A12,A13,A14,A15};
uint8_t sOutB2[16] = {22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52};
uint8_t sOutB3[16] = {23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53};
 OutDel::OutDel()
{
	 for(int i = 0; i < 16; i ++)
	   {
		   pinMode(sOutB2[i], OUTPUT);
		   pinMode(sOutB3[i], OUTPUT);
		   //yeeeeaaaah!!!! 
	 }

}
void OutDel::init()
{
	for(int i = 0; i < 16; i ++)
	{
		digitalWrite(sOutB2[i],HIGH);
		digitalWrite(sOutB3[i],HIGH);
	}
}
 void OutDel::digitalWriteOD(int mux, int chan, int val)
	{
		//here we go:
		//i'll go ahead and set bank 2 to 2, and 3 to 3 here for the purposes of debug.
		if(mux == 2)
		{
			digitalWrite(sOutB2[chan],val);
		}
		else if(mux == 3)
		{
			digitalWrite(sOutB3[chan],val);
		}

	}
 

OutDel OUTDEL;


//spring.h
//contains all data in a struct for spring.c

#ifndef SPRING__H
#define SPRING__H

struct Spring {
	//for allpassLowS
	int NUM_APF_LOW, fCChirp, K1, dlyAPF;
	float K, d, a1, a2, *z1, *z2, *zK1, *v;

	//for allpassHighS
	int NUM_APF_HIGH;
	float ahigh, *zhigh, *outHighAPF;

	//for HdcLowS
	float Rdc, prevOutVal, prevInVal;

	//for HeqLowS
	int Keq, B, fpeak;
	float R, poleAngle, aEQ1, aEQ2, A0, delay1, delay2;

	//for HlpLowS
	int fC;
	float aLP, prevValLP;

	//for delayLineLowS
	int L0, Lripple, Lecho, L, dlyL0Index, dlyLechoIndex, dlyLrippleIndex;
	float Nripple, echoTime, gecho, gripple, *delayL0, *delayLecho, *delayLripple;
		//for noise/LPF
	int gmod;
	float aLPF, LPFPrevVal;

	//for delayLineHighS
	int LHigh, dlyLHighIndex;
	float delayTimeHigh, *delayLHigh;
		//for noise/LPF
	int gmodHigh;
	float aLPFHigh, LPFPrevValH;

	//structural states
	float c1, c2, gDry, gHigh, gLow, gHf, gLf, cHOut, cLOut, APFLout, APFHIn, HeqOut, HdcOut, MTDLout, DLHout;
};

float allpassLowS(float input, struct Spring *S);
float allpassHighS(float input, struct Spring *S);
float MTDLLowS(float input, struct Spring *S);
float delayLineHighS(float input, struct Spring *S);
float HdcLowS(float input, struct Spring *S);
float HeqLowS(float input, struct Spring *S);
float HlpLowS(float input, struct Spring *S);

int initSpring(struct Spring *Spr);
int deleteSpring(struct Spring *Spr);

#endif

//spring.c

#include "spring.h"
#include "globals.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
	int i = 1, inLength;
	float *inArray, *outArray;

	inLength = 88200;

	inArray = (float *) calloc(inLength, sizeof(float));
	outArray = (float *) calloc(inLength, sizeof(float));
	inArray[0] = 1.;


	struct Spring S;
	initSpring(&S);
	FILE *f = fopen("impulse.txt","wt");
	
	printf("\n");
	//printf("NUM_APF_LOW: %i\n",S.NUM_APF_LOW);
	//printf("K: %f\n",S.K);
	//printf("K1: %i\n",S.K1);
	//printf("PI*B: %f\n",M_PI*S.B);


	//for(i = 0; i < 10; i++) {
	//	printf("inArray[%i] = %f\n",i,inArray[i]);
	//}

	for(i = 0; i < inLength; i++) {
		outArray[i] = allpassLowS(inArray[i], &S);
		//printf("outArray[%i] = %f\n",i,outArray[i]);
		fprintf(f,"%f",outArray[i]);
		fprintf(f,"\n");
		//printf("%f\n",k);
	}

	if(deleteSpring(&S)) {
		printf("Spring deleted.\n");
	}
	fclose(f);
	free(inArray);
	free(outArray);
	return 0;
}

float HdcLowS(float input, struct Spring *S) {
	float output;
	output = input - S->prevInVal + S->Rdc * S->prevOutVal;
	//Out = in - zin + Rdc*zout
	S->prevInVal = input; //redefine filter states
	S->prevOutVal = output;
	return output;
}

float HeqLowS(float input, struct Spring *S) {
	float output;
	output = S->A0 * input + S->delay1;
	S->delay1 = S->delay2 - S->aEQ1*output;
	S->delay2 = -S->aEQ2 * output - S->A0 * input;
	return output;
}

float HlpLowS(float input, struct Spring *S) {
	float output;
	output = S->aLP * input + (1-S->aLP) * S->prevValLP;
	S->prevValLP = output;
	return output;
}

float allpassLowS(float input, struct Spring *S) {
	float output;
	int n = 0;
	if(S->dlyAPF >= (S->K1 + 1)) {
		S->dlyAPF = 0;
	}

	S->v[n] = S->z2[n] + S->z1[n] * S->a2;
	S->z2[n] = S->z1[n];
	S->z1[n] = S->zK1[(n * (S->K1 + 1)) + S->dlyAPF] - S->z1[n] * S->a2; //look here
	S->zK1[(n * (S->K1 + 1)) + S->dlyAPF] = input - S->a1 * S->v[n]; //look here

	for(n = 1; n < S->NUM_APF_LOW; n++) {
		S->v[n] = S->z2[n] + S->z1[n] * S->a2;
		S->z2[n] = S->z1[n];
		S->z1[n] = S->zK1[(n * (S->K1 + 1)) + S->dlyAPF] - S->z1[n] * S->a2; //look here
		S->zK1[(n * (S->K1 + 1)) + S->dlyAPF] = input - S->a1 * S->v[n]; //look here
	}

	output = S->a1 * S->zK1[(n * (S->K1 + 1)) + S->dlyAPF] + S->v[n];

	S->dlyAPF++;
	return output;
}

float allpassHighS(float input, struct Spring *S) {
	int i;
	float output;
	S->outHighAPF[0] = input * S->ahigh + S->zhigh[0];
	S->zhigh[0] = input - S->outHighAPF[0] * S->ahigh;

	for(i = 1; i < S->NUM_APF_HIGH; i++) {
		S->outHighAPF[i] = S->outHighAPF[i-1] * S->ahigh + S->zhigh[i];
		S->zhigh[i] = S->outHighAPF[i-1] - S->outHighAPF[i] * S->ahigh;
	}
	output = S->outHighAPF[S->NUM_APF_HIGH-1];

	return output;
}

float MTDLLowS(float input, struct Spring *S) {
	float output, M, frac, inVal, inVal2, L0Output, tempVal;
	int I;

	if(S->dlyL0Index >= S->L0) {
		S->dlyL0Index = 0;
	}
	if(S->dlyLechoIndex >= S->Lecho) {
		S->dlyLechoIndex = 0;
	}
	if(S->dlyLrippleIndex >= S->Lripple) {
		S->dlyLrippleIndex = 0;
	}

	M = S->gmod * (float)rand() / RAND_MAX;
	M = S->aLPF * M + (1-S->aLPF) * S->LPFPrevVal;
	S->LPFPrevVal = M;

	I = floor(M);
	frac = M - I;

	if(S->dlyL0Index + I >= S->L0) {
		inVal = S->delayL0[S->dlyL0Index + I - S->L0];
	}
	else {
		inVal = S->delayL0[S->dlyL0Index + I];
	}
	if(S->dlyL0Index + I + 1 >= S->L0) {
		inVal2 = S->delayL0[S->dlyL0Index + I + 1 - S->L0];
	}
	else {
		inVal2 = S->delayL0[S->dlyL0Index + I + 1];
	}

	L0Output = frac * inVal + (1 - frac) * inVal2;

	tempVal = S->delayLecho[S->dlyLechoIndex];
	output = S->gripple * tempVal + S->delayLripple[S->dlyLrippleIndex];
	S->delayLripple[S->dlyLrippleIndex] = tempVal;
	S->delayLecho[S->dlyLechoIndex] = S->delayL0[S->dlyL0Index];
	S->delayL0[S->dlyL0Index] = input;

	S->dlyL0Index++;
	S->dlyLechoIndex++;
	S->dlyLrippleIndex++;

	return output;
}

float delayLineHighS(float input, struct Spring *S) {

	if(S->dlyLHighIndex >= S->LHigh) {
		S->dlyLHighIndex = 0;
	}

	float output, Mh, frach, inVal, inVal2;
	int Ih;
	//put in the delay line index reset
	Mh = S->gmodHigh * (float)rand() / RAND_MAX;
	Mh = S->aLPFHigh * Mh + (1 - S->aLPFHigh) * S->LPFPrevValH;
	S->LPFPrevValH = Mh;

	Ih = floor(Mh);
	frach = Mh - Ih;

	if(S->dlyLHighIndex + Ih >= S->LHigh) {
		inVal = S->delayLHigh[S->dlyLHighIndex + Ih - S->LHigh];
	}
	else {
		inVal = S->delayLHigh[S->dlyLHighIndex + Ih];
	}
	if(S->dlyLHighIndex + Ih + 1 >= S->LHigh) {
		inVal2 = S->delayLHigh[S->dlyLHighIndex + Ih + 1 - S->LHigh];
	}
	else {
		inVal2 = S->delayLHigh[S->dlyLHighIndex + Ih + 1];
	}

	output = frach * inVal + (1 - frach) * inVal2;
	S->delayLHigh[S->dlyLHighIndex] = input;

	S->dlyLHighIndex++;

	return output;
}

int initSpring(struct Spring *Spr) {
	//initialize spring values
	int fCnoise = 100;
	//allpassLowS
	Spr->NUM_APF_LOW = 80;
	Spr->fCChirp = 4000;
	Spr->K = fs/(2*(float)Spr->fCChirp);
	Spr->K1 = roundf(Spr->K);
	Spr->d = Spr->K - Spr->K1;
	Spr->dlyAPF = 0;
	Spr->a1 = 0.75;
	Spr->a2 = (1-Spr->d)/(1+Spr->d);
	Spr->z1 = (float *) calloc(Spr->NUM_APF_LOW, sizeof(float));
	Spr->z2 = (float *) calloc(Spr->NUM_APF_LOW, sizeof(float));
	Spr->zK1 = (float *) calloc(Spr->NUM_APF_LOW, (Spr->K1 + 1) * sizeof(float));
	Spr->v = (float *) calloc(Spr->NUM_APF_LOW, sizeof(float));

	//allpassHighS
	Spr->NUM_APF_HIGH = 80;
	Spr->ahigh = -0.6;
	Spr->zhigh = (float *) calloc(Spr->NUM_APF_HIGH, sizeof(float));
	Spr->outHighAPF = (float *) calloc(Spr->NUM_APF_HIGH, sizeof(float));

	//HdcLowS
	Spr->Rdc = 0.995;
	Spr->prevOutVal = 0.;
	Spr->prevInVal = 0.;

	//HeqLowS
	Spr->Keq = 1;//floor(Spr->K);
	Spr->B = 130;
	Spr->fpeak = 95;
	Spr->delay1 = 0.;
	Spr->delay2 = 0.;
	Spr->R = 1 - (M_PI*Spr->B*Spr->Keq/(float)fs);
	Spr->poleAngle = (1+powf(Spr->R,2))/(2*Spr->R)*cos(2*M_PI*Spr->fpeak*Spr->Keq/(float)fs);
	Spr->aEQ1 = -2*Spr->R*(Spr->poleAngle);
	Spr->aEQ2 = powf(Spr->R,2);
	Spr->A0 = (1-Spr->aEQ2)/2;

	//HlpLowS
	Spr->fC = 4000;
	Spr->aLP = (2*M_PI*Spr->fC/(float)fs)/(2*M_PI*Spr->fC/(float)fs+1);
	Spr->prevValLP = 0.;

	//MTDLLowS
	Spr->Nripple = 5;
	Spr->echoTime = 0.050;
	Spr->L = round(fs * Spr->echoTime - Spr->K * Spr->NUM_APF_LOW * (1 - Spr->a1) / (1 + Spr->a1));
	Spr->Lripple = round(2*Spr->K*Spr->Nripple);
	Spr->Lecho = round(Spr->L/5);
	Spr->L0 = Spr->L - Spr->Lripple - Spr->Lecho;
	Spr->gecho = 0.1;
	Spr->gripple = 0.65;
	Spr->delayL0 = (float *) calloc(Spr->L0, sizeof(float));
	Spr->delayLecho = (float *) calloc(Spr->Lecho, sizeof(float));
	Spr->delayLripple = (float *) calloc(Spr->Lripple, sizeof(float));
	Spr->dlyL0Index = 0;
	Spr->dlyLechoIndex = 0;
	Spr->dlyLrippleIndex = 0;
		//for noise/LPF
	Spr->aLPF = (2*M_PI*fCnoise/(float)fs)/(2*M_PI*fCnoise/(float)fs+1);
	Spr->LPFPrevVal = 0.;
	Spr->gmod = 12;

	//DelayLineHighS
	Spr->delayTimeHigh = Spr->echoTime/2.3;
	Spr->LHigh = round((float)fs*Spr->delayTimeHigh);
	Spr->delayLHigh = (float *) calloc(Spr->LHigh, sizeof(float));
	Spr->dlyLHighIndex = 0;
		//for noise/LPF
	Spr->aLPFHigh = (2*M_PI*fCnoise/(float)fs)/(2*M_PI*fCnoise/(float)fs+1);
	Spr->LPFPrevValH = 0.;
	Spr->gmodHigh = 12;

	//structural states
	Spr->c1 = 0;
	Spr->c2 = 0;
	Spr->gDry = 0;
	Spr->gHigh = 0.001;
	Spr->gLow = 1;
	Spr->gHf = -0.4;
	Spr->gLf = -0.4;
	Spr->cHOut = 0.;
	Spr->cLOut = 0.;
	Spr->APFLout = 0.;
	Spr->APFHIn = 0.;
	Spr->HeqOut = 0.;
	Spr->HdcOut = 0.;
	Spr->MTDLout = 0.;
	Spr->DLHout = 0.;

	return 1;
}

int deleteSpring(struct Spring *Spr) {
	free(Spr->z1);
	free(Spr->z2);
	free(Spr->zK1);
	free(Spr->v);
	free(Spr->zhigh);
	free(Spr->outHighAPF);
	free(Spr->delayL0);
	free(Spr->delayLecho);
	free(Spr->delayLripple);
	free(Spr->delayLHigh);
	return 1;
}
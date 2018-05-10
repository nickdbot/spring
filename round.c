#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main() {
	float k;
	int i;
	FILE *f = fopen("test.txt","wt");

	if (f == NULL) {
		printf("error opening file");
		exit(1);
	}

	for(i = 0; i < 14; i++) {
		k = (float)rand();//RAND_MAX;
		//printf("Rand val %i, %2f\n",i,k);
		fprintf(f,"%f",k);
		fprintf(f,"\n");
		printf("%f\n",k);
	}

	fclose(f);

	return 0;
}
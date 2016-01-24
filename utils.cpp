#include "utils.h"


float gettime(){
	struct 	timespec monotime;
	clock_gettime(CLOCK_MONOTONIC, &monotime);
	float curtime = monotime.tv_sec + ((float)monotime.tv_nsec)/1000000000;

	return curtime;
}

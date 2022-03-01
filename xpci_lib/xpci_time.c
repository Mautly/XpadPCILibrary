/*******************************************************
                       xpci_time.c

 Commodity to measure time in program sections.

ATTENTION: Be carefull if used in libs and programs that
           they dont compete on the same timer.
TIPS:      Use index<5 in libs
               index>=5 in progs
*******************************************************/
#include <stdio.h>
#include <sys/time.h>
#include "xpci_time.h"

static struct timeval start[10], finish[10] ;
static int msec;

void xpci_timerStart(int id){
  static struct timeval *startPtr;
  startPtr = start+id;
  gettimeofday (startPtr, 0);
}

void xpci_timerStop(int id){
  static struct timeval *finishPtr, *startPtr;
  startPtr  = start+id;
  finishPtr = finish+id;
  gettimeofday (finishPtr, 0);
  msec =  finishPtr->tv_sec * 1000 + finishPtr->tv_usec / 1000;
  msec -= startPtr->tv_sec * 1000 + startPtr->tv_usec / 1000;
  printf("Elapse from Timer id=%d stop: %d milliseconds\n", id, msec);
}

#ifdef TEST
int main(){
  xpci_timerStart(0);
  xpci_timerStart(1);
  sleep(2);
  xpci_timerStop(0);
  sleep(2);
  xpci_timerStop(1);
}
#endif

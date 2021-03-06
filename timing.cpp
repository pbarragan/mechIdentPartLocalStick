#include "timing.h"

/* 
author: jbenet
os x, compile with: gcc -o testo test.c 
linux, compile with: gcc -o testo test.c -lrt
*/
 
void timing::get_time(timespec& ts) {
 
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts.tv_sec = mts.tv_sec;
  ts.tv_nsec = mts.tv_nsec;
#else
  clock_gettime(CLOCK_REALTIME, &ts);
#endif
 
}

double timing::timeDiff(timespec& ts1, timespec& ts2){
  return (double) ts2.tv_sec - (double) ts1.tv_sec + 
    ((double) ts2.tv_nsec - (double) ts1.tv_nsec)/1000000000; 
}

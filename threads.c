/*=======================================================Assignment1 ========================================*
* =============================================== Real Time Embedded System  ================================*
* ======================================================= CSE 522 ===========================================*/



#include<stdio.h>
#include<sched.h>
#include<string.h>
#include<pthread.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include<semaphore.h>
#include<linux/input.h>
#include<fcntl.h>
#include <time.h>
#include <stdint.h>
#include <sys/time.h>
#include<stdbool.h>
#include <sys/types.h>
#include <sys/syscall.h>

#define KEYBOARD_INPUT "/dev/input/event3" 		// Update this for keyboard input as per your machine
bool enable_PI=true;			     		//flag for enabling the priority inheritence

pthread_mutex_t locks[10] = PTHREAD_MUTEX_INITIALIZER;  // intialize 10 mutexes
bool stopThreads=false;					// flag to stop all the threads while finishing main program
sem_t lock_Key[5];	
struct timespec mainStopTime;				// Used to compare absolute sleep time of main program and periodic thread
static pthread_barrier_t barrier; 			// barrier for starting the threads at the same time

/* ==========================================================================================================================
* doSomeThingPeriodic:	Function to do Periodic tasks.
* Input parameters:	void *strs which specifies the tasks(P priority Period Computations <CriticalSection>)
* Return Value: 	void
* ========================================================================================================================== */

void* doSomeThingPeriodic(void* strs){	
	char *str = (char*)(strs);
	pthread_t id= pthread_self();
	pid_t tid = syscall(SYS_gettid);
	struct sched_param param;
	sched_getparam(id, &param);
	printf( "new Periodic TASK created, ID: %ld  Task: %s\n", (long)(tid), str);
	char *tempstr;
	char *token;

	token = strsep(&str, " ");
	token = strsep(&str, " ");
	token=strsep(&str, " ");
	int timer = atoi(token);
	
	tempstr=strdup(str);
	
	pthread_barrier_wait(&barrier);         	//wait for activation barrier

	struct timespec initTime;
	bool breakNow=false;				// flag to terminate periodic thread when it is in sleep mode

	while(!stopThreads){
		long timersec = 0;
		long timermsec = timer;
		clock_gettime(CLOCK_MONOTONIC, &initTime);
		str=strdup(tempstr);
		token = strsep(&str, " ");
		while( token != NULL ){
			if(*token == 'U'){
				char* temp=++token;	
				int err=pthread_mutex_unlock(&locks[atoi(temp)]);
				if(err!=0)
					printf("error while unlocking Lock: %s\n", temp);
			}
			else if(*token == 'L'){
				char* temp=++token;	
				int err=pthread_mutex_lock(&locks[atoi(temp)]);
				if(err!=0)
					printf("error while locking Lock: %s\n", temp);
			}
			else{	
				int counter = atoi(token);
				for(int i = 0; i < counter; i ++);
			}
			token = strsep(&str, " ");
		}	
		if(stopThreads){
			break;		
		}
		while(timermsec >= 1000){
			timermsec -= 1000;
			timersec++;
		}
		initTime.tv_nsec += (timermsec*1000000);
		initTime.tv_sec+=(timersec);
		while(initTime.tv_nsec >= 1000000000){
			initTime.tv_nsec -= 1000000000;
			initTime.tv_sec++;
		}
		// wait for sleep period of main if time after sleep period is > absolute sleep period of periodic thread		
		if((mainStopTime.tv_sec < initTime.tv_sec) || ((mainStopTime.tv_sec == initTime.tv_sec) && (mainStopTime.tv_nsec < initTime.tv_nsec))){			
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &mainStopTime, NULL);
			breakNow=true;
		}
		// wait for sleep period if absolute time after sleep period is < aboslute time of main sleep
		else{	
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &initTime, NULL); 			// wait for the period
		}		
		if(breakNow){
			break;		
		}
	}
	pthread_exit(NULL); 
}	// End of Periodic Task



/* ==========================================================================================================================
* doSomeThingAPeriodic:	Function to do Aperiodic tasks.
* Input parameters:	void *strs which specifies the tasks(A priority keyEvent Computations <CriticalSection>)
* Return Value: 	void
* ========================================================================================================================== */

void* doSomeThingAperiodic(void* strs){
	char *str = (char*)(strs);
	pthread_t id= pthread_self();
	struct sched_param param;
	sched_getparam(id, &param);
	pid_t tid = syscall(SYS_gettid);
	printf( "new Aperiodic TASK created, ID: %ld Task : %s\n", (long)(tid), str);
	char *token;
	char *tempstr;

	token = strsep(&str, " ");
	token = strsep(&str, " ");
	token=strsep(&str, " ");
	int event = atoi(token);
	
	tempstr=strdup(str);
	pthread_barrier_wait(&barrier);         	//wait for activation barrier
	
	sem_wait(&lock_Key[event]);     		// wait for a key event
	while(!stopThreads){		
		str = strdup(tempstr);		
		token = strsep(&str, " ");
		while( token != NULL ){
			if(*token == 'U'){
				char* temp=++token;	 
				int err=pthread_mutex_unlock(&locks[atoi(temp)]);
				if(err!=0)
					printf("error while unlocking Lock : %s\n", temp);
			}
			else if(*token == 'L'){
				char* temp=++token;	
				int err=pthread_mutex_lock(&locks[atoi(temp)]);
				if(err!=0)
					printf("error while locking Lock : %s\n", temp);
			}
			else{
				int counter = atoi(token);
				for(int i = 0; i < counter; i ++);
			}
			token = strsep(&str, " ");
		}
		if(stopThreads){
			break;				//Exit if main thread sleep has already expired
		}
		sem_wait(&lock_Key[event]);		// Wait for the key event
	}
	pthread_exit(NULL); 
}		// End of Aperiodic task


/* =============================================================================================================================
* readChars:		Function to read Keyboard events for Aperiodic tasks. It generates Semaphore whenever a key event happens
* Input parameters:	Null
* Return Value: 	void
* ============================================================================================================================*/

void *readChars(){
	
	//pid_t tid = syscall(SYS_gettid);
	pthread_t id= pthread_self();
	struct sched_param param;
	sched_getparam(id, &param);
	struct input_event ev;
	int fd;
	ssize_t n;
	
	if(getuid()!=0){
		printf("\n\n--------PERMISSION DENIED!.. Run with SUPERUSER-----\n\n");	
	}
	fd=open(KEYBOARD_INPUT, O_RDONLY);		// Open keyboard input event driver
	if(fd==-1){
	 	printf("Error while openeing the Keyboard Event handler!\n");
	}
	else{

		while(!stopThreads){
			n=read(fd,&ev,sizeof(ev));
			if(n==(ssize_t)-1){
				//error	
			}

			// Check for key Release: Keys(0-4)
			if(ev.type==EV_KEY && ev.value==0 && ev.code==11){
				sem_post(&lock_Key[0]);
			}
			if(ev.type==EV_KEY && ev.value==0 && ev.code==2){
				sem_post(&lock_Key[1]);
			}
			if(ev.type==EV_KEY && ev.value==0 && ev.code==3){
				sem_post(&lock_Key[2]);
			}
			if(ev.type==EV_KEY && ev.value==0 && ev.code==4){
				sem_post(&lock_Key[3]);
			}
			if(ev.type==EV_KEY && ev.value==0 && ev.code==5){
				sem_post(&lock_Key[4]);
			}
		}
	}
	pthread_exit(NULL); 
	return 0;
}		// end of keyborad thread task


/* ==========================================================================================================================
* main:			Function to create threads for Periodic/Aperiodic/Reading Keyboardinput tasks.
* Input parameters:	argc(no of command line arguments), argv(array of input arguments)
* Return Value: 	0 on successful termination
* ========================================================================================================================== */


int main(int argc, char *argv[])
{	
	if(argc < 2){
		printf("---------Enter the input file name through the command line--- \n");
		exit(-1);
	}
	struct timespec init_Time;
	clock_gettime(CLOCK_MONOTONIC, &init_Time);  // starting time of the program
	//pid_t ttid = syscall(SYS_gettid);	    //ID of the main thread
	//pthread_t id= pthread_self();
	pthread_attr_t attrKB;
	struct sched_param priorityparamKB;
	struct timespec current_Time;
	
	struct timespec sleepTime; 
	pthread_attr_init(&attrKB);
	priorityparamKB.sched_priority=97;
	pthread_attr_setschedparam(&attrKB,&priorityparamKB);

	pthread_t readt;	    		// thread to read the keyboard wait

	pthread_create(&readt, &attrKB, readChars, NULL); // create keyboard thread
	
	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init (&mutex_attr);
	pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT);
	if(enable_PI){
		for(int i=0;i<10;i++){
			 pthread_mutex_init (&locks[i], &mutex_attr); 
		} 
	}
	
	stopThreads=false;	    		// Flag which is set when main is about to terminate 
	sem_init(&lock_Key[0],0,0); 		// initialize the semaphore for key event 0
	sem_init(&lock_Key[1],0,0); 		// initialize the semaphore for key event 1
	sem_init(&lock_Key[2],0,0); 		// initialize the semaphore for key event 2
	sem_init(&lock_Key[3],0,0); 		// initialize the semaphore for key event 3
	sem_init(&lock_Key[4],0,0); 		// initialize the semaphore for key event 4

	FILE *fp;			  	// file handler

	char buff[255];		          	// string to store the first line of the text file

	fp = fopen(argv[1], "r");
	if(fp == 0){
		printf("\n\n-------------Unable to open input file-------- \n\n");
		exit(-1);	
	}	

	fgets(buff, 255, fp);	          	// read the first line

	char *token = strtok(buff, " ");  	// tokenize the first string
	int num_tasks=atoi(token);	  	// number of tasks	
	pthread_t tid[num_tasks];	  	// declare pthreads 	
	char strbuff[num_tasks][255];     	// srting array to store the indiviual task info 
	int j=0;  
	token = strtok(NULL, " ");
	int netperiod=atoi(token);	    	//total duration of the main task	
	char tempstr[num_tasks][255];

	pthread_barrier_init(&barrier, NULL, num_tasks);// activate all threads using a barrier		
	
	pthread_attr_t attr[num_tasks];
	struct sched_param priorityparam[num_tasks];

	priorityparam[0].sched_priority = sched_get_priority_max(SCHED_FIFO);
	// Parse the input read from the file
	while(fgets(strbuff[j], 255, fp) != NULL) {
		memset(tempstr[j], '\0', sizeof(tempstr[j]));
		strcpy(tempstr[j],strbuff[j]);
		char* type= strtok(strbuff[j], " ");
		int priority=0;
		
		int policy;
		int ret=pthread_attr_init(&attr[j]);
		pthread_attr_getschedpolicy (&attr[j], &policy);
		pthread_attr_setschedpolicy(&attr[j], SCHED_FIFO);
		pthread_attr_getschedpolicy (&attr[j], &policy);
		if(ret!=0){
			printf("\nUnable to initialize pthread attributes\n");
		}
		
		if( strcmp(type,"P")==0){	// Check if it is periodic task
			priority= atoi(strtok(NULL, " "));
			priorityparam[j].sched_priority=priority;
			pthread_attr_setschedparam(&attr[j],&priorityparam[j]);
			int errr=pthread_create(&(tid[j]), &attr[j], doSomeThingPeriodic, tempstr[j]);
			if(errr!=0){
				printf("error creating a Periodic thread!\n");	
			}	
		}
		if( strcmp(type,"A")==0){	// Check if it is Aperiodic task
			priority= atoi(strtok(NULL, " "));			
			priorityparam[j].sched_priority=priority;
			pthread_attr_setschedparam(&attr[j],&priorityparam[j]);
			int errr=pthread_create(&(tid[j]), &attr[j], doSomeThingAperiodic, tempstr[j]);
			if(errr!=0){
				printf("error creating an Aperiodic thread!\n");	
			}	
		}
		j++;

	} 

	fclose(fp);		// close the file handler
	clock_gettime(CLOCK_MONOTONIC, &current_Time);
	long diffnsec=(long)((current_Time.tv_nsec-init_Time.tv_nsec)/1000000.0);
	long diffsec=(current_Time.tv_sec-init_Time.tv_sec)*1000;
	long totaltime=diffnsec+diffsec;
	
	long netsec = 0;
	long netmsec = netperiod;
	while(netmsec > 1000){
		netmsec-=1000;
		netsec++;
		
	}
	sleepTime.tv_sec=netsec + init_Time.tv_sec;
	sleepTime.tv_nsec=netmsec*1000000 + init_Time.tv_nsec;
	while(sleepTime.tv_nsec > 1000000000){
	
		sleepTime.tv_nsec-=1000000000;
		sleepTime.tv_sec++;
	}
	mainStopTime.tv_sec = (sleepTime.tv_sec);
	mainStopTime.tv_nsec = (sleepTime.tv_nsec);	
	usleep((((long)(netperiod) - totaltime)*1000.0));	
	stopThreads=true;

	for(int j=0;j<5;j++){
		sem_post(&lock_Key[j]);	
	}

	for(int j=0;j<num_tasks;j++){
		
		pthread_join(tid[j],NULL);
	}
	return 0;
}		// End of main program

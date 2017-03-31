/*
* Created by Deepak and Ankush on 3/31/2017.
* All rights reserved
* Contains the implementation for EDF(Earliest Deadline First), RM(Rate Monotonic) and DM(Deadline Monotonic) scheduling algorithms
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

struct taskBody{
	float wcet;
	float deadline;
	float period;
};

bool EDF_check_Util(struct taskBody *tasks,int num_task){
	float util=0;
	for(int i=0;i<num_task;i++){
		float min=tasks[i].deadline;
		if(min>tasks[i].period){
			min=tasks[i].period;		
		}
		util+=tasks[i].wcet/min;
	}
	if(util>1.0){
		return false;	
	}
	return true;

}
bool EDF_check_loadingFactor(int t1, int t2, struct taskBody *tasks,int num_task){
	int util=0;
	for(int i=0;i<num_task;i++){
		if(tasks[i].deadline<=(t2-t1)){
			util+=(1+floor((t2-tasks[i].deadline)/tasks[i].period))*tasks[i].wcet;
		}
	}
	printf("Util is %f\n",(float)util/(float)t2);
	if((float)util/(float)t2>1.0){
		return false;	
	}
	return true;
}

void EDF_check(struct taskBody *tasks,int num_task , int i){

	/**********************Utilization Approach************/
	if(EDF_check_Util(tasks,num_task)){
		printf("Taskset %d schedulable by Utilization approach EDF\n",i);
		return;	
	}
	printf("Taskset %d not schedulable by Utilization approach checking for Loading Factor EDF\n",i);
	
		
	/******************** Loading factor approach***************/
	int l=0;
	for(int i=0;i<num_task;i++){
		l+=tasks[i].wcet;
	}
	int newl=0;
	/* Calculate the busy period */
	while(1){
		newl=0;
		for(int i=0;i<num_task;i++){
		    newl+=ceil(l/tasks[i].period)*tasks[i].wcet;
		    //printf("%d\n",newl);	
		}
		if(l==newl){
			printf("Taskset %d has a busy period of %d EDF\n",i,newl);
			break;			
		}
		else{
			l=newl;			
		}
	}
	/* Check for the critical instances upto the busy period */
	for(int j=0;j<num_task;j++){
		int init_d=tasks[j].deadline;
		while(init_d<=l){
			if(!EDF_check_loadingFactor(0,init_d,tasks,num_task))
				printf("Taskset %d is not schedualable EDF\n",i);
			init_d+=tasks[j].period;			
		}
	}
	printf("Taskset %d is schedualable EDF\n",i);		
}

bool RM_check_Util(struct taskBody *tasks,int num_task){
	float util=0;
	for(int i=0;i<num_task;i++){
		float min=tasks[i].deadline;
		if(min>tasks[i].period){
			min=tasks[i].period;		
		}
		util+=tasks[i].wcet/min;
	}
	//printf("util RM %f\n", util);
	//printf("upper bound RM %f\n", num_task*(pow(2.0,(double)(1/(double)(num_task)))-1));
	if(util>(num_task*(pow(2.0,(double)(1/(double)(num_task)))-1))){
		return false;	
	}
	return true;

}

bool RM_check_RT(struct taskBody *tasks,int num_task ){

	float a=0;
	for(int i=0;i<num_task;i++){
		a+=tasks[i].wcet;	
	}
	while(1){
		//printf("a is %f \n : ",a); 
		float newa=tasks[num_task-1].wcet;
		for(int i=0;i<num_task-1;i++){
			newa+=ceil(a/tasks[i].period)*tasks[i].wcet;	
		}
		if(a==newa){			
			break;
		}
		else
			a=newa;
	}
	if(a<tasks[num_task-1].period){
		return (true && (RM_check_Util(tasks,num_task-1)));	
	}
	else
		return false;
}

void RM_check(struct taskBody *tasks,int num_task , int i){
	bool result = false;
	if(RM_check_Util(tasks,num_task)){
		printf("Taskset %d schedulable by Utilization approach RM\n",i);
		return;	
	}
	else{
		printf("Taskset %d not schedulable by Utilization approach RM\n",i);
		int n=num_task;	
		while(n>0){
			result = RM_check_RT(tasks,n);
			if(result){
				printf("Taskset %d schedulable by RT approach RM : Range = 1 to %d\n",i, n);
				break;
			}
			else{
				n--;
			}
		}
	}
	if(!result){
		printf("Taskset %d not schedulable by RT approach RM\n",i);						
	}	
}

int main(int argc, char *argv[])
{	
	if(argc < 2){
		printf("---------Enter the input file name through the command line--- \n");
		exit(-1);
	}

	FILE *fp;			  	// file handler
	fp = fopen(argv[1], "r");
	if(fp == 0){
		printf("\n\n-------------Unable to open input file-------- \n\n");
		exit(-1);	
	}	
	int num_taskSets;
	fscanf(fp,"%d", &num_taskSets);	        // read the first line
	struct taskBody temp;
	printf("%d\n", num_taskSets);
	for(int i=0;i<num_taskSets;i++){
		int num_task;
		
		fscanf(fp,"%d", &num_task);	// read the first line
		//printf("%d\n", num_task);		
		struct taskBody *tasks;
		tasks=calloc(num_task, sizeof(temp));
		for(int i=0;i<num_task;i++){
			fscanf(fp,"%f", &(tasks+i)->wcet);	
			fscanf(fp,"%f", &(tasks+i)->deadline);
			fscanf(fp,"%f", &(tasks+i)->period);
			printf("%f  %f  %f\n", tasks[i].wcet, tasks[i].deadline,tasks[i].period);
				
		}
		
		EDF_check(tasks,num_task, i);
		
		RM_check(tasks,num_task, i);
		printf("\n");
	}	
	
	fclose(fp);		// close the file handler

	return 0;
}	// End of main program
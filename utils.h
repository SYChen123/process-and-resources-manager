#include<stdio.h>
#include<stdlib.h>
typedef struct List List;
typedef struct status status;
typedef struct PCB PCB;
typedef struct RCB RCB;
typedef struct RList RList;
typedef struct WList WList;
typedef struct LIST LIST;

typedef struct status {
	int type;//0-running 1-ready 2-blocked
	LIST *readyORblocked; //point to ready or blocked list
}status;

typedef struct PCB {
	char name;
	int PID;
	RList *resources; //point to resources list being occupied by this process.
	status state; //type and list
	PCB *parent;
	PCB *children;
	PCB *siblings; //siblings is a circular linked list
	int priority; //0 1 2 priority is higher when the number is bigger
}PCB;

typedef struct List {
	PCB *p;
	List *next;
}List;

typedef struct WList {//for waiting list of RCB
	PCB *p;
	int req_num;//resources num requested
	WList *next;
}WList;

typedef struct RCB {
	int RID;
	int all; //num of resources
	int available; //available resources
	WList *waiting_list; //blocked processes awaiting this resource
}RCB;

typedef struct RList {//for resources list of PCB
	//resources list for PCB which stores number of used resources of specific process
	RCB *R;//point to R1 or R2 or R3 or R4
	int occupied_num;
	RList *next;
}RList;

typedef struct LIST {
	//array index 0,1,2 corresponding to the priority
	//which means processes of the highest priority are maintained in list[2]
	//processes of the middle priority are maintained in list[1]
	//processes of the lowest priority are maintained in list[0]
	List list[3];  
}LIST;





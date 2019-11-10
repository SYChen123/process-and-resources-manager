#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include"utils.h"
PCB *current = NULL;
LIST *RL = (LIST*)malloc(100*sizeof(LIST)), *BL = (LIST*)malloc(100*sizeof(LIST));
RCB r[4];//rid from 0 to 3

void createProcess(char name, int pid, int prio);
void insertInRLBL(PCB *ptr, int flag);
void deleteInRLBL(PCB *ptr, int flag);
void updateListsandPointers(PCB *goal);
int killTree(PCB *goal);
void destroyProcess(int pid);
void ReleaseProcResources(PCB *ptr);
void Request(int rid, int n);
void Release(PCB *p, int rid, int n, int isSchedule);
void preept(PCB *ptr);
void Scheduler();
void Timeout();
void kernel_Init();
void Shell();
void iterProcTree(PCB *ptr, int iterSibFlag);


void createProcess(char name, int pid, int prio) {
	PCB *p=(PCB*)malloc(sizeof(PCB)); //new process
	p->name = name;
	p->PID = pid;
	p->priority = prio;
	p->state = { 1,RL };//new process should be put into ready list
	//link to creation tree
	p->parent = current;
	if (current == NULL) {//for init. When there is only one process, its siblings point to itself
		p->children = NULL;
		p->siblings = p;
	}
	else if (current != NULL) {
		PCB *tmp = current->children;
		current->children = p;
		p->children = NULL;
		if (tmp == NULL) {//current has no child
			p->siblings = p;
		}
		else {
			p->siblings = tmp;
			//find who's siblings point to tmp and then make it point to p
			//also applicable for situation where current has only one child
			PCB *tmp2 = tmp;
			while (tmp2->siblings != tmp) {
				tmp2 = tmp2->siblings;
			}
			tmp2->siblings = p;
		}
	}
	p->resources = NULL; //do not require resources when first being created

	//insert into ready list
	insertInRLBL(p, p->state.type);
	//schedule
	Scheduler();
	//return *p;
}

void insertInRLBL(PCB *ptr, int flag) {
	if (flag == 1) {//ready
		List *li = &RL->list[ptr->priority], *res = (List*)malloc(sizeof(List));
		res->next = NULL; res->p = ptr;
		if (li->p == NULL && li->next == NULL) {//for init
			li->p = res->p;
			li->next = res->next;
			return;
		}
		while (li != NULL) {
			if (li->p->PID == ptr->PID)return;//already in blocked list
			if (li->next == NULL)li->next = res;//reach the end
			li = li->next; //reach the end of the ready list
		}
	}
	else if (flag == 2) {//blocked
		List *pt = &BL->list[ptr->priority], *res2 = (List*)malloc(sizeof(List));
		res2->next = NULL; res2->p = ptr;
		if (pt->p == NULL && pt->next == NULL) {//for init
			pt->p = res2->p;
			pt->next = res2->next;
			return;
		}
		while (pt != NULL) {
			if (pt->p->PID == ptr->PID)return;//already in blocked list
			if (pt->next == NULL)pt->next = res2;//reach the end
			pt = pt->next; //reach the end of the ready list
		}
	}
}

void deleteInRLBL(PCB *ptr, int flag) {//ptr is the node need to be deleted
	if (flag == 1) {
		List *tmp = &RL->list[ptr->priority];
		if (tmp->next == NULL) {//only one or no node in ready list
			if (tmp->p != NULL && tmp->p->PID == ptr->PID) RL->list[ptr->priority] = { NULL,NULL };//set NULL
			return;
		}
		//more than one
		if (tmp->p->PID == ptr->PID) {//goal is the first in ready list
			//cannot free tmp directly so copy content of tmp->next to tmp and free tmp->next
			List *copy = tmp->next;
			tmp->p = tmp->next->p;
			tmp->next = tmp->next->next;
			free(copy);
		}
		else {//goal isn't the first one in ready list
			while (true) {//get locatioin of ptr-1, which is tmp in ready list
				if (tmp->next->p->PID == ptr->PID)break;
				tmp = tmp->next;
			}
			//goal is the last node and then make tmp the last node
			if (tmp->next->next == NULL) {
				free(tmp->next->next);
				tmp->next = NULL;
			}
			else {//goal is not the last node
				List *ptr1 = tmp->next;//copy goal
				tmp->next = tmp->next->next;
				free(ptr1);
			}
		}
	}
	else if (flag == 2) {
		List *tmp2 = &BL->list[ptr->priority];
		if (tmp2->next == NULL) {//only one or no node in ready list
			if (tmp2->p != NULL && tmp2->p->PID == ptr->PID) BL->list[ptr->priority] = { NULL,NULL };//set NULL
			return;
		}
		//more than one
		if (tmp2->p->PID == ptr->PID) {//goal is the first in ready list
			//cannot free tmp directly so copy content of tmp->next to tmp and free tmp->next
			List *copy = tmp2->next;
			tmp2->p = tmp2->next->p;
			tmp2->next = tmp2->next->next;
			free(copy);
		}
		else {//goal isn't the first one in ready list
			while (true) {//get locatioin of ptr-1, which is tmp in ready list
				if (tmp2->next->p->PID == ptr->PID)break;
				tmp2 = tmp2->next;
			}
			//goal is the last node and then make tmp the last node
			if (tmp2->next->next == NULL) {
				free(tmp2->next->next);
				tmp2->next = NULL;
			}
			else {//goal is not the last node
				List *ptr2 = tmp2->next;//copy goal
				tmp2->next = tmp2->next->next;
				free(ptr2);
			}
		}
	}
}

void updateListsandPointers(PCB *goal) {
	//update ready and blocked list
	deleteInRLBL(goal, goal->state.type);
	//update resources available
	if (goal->resources != NULL) { //there are resoures occupied by this process
		RList *r = goal->resources;
		while (r != NULL) {
			r->R->available += r->occupied_num;
			r = r->next;
		}
	}
	//update all lists and pointers
	//delete goal
	if (goal->parent->children == goal) {
		//update parent and children pointer
		if (goal->siblings == goal){
			goal->parent->children = NULL;//no siblings of goal
		}
		else {
			goal->parent->children = goal->siblings;
			//update siblings pointer
			PCB *t = goal->siblings;
			if (goal->siblings != goal) {//at least one sibling of goal
				while (true) {
					if (t->siblings == goal) {
						t->siblings = goal->siblings;
						break;
					}
					t = t->siblings;
				}
			}
		}
	}
	else {//no need to update parent,children pointer but have to update siblings pointer
		//at least one sibling for goal
		PCB *t2 = goal->siblings;
		if (goal->siblings != goal) {//at least one sibling of goal
			while (true) {
				if (t2->siblings == goal) {
					t2->siblings = goal->siblings;
					break;
				}
				t2 = t2->siblings;
			}
		}
	}
	free(goal);
}

int killTree(PCB *goal) {//1 finished -1 still need to kill
	if (goal == NULL) {//work finished
		return 1;
	}
	else if (goal->children == NULL) {//leaves children of goal has no children
		ReleaseProcResources(goal);
		updateListsandPointers(goal);//delete children of goal
		return -1;//still need to kill goal's siblings 
		//but after updateListsandPointers(goal), 
		//goal's first sibling becomes direct child of goal's parent
	}
	else 
		while (killTree(goal->children) != 1);

	//the code below will work only if goal has children
	//if goal had no children, then goal would be killed by the 'else if' branch and then return
	//if goal had children, 'else' branch would work and goal's children would be killed while goal wouldn't be killed
	//so we need to add some extra codes to kill goal for 'else' branch
	ReleaseProcResources(goal);
	updateListsandPointers(goal);
	return 1;
}

void destroyProcess(int pid) {
	PCB *p = current, *goal = NULL;
	PCB *psib = p->siblings;
	int sign = 0;
	if (p->PID == pid) {//delete current process
		goal = p;
		current = NULL;
	}
	else {
		//find goal
		while (true) {
			//search siblings
			while (psib != p) {
				if (psib->PID == pid) {
					goal = psib;
					sign = 1;
					break;
				}
				psib = psib->siblings;
			}
			if (sign == 1)break;
			//search children
			p = p->children;
			psib = p->siblings;
			if (p->PID == pid) {
				goal = p;
				break;
			}
		}
	}
	//find the goal and then destroy it and its children and release all their resources
	if (goal == NULL)return;
	else {
		killTree(goal);
	}
	//schedule
	Scheduler();
}

void ReleaseProcResources(PCB *ptr) {
	//release resources occupied by specific process when destroying it
	RList *r1 = ptr->resources, *clr = NULL;
	if (r1 != NULL) {//there are resources occupied by it
		while (r1 != NULL) {
			Release(ptr, r1->R->RID, r1->occupied_num, 0);//set isSchedule=0 so that we can release all processes and then schedule altogether
			//when I free the first node in ptr->resources, 
			//I make ptr->resources point to ptr->resources->next and free ptr->resources
			r1 = ptr->resources;		
		}
	}
	//update waiting list of all resources
	WList *w1 = NULL, *w1_prev = w1;
	for (int i = 0; i < 4; i++) {
		w1 = r[i].waiting_list;
		w1_prev = w1;
		while (w1 != NULL && w1->p!=NULL) {
			if (w1->p->PID == ptr->PID) {
				if (w1->next == NULL) { 
					if (w1_prev == r[i].waiting_list && w1==r[i].waiting_list) 
						r[i].waiting_list = NULL;
					else w1_prev->next = NULL; 
				}//end of the waiting list
				else w1_prev->next = w1->next;
				free(w1);
				w1 = NULL;
				break;
			}
			w1_prev = w1;
			w1 = w1->next;
		}
	}
}



//所有的资源申请请求按照FIFO的顺序进行
void Request(int rid, int n) {
	if (r[rid].available >= n) {//ok, insert it into PCB.resources
		r[rid].available -= n;
		//update resources list in PCB
		if (current->resources == NULL) {//no resource was requested before 
			RList *a = (RList*)malloc(sizeof(RList));
			a->R = &(r[rid]); a->occupied_num = n; a->next = NULL;
			current->resources = a;
		}
		else {
			RList *tmp = current->resources;
			while (true) {
				if (tmp->R->RID == rid) {
					//this specific resources have been requested by specific process before
					tmp->occupied_num += n;
					break;
				}
				else if (tmp->next == NULL) {//reach the end of list
					RList *b = (RList*)malloc(sizeof(RList));
					b->R = &(r[rid]); b->occupied_num = n; b->next = NULL;
					tmp->next = b;
					break;
				}
				tmp = tmp->next;
			}
		}
		
	}
	else {//blocked this process
		if (n > r[rid].all) {//resources requested > all 
			printf("error!");
			return;
		}
		current->state.type = 2;//blocked
		current->state.readyORblocked = BL;//blocked
		//insert PCB into waiting list of r[rid]
		WList *l = r[rid].waiting_list;//l point to waiting list of r[rid]
		if (l->p == NULL) {//no process waiting for this resource
			l->p = current; l->req_num = n; l->next = NULL;
		}
		else {//there is at least one process waiting for this resource
			WList *res = (WList*)malloc(sizeof(WList));
			res->p = current; res->req_num = n; res->next = NULL;
			while (true) {
				if (l->p->PID == current->PID)break;//already in waiting list
				if (l->next == NULL) {
					l->next = res;//reach the end
					break;
				}
				l = l->next;
			}
		}
		//insert PCB into blocked list
		insertInRLBL(current, current->state.type);
		//schedule
		Scheduler();
	}
}

void Release(PCB *p,int rid,int n,int isSchedule) {
	/*isSchedule is 0 when destroying a process subtree and don't wanna call scheduler
	//everytime we release resources occpuied by a process of subtree 
	//so that current process can do the rest work, after which we can call scheduler and schedule altogether
	*/
	//remove corresponding resources from current->resources and update r[rid].available
	RList *ptr = p->resources;
	if (ptr->R->RID == rid) {//deal with situation where the goal is the first one in list
		if (ptr->next == NULL) {//reach the end
			r[rid].available += n;
			if (n == ptr->occupied_num) {//release all the resources occupied
				free(p->resources);
				p->resources = NULL;
			}
			else ptr->occupied_num -= n;
		}
		else {//ptr is not the end of the list
			r[rid].available += n;
			p->resources = ptr->next;
			if(n==ptr->occupied_num)
				free(ptr);
			else ptr->occupied_num -= n;
		}
	}
	else {//goal is not the first one in list
		while (true) {
			if (ptr == NULL)break;
			if (ptr->next->R->RID == rid) {//next one of ptr is the goal
				if (ptr->next->next == NULL) {//ptr->next is the end of list
					r[rid].available += n;
					if (n == ptr->next->occupied_num) {//release all resources occupied
						free(ptr->next);
						ptr->next = NULL;
					}
					else ptr->next->occupied_num -= n;
					break;
				}
				else {//ptr->next is not the end of list
					RList *copy = ptr->next;
					r[rid].available += n;
					if (n == ptr->next->occupied_num) {
						ptr->next = ptr->next->next;
						free(copy);
					}
					else ptr->next->occupied_num -= n;
					break;
				}
			}
			ptr = ptr->next;
		}
	}

	//activate processes in the waiting list of r[rid]
	WList *w = r[rid].waiting_list, *w_prev = w;
	RList *rl = NULL;
	int reqNum;
	if (w!=NULL && w->p != NULL) {//there are processes awaiting r[rid]
		reqNum = w->req_num;
		while (w != NULL && r[rid].available > reqNum) {
			deleteInRLBL(w->p, w->p->state.type);//delete node from blocked list
			w->p->state.type = 1;//ready
			w->p->state.readyORblocked = RL;
			insertInRLBL(w->p, w->p->state.type);//insert into ready list

			//insert r[rid] into w->p->resources
			rl = w->p->resources;
			if (rl == NULL) {//no resources are occupied by w->p
				RList *res = (RList*)malloc(sizeof(RList));
				res->R = &r[rid]; res->occupied_num = reqNum; res->next = NULL;
				w->p->resources = res;
			}
			else {//at least one resource is occupied by w->p 
				while (rl != NULL) {
					if (rl->R->RID == rid) {//r[rid] has been requested by this process
						rl->occupied_num += reqNum;
						break;
					}
					else if (rl->next == NULL) {//reach the end
						RList *res1 = (RList*)malloc(sizeof(RList));
						res1->R = &r[rid]; res1->occupied_num = reqNum; res1->next = NULL;
						rl->next = res1;
						break;
					}
					rl = rl->next;
				}
			}
			w_prev = w;
			w = w->next;//activate next process blocked by this resource
			if(w!=NULL)
				reqNum = w->req_num;
			free(w_prev);
		}
		//update waiting list
		if (w == NULL) {//make waiting list point to new area
			r[rid].waiting_list = (WList*)malloc(sizeof(WList));
			r[rid].waiting_list->p = NULL; r[rid].waiting_list->req_num = 0; r[rid].waiting_list->next = NULL;
		}
		else {
			r[rid].waiting_list = w;
		}
	}
	//schedule
	if(isSchedule==1)
		Scheduler();
}

void preept(PCB *ptr) {
	if (current == NULL) {//for init
		current = ptr;
		deleteInRLBL(ptr, ptr->state.type);
		current->state.type = 0;
	}
	else {
		if (current->PID != ptr->PID) {
			deleteInRLBL(ptr, ptr->state.type);//delete from rl or bl
			PCB *copy = current;//copy current process
			if (copy->state.type == 0)//if current process is not blocked
				copy->state.type = 1;//reset state of copy process in order to insert into rl or bl
			insertInRLBL(copy, copy->state.type);
			current = ptr;//update current process
			current->state.type = 0;
		}
		else {//current and the process from ready list are the same which means only this process can be running except init process
			deleteInRLBL(ptr, ptr->state.type);
			current->state.type = 0;
		}
	}
}

void Scheduler() {
	List *ptr = NULL;
	if (RL->list[2].p != NULL) {//there is process with priority 2
		ptr = &RL->list[2];
		if (current == NULL || current->priority < ptr->p->priority || current->state.type != 0)
			preept(ptr->p);
	}
	else if (RL->list[1].p != NULL) {//priority 1
		ptr = &RL->list[1];
		if (current == NULL || current->priority < ptr->p->priority || current->state.type != 0)
			preept(ptr->p);
	}
	else if (RL->list[0].p != NULL) {//priority 0
		ptr = &RL->list[0];
		if (current == NULL || current->priority < ptr->p->priority || current->state.type != 0)
			preept(ptr->p);
	}
}

void Timeout() {
	current->state.type = 1;
	insertInRLBL(current, current->state.type);
	Scheduler();
}

void kernel_Init() {
	current = NULL;
	for (int i = 0; i < 4; i++) {//init resources
		r[i].all = i + 1;
		r[i].available = i + 1;
		r[i].RID = i;
		r[i].waiting_list = (WList*)malloc(sizeof(WList));
		r[i].waiting_list->p = NULL; r[i].waiting_list->req_num = 0; r[i].waiting_list->next = NULL;
	}
	for (int i = 0; i < 3; i++) {//init ready and blocked list
		RL->list[i] = { NULL,NULL };
		BL->list[i] = { NULL,NULL };
	}
	createProcess('i',0, 0);//init process with pid 0 and priority 0
	printf("%c\n", current->name);
	//test
	/*createProcess('x',1, 1);//x
	printf("%c", current->name);
	createProcess('p',2, 1);//p
	printf("%c", current->name);
	createProcess('q',3, 1);//q
	printf("%c", current->name);
	createProcess('r',4, 1);//r
	printf("%c", current->name);
	destroyProcess(1);
	Timeout();//proc2
	printf("%c", current->name);
	Request(1, 1);
	printf("%c", current->name);
	Timeout();//proc3
	printf("%c", current->name);
	Request(2, 3);
	printf("%c", current->name);
	Timeout();//proc4
	printf("%c", current->name);
	Request(3, 3);
	printf("%c", current->name);
	Timeout();
	printf("%c", current->name);
	Timeout();
	printf("%c", current->name);
	Request(2, 1);
	printf("%c", current->name);
	Request(3, 2);
	printf("%c", current->name);
	Request(1, 2);
	printf("%c", current->name);

	Timeout();
	printf("%c", current->name);
	destroyProcess(3);
	printf("%c", current->name);
	Timeout();
	printf("%c", current->name);
	Timeout();
	printf("%c", current->name);*/
}

void Shell() {
	//shell version
	char buffer[20], cr[2] = { 'c','r' }, init[4] = { 'i','n','i','t' };
	char req[3] = { 'r','e','q' }, rel[3] = {'r','e','l'}, de[2] = { 'd','e' }, to[2] = { 't','o' };
	char ls[2] = { 'l','s' };
	int processNum = 0, crpid = 1, priority;
	char pname;
	int rid, rnum;
	int depid;
	while (true) {//main loop
		printf("CSYshell> ");
		gets_s(buffer, 20);
		if (buffer[0]==cr[0] && buffer[1]==cr[1]) {
			//create a process by 'cr x 1'
			pname = buffer[3];
			priority = atoi(&buffer[5]);
			createProcess(pname, crpid, priority);
			crpid += 1;
			processNum += 1;
			printf("%c\n", current->name);
		}
		else if (buffer[0] == init[0] && buffer[1] == init[1] && buffer[2] == init[2] && buffer[3] == init[3]) {
			kernel_Init();
		}
		else if (buffer[0] == req[0] && buffer[1] == req[1] && buffer[2]==req[2]) {
			//request some resources by 'req R0 1'
			rid = atoi(&buffer[5]);
			rnum = atoi(&buffer[7]);
			Request(rid, rnum);
			printf("%c\n", current->name);
		}
		else if (buffer[0] == rel[0] && buffer[1] == rel[1] && buffer[2] == rel[2]) {
			//release some resources by 'rel R0 1'
			rid = atoi(&buffer[5]);
			rnum = atoi(&buffer[7]);
			Release(current, rid, rnum,1);
			printf("%c\n", current->name);
		}
		else if (buffer[0] == de[0] && buffer[1] == de[1]) {
			//destroy process and its children by 'de 1'
			depid = atoi(&buffer[3]);
			destroyProcess(depid);
			printf("%c\n", current->name);
		}
		else if (buffer[0] == to[0] && buffer[1] == to[1]) {
			//time out by 'to'
			Timeout();
			printf("%c\n", current->name);
		}
		else if (buffer[0] == ls[0] && buffer[1] == ls[1]) {
			//list all processes info or resources info using ls -p or ls -r
			if (buffer[4] == 'p') {//list all processes
				PCB *p = current;
				while (p->parent != NULL) p = p->parent;//find init process
				iterProcTree(p,1);
			}
			else if (buffer[4] == 'r') {//list all resources info
				WList *ptr = NULL;
				char waitingProc[100];
				int waitingProcCount = 0;
				for (int i = 0; i < 4; i++) {
					waitingProcCount = 0;
					printf("*****************************************\n");
					printf("Resource %d 共有%d个\n", r[i].RID, r[i].all);
					printf("Resource %d 空闲%d个\n", r[i].RID, r[i].available);
					ptr = r[i].waiting_list;
					while (ptr!=NULL && ptr->p != NULL) {
						waitingProc[waitingProcCount] = ptr->p->name;
						ptr = ptr->next;
						waitingProcCount += 1;
					}
					if (waitingProcCount == 0)printf("没有进程在等待Resource %d\n", r[i].RID);
					else {//print processes info which are waiting for this resource
						printf("有进程在等待Resource %d，分别是", r[i].RID);
						for (int k = 0; k < waitingProcCount; k++) {
							printf(" %c", waitingProc[k]);
						}
						printf("\n");
					}
				}
			}
		}
	}
}

void iterProcTree(PCB *ptr, int iterSibFlag) {//find all nodes on process tree
	//itersibflag is used to avoid that every process is searching its siblings and trap into infinite loop
	printf("*****************************************\n");
	switch (ptr->state.type) {
	case 0:
		printf("Process %c 状态为 正在运行\n", ptr->name);
		break;
	case 1:
		printf("Process %c 状态为 就绪\n", ptr->name);
		break;
	case 2:
		printf("Process %c 状态为 阻塞\n", ptr->name);
		break;
	}
	if(ptr->parent==NULL)printf("Process %c 是init进程，没有父进程 %c\n", ptr->name);
	else printf("Process %c 的父进程为 %c\n", ptr->name, ptr->parent->name);
	if(ptr->children==NULL)printf("Process %c 没有子进程\n", ptr->name);
	else printf("Process %c 的直接子进程为 %c\n", ptr->name, ptr->children->name);
	printf("Process %c 的优先级为 %d\n", ptr->name, ptr->priority);
	RList *r1 = ptr->resources;
	if (r1 == NULL)
		printf("Process %c 没有占用资源\n", ptr->name);
	else {
		printf("Process %c 占用的资源id和个数分别为", ptr->name);
		while (r1 != NULL && r1->R != NULL) {
			printf(" (%d,%d)", r1->R->RID,r1->occupied_num);
			r1 = r1->next;
		}
		printf("\n");
	}

	if (ptr->children != NULL) {//has children
		iterProcTree(ptr->children,1);
	}
	else if (ptr->siblings == ptr) {//no children and no siblings
		return;
	}
	else if (ptr->siblings != NULL && iterSibFlag == 1) {//no children, has siblings and can be itered
		PCB *tmp = ptr->siblings;
		while (tmp != ptr) {
			iterProcTree(tmp, 0);
			tmp = tmp->siblings;
		}
	}
}

int main() {
	Shell();
	system("pause");
	return 0;
}
# process-and-resources-manager

用C语言模拟了一个简单的进程资源管理器。功能有创建、销毁进程，进程申请、释放资源，基于优先级的调度，时钟中断。

进程有ready, blocked, running三种状态；PCB的属性有父进程，子进程，兄弟进程，占用的资源链表，进程状态等，其中我将进程的兄弟进程维护成一个循环链表。进程的子进程节点指向某个子进程，通过一个子进程就可以找到其他全部的子进程。

资源共有4类，分别为R1,R2,R3,R4。每类资源都有一个RCB，其中存放资源的总个数、空闲个数和阻塞在该资源上的进程链表。

调度是基于优先级的调度，优先级越高就越优先调度。优先级有0、1、2三个。0是最低的优先级，只有init进程优先级为0，其他进程优先级不能为0，这样才能保证init进程永远不会被调度。


最后，实现了shell界面。命令有：

'cr x 1'创建进程x，优先级为1

'de x'删除进程x及其所有子进程

'req R2 3'为当前进程申请资源R2，数量为3个

'rel R2 3'当前进程释放3个R2资源

'to'时钟中断

'ls -p'查看当前所有已经被创建的进程的信息

'ls -r'查看当前所有资源的信息

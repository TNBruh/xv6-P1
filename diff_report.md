# Modified Code

### defs.h

struct buf;
struct context;
struct file;
struct inode;
struct pipe;
struct proc;
struct rtcdate;
struct spinlock;
struct sleeplock;
struct stat;
struct superblock;
#ifdef CS333_P2
struct uproc;
#endif

//PAGEBREAK: 16
// proc.c
int             cpuid(void);
void            exit(void);
int             fork(void);
int             growproc(int);
int             kill(int);
struct cpu*     mycpu(void);
struct proc*    myproc();
void            pinit(void);
void            procdump(void);
void            scheduler(void) __attribute__((noreturn));
void            sched(void);
void            setproc(struct proc*);
void            sleep(void*, struct spinlock*);
void            userinit(void);
int             wait(void);
void            wakeup(void*);
void            yield(void);
#ifdef CS333_P2
int             getprocs(uint, struct uproc*);
#endif

### Makefile

CS333_PROJECT ?= 2

### proc.c

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"
#include "uproc.h"

static struct proc*
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);
  int found = 0;
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == UNUSED) {
      found = 1;
      break;
    }
  if (!found) {
    release(&ptable.lock);
    return 0;
  }
  p->state = EMBRYO;
  p->pid = nextpid++;
  release(&ptable.lock);

  // Allocate kernel stack.
  if((p->kstack = kalloc()) == 0){
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;

  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe*)sp;

  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint*)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context*)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;

 #ifdef CS333_P1
 p->start_ticks = ticks;
 #endif //CS333_P1
 #ifdef CS333_P2
 p->cpu_ticks_total = 0;
 p->cpu_ticks_in = 0;
 #endif

  return p;
}

void
userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];

  p = allocproc();

  initproc = p;
  if((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0;  // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  // this assignment to p->state lets other cores
  // run this process. the acquire forces the above
  // writes to be visible, and the lock is also needed
  // because the assignment might not be atomic.
  acquire(&ptable.lock);
  p->state = RUNNABLE;
  release(&ptable.lock);
  
  #ifdef CS333_P2
  p->uid = DEFUID;
  p->gid = DEFGID;
  #endif
}

int
fork(void)
{
  int i;
  uint pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy process state from proc.
  if((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0){
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;
  
  #ifdef CS333_P2
  np->uid = curproc->uid;
  np->gid = curproc->gid;
  #endif

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for(i = 0; i < NOFILE; i++)
    if(curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);
  np->state = RUNNABLE;
  release(&ptable.lock);

  return pid;
}

void
scheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();
  c->proc = 0;
#ifdef PDX_XV6
  int idle;  // for checking if processor is idle
#endif // PDX_XV6

  for(;;){
    // Enable interrupts on this processor.
    sti();

#ifdef PDX_XV6
    idle = 1;  // assume idle unless we schedule a process
#endif // PDX_XV6
    // Loop over process table looking for process to run.
    acquire(&ptable.lock);
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->state != RUNNABLE)
        continue;

      // Switch to chosen process.  It is the process's job
      // to release ptable.lock and then reacquire it
      // before jumping back to us.
#ifdef PDX_XV6
      idle = 0;  // not idle this timeslice
#endif // PDX_XV6
      c->proc = p;
      switchuvm(p);
      p->state = RUNNING;
      #ifdef CS333_P2
      p->cpu_ticks_in = ticks;
      #endif
      swtch(&(c->scheduler), p->context);
      switchkvm();

      // Process is done running for now.
      // It should have changed its p->state before coming back.
      c->proc = 0;
    }
    release(&ptable.lock);
#ifdef PDX_XV6
    // if idle, wait for next interrupt
    if (idle) {
      sti();
      hlt();
    }
#endif // PDX_XV6
  }
}

void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&ptable.lock))
    panic("sched ptable.lock");
  if(mycpu()->ncli != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(readeflags()&FL_IF)
    panic("sched interruptible");
  intena = mycpu()->intena;
  #ifdef CS333_P2
  p->cpu_ticks_total += ticks - p->cpu_ticks_in;
  #endif
  swtch(&p->context, mycpu()->scheduler);
  mycpu()->intena = intena;
}

#if defined(CS333_P2)
void
procdumpP2P3P4(struct proc *p, char *state_string)
{
 int ppid;
 uint elapsed = ticks - (p->start_ticks);
 uint pc[10];
 int cpu = p->cpu_ticks_total;
 
 if (p->parent)
 {
  ppid = p->parent->pid;
 }
 else
 {
  ppid = p->pid;
 }
 
 getcallerpcs((uint*)p->context->ebp+2, pc);
 
 cprintf("\n%d \t%s \t%d \t%d \t%d \t%d \t%d \t%s \t%d \t", p->pid, p->name, p->uid, p->gid, ppid, elapsed, cpu, state_string, p->sz);
 
 int i;
 for (i = 0; i < 10 && pc[i] != 0; i++)
 {
  cprintf(" %p", pc[i]);
 }
 
 cprintf("\n$ ");
 
  return;
}

#ifdef CS333_P2
int getprocs(uint max, struct uproc* table)
{
 struct proc * p;
  int count;

  count = 0;
  p = ptable.proc;

  acquire(&ptable.lock);
  while(p < &ptable.proc[NPROC] && count < max) {
    if(p->state != UNUSED && p->state != EMBRYO) {
      ++count;
      table->pid = p->pid;
      table->uid = p->uid;
      table->gid = p->gid;
      if(p->parent)
        table->ppid = p->parent->pid;
      else
        table->ppid = p->pid;
      table->elapsed_ticks = ticks - p->start_ticks;
      table->CPU_total_ticks = p->cpu_ticks_total;
      safestrcpy(table->state, states[p->state], STRMAX);
      table->size = p->sz;
      safestrcpy(table->name, p->name, STRMAX);
      #ifdef CS333_P3P4
      table->priority = p->priority;
      #endif
      ++table;
    }
    ++p;
  }
  release(&ptable.lock);

  return count;
}
#endif

### proc.h

extern struct cpu *cpu asm("%gs:0");       // &cpus[cpunum()]
extern struct proc *proc asm("%gs:4");     // cpus[cpunum()].proc

struct proc {
  uint sz;                     // Size of process memory (bytes)
  pde_t* pgdir;                // Page table
  char *kstack;                // Bottom of kernel stack for this process
  enum procstate state;        // Process state
  uint pid;                    // Process ID
  struct proc *parent;         // Parent process. NULL indicates no parent
  struct trapframe *tf;        // Trap frame for current syscall
  struct context *context;     // swtch() here to run process
  void *chan;                  // If non-zero, sleeping on chan
  int killed;                  // If non-zero, have been killed
  struct file *ofile[NOFILE];  // Open files
  struct inode *cwd;           // Current directory
  char name[16];               // Process name (debugging)
  #ifdef CS333_P1
  uint start_ticks;
  #endif //CS333_P1
  #ifdef CS333_P2
  uint uid;
  uint gid;
  uint cpu_ticks_total;
  uint cpu_ticks_in;
  #endif
};

### ps.c

#ifdef CS333_P2
#include "types.h"
#include "user.h"
#include "uproc.h"

int
main(void)
{
 uint max = 16;
  struct uproc* table;
  int count;
  int elapsed;
  int millisec;
  int cpu;
  int cpu_millisec;

  table = malloc(sizeof(struct uproc) * max);
  count = getprocs(max, table);

  if(count < 0)
    printf(2, "E\n");
  else {
    printf(1, "\nPID\tName\t\tUID\tGID\tPPID\tElapsed\tCPU\tState\tSize\n");

    for(int i = 0; i < count; ++i) {
      elapsed = table[i].elapsed_ticks;
      millisec = elapsed % 1000;
      elapsed = elapsed/1000;
      cpu = table[i].CPU_total_ticks;
      cpu_millisec = cpu % 1000;
      cpu = cpu/1000;

      printf(1, "%d\t%s\t", table[i].pid, table[i].name);
      if (strlen(table[i].name) < 7)
      
      printf(1, "%d\t%d\t%d\t%d.", table[i].uid, table[i].gid, table[i].ppid, elapsed);
      
      if (millisec < 10)
        printf(1, "00");
      else if (millisec < 100 && millisec >= 10)
        printf(1, "0");  
      printf(1, "%d\t%d.", millisec, cpu);
                  
      if (cpu_millisec == 0)
        printf(1, "000");
      else if (cpu_millisec < 10 && cpu_millisec > 0)
        printf(1, "00");
      else if (cpu_millisec < 100 && cpu_millisec >= 10)
        printf(1, "0");  
      printf(1, "%d\t%s\t%d\n", cpu_millisec, table[i].state, table[i].size);
    }
  }

  free(table);
  exit();
}
#endif

### syscall.c

extern int sys_chdir(void);
extern int sys_close(void);
extern int sys_dup(void);
extern int sys_exec(void);
extern int sys_exit(void);
extern int sys_fork(void);
extern int sys_fstat(void);
extern int sys_getpid(void);
extern int sys_kill(void);
extern int sys_link(void);
extern int sys_mkdir(void);
extern int sys_mknod(void);
extern int sys_open(void);
extern int sys_pipe(void);
extern int sys_read(void);
extern int sys_sbrk(void);
extern int sys_sleep(void);
extern int sys_unlink(void);
extern int sys_wait(void);
extern int sys_write(void);
extern int sys_uptime(void);
#ifdef PDX_XV6
extern int sys_halt(void);
#endif // PDX_XV6
#ifdef CS333_P1
extern int sys_date(void);
#endif
#ifdef CS333_P2
extern int sys_getuid(void);
extern int sys_getgid(void);
extern int sys_getppid(void);
extern int sys_setuid(void);
extern int sys_setgid(void);
extern int sys_getprocs(void);
#endif

static int (*syscalls[])(void) = {
[SYS_fork]    sys_fork,
[SYS_exit]    sys_exit,
[SYS_wait]    sys_wait,
[SYS_pipe]    sys_pipe,
[SYS_read]    sys_read,
[SYS_kill]    sys_kill,
[SYS_exec]    sys_exec,
[SYS_fstat]   sys_fstat,
[SYS_chdir]   sys_chdir,
[SYS_dup]     sys_dup,
[SYS_getpid]  sys_getpid,
[SYS_sbrk]    sys_sbrk,
[SYS_sleep]   sys_sleep,
[SYS_uptime]  sys_uptime,
[SYS_open]    sys_open,
[SYS_write]   sys_write,
[SYS_mknod]   sys_mknod,
[SYS_unlink]  sys_unlink,
[SYS_link]    sys_link,
[SYS_mkdir]   sys_mkdir,
[SYS_close]   sys_close,
#ifdef PDX_XV6
[SYS_halt]    sys_halt,
#endif // PDX_XV6
#ifdef CS333_P1
[SYS_date] sys_date,
#endif
#ifdef CS333_P2
[SYS_getuid] sys_getuid,
[SYS_getgid] sys_getgid,
[SYS_getppid] sys_getppid,
[SYS_setuid] sys_setuid,
[SYS_setgid] sys_setgid,
[SYS_getprocs] sys_getprocs,
#endif
};

### syscall.h

#define SYS_fork    1
#define SYS_exit    SYS_fork+1
#define SYS_wait    SYS_exit+1
#define SYS_pipe    SYS_wait+1
#define SYS_read    SYS_pipe+1
#define SYS_kill    SYS_read+1
#define SYS_exec    SYS_kill+1
#define SYS_fstat   SYS_exec+1
#define SYS_chdir   SYS_fstat+1
#define SYS_dup     SYS_chdir+1
#define SYS_getpid  SYS_dup+1
#define SYS_sbrk    SYS_getpid+1
#define SYS_sleep   SYS_sbrk+1
#define SYS_uptime  SYS_sleep+1
#define SYS_open    SYS_uptime+1
#define SYS_write   SYS_open+1
#define SYS_mknod   SYS_write+1
#define SYS_unlink  SYS_mknod+1
#define SYS_link    SYS_unlink+1
#define SYS_mkdir   SYS_link+1
#define SYS_close   SYS_mkdir+1
#define SYS_halt    SYS_close+1
#define SYS_date SYS_halt+1
#define SYS_getuid SYS_date+1
#define SYS_getgid SYS_getuid+1
#define SYS_getppid SYS_getgid+1
#define SYS_setuid SYS_getppid+1
#define SYS_setgid SYS_setuid+1
#define SYS_getprocs SYS_setgid+1

### sysproc.c

#ifdef CS333_P2
#include "uproc.h"
#endif

#ifdef CS333_P2
uint sys_getuid(void)
{
 return myproc()->uid;
}

uint sys_getgid(void)
{
 return myproc()->gid;
}

uint sys_getppid(void)
{
 if(!myproc()->parent)
 {
  return myproc()->pid;
 }
 return myproc()->parent->pid;
}

int sys_setuid(void)
{
 int uid;
 
 if(argint(0, &uid) < 0)
 {
  return -1;
 }
 if(uid < 0 || uid > 32767)
 {
  return -1;
 }
 myproc()->uid = uid;
 return 0;
}

int sys_setgid(void)
{
 int gid;
 
 if(argint(0, &gid) < 0)
 {
  return -1;
 }
 if (gid < 0 || gid > 32767)
 {
  return -1;
 }
 myproc()->gid = gid;
 return 0;
}

int sys_getprocs(void)
{
 int max;
 struct uproc* table;
 
 if(argint(0, &max) < 0)
 {
  return -1;
 }
 if(argptr(1, (void*) &table, sizeof(struct uproc) * max) < 0)
 {
  return -1;
 }
 return getprocs(max, table);
}

#endif

### time.c

#ifdef CS333_P2
#include "types.h"
#include "user.h"
int
main(int argc, char* argv[])
{
  int run_time;
  int start;
  int millisec;
  int pid;

  start = uptime();
  pid = fork();
  if (pid < 0) {
    printf(2, "Fork error\n");
    exit();
  }
  else if (pid == 0) {
    exec(argv[1], &argv[1]);
    exit();
  }
  else {
    wait();
    printf(1, "%s ran in ", argv[1]);
    run_time = uptime() - start;
    millisec = run_time % 1000;
    run_time = run_time/1000;
    printf(1, "%d.", run_time);
    if (millisec < 10)
    {
     printf(1, "00");
    }
    else if (millisec < 100 && millisec >= 10)
    {
     printf(1, "0");  

     printf(1, "%d seconds.\n", millisec);
    }
      
  }
  exit();
}
#endif

### user.h

// system calls
int fork(void);
int exit(void) __attribute__((noreturn));
int wait(void);
int pipe(int*);
int write(int, void*, int);
int read(int, void*, int);
int close(int);
int kill(int);
int exec(char*, char**);
int open(char*, int);
int mknod(char*, short, short);
int unlink(char*);
int fstat(int fd, struct stat*);
int link(char*, char*);
int mkdir(char*);
int chdir(char*);
int dup(int);
int getpid(void);
char* sbrk(int);
int sleep(int);
int uptime(void);
int halt(void);
#ifdef CS333_P1
int date(struct rtcdate*);
#endif
#ifdef CS333_P2
uint getuid(void);
uint getgid(void);
uint getppid(void);
int setuid(uint);
int setgid(uint);
int getprocs(uint, struct uproc*);
#endif

### usys.S

SYSCALL(fork)
SYSCALL(exit)
SYSCALL(wait)
SYSCALL(pipe)
SYSCALL(read)
SYSCALL(write)
SYSCALL(close)
SYSCALL(kill)
SYSCALL(exec)
SYSCALL(open)
SYSCALL(mknod)
SYSCALL(unlink)
SYSCALL(fstat)
SYSCALL(link)
SYSCALL(mkdir)
SYSCALL(chdir)
SYSCALL(dup)
SYSCALL(getpid)
SYSCALL(sbrk)
SYSCALL(sleep)
SYSCALL(uptime)
SYSCALL(halt)
SYSCALL(date)
SYSCALL(getuid)
SYSCALL(getgid)
SYSCALL(getppid)
SYSCALL(setuid)
SYSCALL(setgid)
SYSCALL(getprocs)
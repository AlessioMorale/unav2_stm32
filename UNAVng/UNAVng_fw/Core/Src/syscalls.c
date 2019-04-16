#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
caddr_t _sbrk_r(struct _reent *r, int incr){
    (void)incr;
    __errno_r(r) = ENOMEM;
    return (caddr_t)-1;
}

int _kill(int pid, int sig)
{
  errno = ENOSYS;
  return -1;
}

void _exit (int status)
{
    (void)status;
}
int _getpid (int n)
{
    (void)n;
    return 1;
}
caddr_t _sbrk (int incr){
    (void)incr;
    return (caddr_t) -1;
}
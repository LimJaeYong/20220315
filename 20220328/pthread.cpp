#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void*), void *arg);
/*
 쓰레드 생성
thread 는 쓰레드가 생성되었을때 쓰레드를 식별하는 식별자
attr은 쓰레드 특성을 지정할 떄 사용
start_routine은 분기시켜 사용하는 쓰레드의 함수
arg는 쓰레드 함수 인자 
생성된 쓰레드는 반드시 사용자원을 반납해야함
*/

int pthread_join(pthread_t th, void **thread_return);
/* 
쓰레드 반환에 사용
th는 쓰레드 식별자 
thread_return은 쓰레드 리턴, NULL이 아닐 경우 포인터로 리턴값을 받아올 수 있음
*/

int pthread_detach(pthread_t th);
/* 
pthread_create로 생성된 쓰레드를 분리 시킬 떄 사용
식별자가 th인 쓰레드를 분리
분리된 쓰레드는 쓰레드가 종료되면 pthread_join을 사용하지 않아도 free됨
*/

void pthread_exit(void *retval);
/*
실행중인 쓰레드를 종료시킬때 사용
pthread_cleanup_push가 정의 되었을 때는 cleanup handler가 호출됨
*/

void pthread_cleanup_push(void (*routine)(void*), void *arg);
/*
cleanup handler를 등록하기 위해 사용
쓰레드를 종료할떄 routine함수를 호출
arg는 routine함수에 넘겨지는 매개변수
cleanup handler는 자원 반납, mutex 잠금 해제등에 사용
*/

void pthread_cleanup_pop(int execute);
/*
cleanup handler 제거에 사용
execute 인자가 0일때 cleanup handler 실행 시키지 않고 삭제
0이 아닐때 실행 시키고 삭제
반드시 push와 pop은 같은 함수내의 같은 레벨블럭에 사용
*/

pthread_t pthread_self(void);
/*
pthread_self를 호출하는 쓰레드의 식별자 반환
*/

int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutex_attr *attr);
//mutex는 여러개의 쓰레드가 공유하는 데이터를 보호하기 위해서 사용되는 도구로 한번에 한 쓰레드가 데이터를 독점하는식으로 보호
//사용할때 mutex 객체 선언 후 사용
/*
init은 mutex를 초기화할때 사용
첫번째 인자는 mutex 객체인 mutex를 초기화
두번쨰 인자는 mutex 특성 변경에 사용
기본 특성은 NULL사용
특성에는 fast, recursive, error checking 이 있고 주로 fast를 사용
*/

int pthread_mutex_destory(pthread_mutex_t *mutex);
/*
mutex 객체 제거
객체를 제거하기 위해 반드시 unlock상태가 되어야 함
*/

int pthread_mutex_lock(pthread_mutex_t *mutex);
/*
mutex가 임계영역에 들어가기 위해 lock을 요청
먼저 lock된 mutex가 있으면 선진입한 mutex가 unlock될 때까지 대기
*/

int pthread_mutex_unlock(pthread_mutex_t *mutex);
/*
임계영역을 빠져나온 mutex의 lock을 해제
*/

int pthread_cond_init(pthread_cond_t *cond, const pthread_cond_attr *attr);
/*
조건변수 cond 초기화를 위해 사용
attr로 변수의 특성 변경 가능, 기본 NULL
*/

int pthread_cond_signal(pthread_cond_t *cond);
/*
조건변수에 시그널을 보냄
cond에서 기다리는 쓰레드가 있으면 쓰래드를 깨움
*/

int pthread_cond_broadcast(pthread_cond_t *cond);
/*
cond에서 기다리는 쓰레드들에게 broadcast
*/

int pthread_cond_wait(pthread_cond_t cond, pthread_mutex_t *mutex);
/*
cond를 통해 신호가 전달될때까지 블록
블록되기전에 mutex잠금을 되돌려줌
*/

int pthread_cond_timewait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *abstime);
/*
pthread_cond_wait와 동일하지만 시간체크가 가능
abstime시간동안 신호가 도착하지 않으면 error를 띄움
리턴으로 에러가 넘어오는 것에 주의
다른 신호에 의해 인터럽트 될 수있기때문에 대책이 필요
*/

int pthread_cond_destroy(pthread_cond_t *cond);
/*
cond에 대한 자원 해제
함수호출 전에 어떤 쓰레드도 신호를 기다리고 있지 않는것을 확인해야함
*/

int pthread_attr_init(pthread_attr_t *attr);
/*
attr을 디폴트로 초기화
성공하면 0, 실패히면 -1을 반환
*/

int pthread_attr_destroy(const pthread_attr_t * attr, int *scope);
/*
attr 제거
재사용하려면 init으로 재생성해줘야 함
*/

int pthread_attr_getscope(const pthread_attr_t *attr, int *scope);
/*
쓰레드가 어떤 영역에서 다루어지고 있는지 얻어오기 위해 사용
PTHREAD_SCOPE_SYSTEM(user)과 PTHREAD_SCOPE_PROCESS(kernel) 2가지 영역 선택가능
리눅스에서는 커널에서 쓰레드 스케줄링을 하지 않고 쓰레드 라이브러리를 통해 쓰레드를 스케줄링 하는 방식
*/

int phtread_attr_setscope(pthread_attr_t *attr, int scope);
/*
쓰레드의 작동 영역을 지정
리눅스에서는 커널모드를 지원하지 않으므로 PTHREAD_SCOPE_SYSTEM만 선택 가능
*/

int pthread_attr_getdetachstate(pthread_attr_t *attr, int detachstate);
/*
쓰레드가 join 가능한 상태인지 detach 상태인지 알아냄
알아낸 값은 detachstate에 저장
기본은 PTHREAD_CREAT_JOINABLE이고 생성된 쓰레드를 detach 상태로 만들었을떄, 
pthread_attr_setdetachstate를 이용해 쓰레드를 detach 상태로 만들었을 경우 PTHREAD_CREAT_DETACHED 상태가 됨
*/

int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate);
/*
쓰레드 상태 변경을위해 사용
*/

int pthread_sigmask(int how, const sigset_t *newmask, sigst_t *oldmask);
/*
쓰레드에서 시그널은 서로 공유되기 때문에 프로세스에 시그널이 전달되면 프로세스가 생성된 모든 쓰레드로 시그널을 전달시킴
특정 쓰레드에만 시그널을 받게하기 위해 사용하는 함수
*/

int pthread_kill(pthread_t thread, int signo);
/*
쓰레드 식별자 thread로 signo 숫자를 전달
*/

int sigwait(const sigset_t *set, int *sig);
/*
시그널 전달을 동기적을 기다림
*/

//쓰레드 취소 함수
int pthread_cancel(pthread_t thread);

int pthread_setcancelstate(int state, int *oldstate);

int pthread_setcancelstate(int state, int *oldstate);

int pthread_setcanceltype(int type, int *oldtype);

void pthread_testcancel(void);
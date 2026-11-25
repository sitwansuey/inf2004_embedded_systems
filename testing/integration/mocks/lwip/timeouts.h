#ifndef TEST_MOCK_LWIP_TIMEOUTS_H
#define TEST_MOCK_LWIP_TIMEOUTS_H

void sys_timeout (unsigned ms, void (*cb)(void *), void * arg);
void sys_untimeout (void (*cb)(void *), void * arg);

#endif /* TEST_MOCK_LWIP_TIMEOUTS_H */

/*** end of file ***/

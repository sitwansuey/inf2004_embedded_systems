#ifndef IP_TEST_MOCK_H
#define IP_TEST_MOCK_H
typedef struct {unsigned value;} ip_addr_t;
#define ip_2_ip4(x) (x)
#define IP_SET_TYPE(x,y) ((void)(x),(void)(y))
#define IPADDR_TYPE_V4 0
int ip4addr_aton(const char *s,ip_addr_t *ip);
#endif

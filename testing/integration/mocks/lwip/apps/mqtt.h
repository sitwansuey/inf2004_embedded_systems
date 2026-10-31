#ifndef MQTT_TEST_MOCK_H
#define MQTT_TEST_MOCK_H
#include <stdint.h>
typedef uint8_t u8_t;typedef uint16_t u16_t;typedef uint32_t u32_t;typedef int err_t;
enum {ERR_OK=0,ERR_MEM=-1,MQTT_DATA_FLAG_LAST=1,MQTT_CONNECT_ACCEPTED=0};
typedef int mqtt_connection_status_t;
typedef struct {int connected;} mqtt_client_t;
struct mqtt_connect_client_info_t {const char *client_id;unsigned keep_alive;const char *client_user,*client_pass;};
typedef void (*done_t)(void *,err_t);
mqtt_client_t *mqtt_client_new(void);
int mqtt_client_is_connected(mqtt_client_t *c);
err_t mqtt_client_connect(mqtt_client_t *c,const void *ip,unsigned port,void (*cb)(mqtt_client_t *,void *,mqtt_connection_status_t),void *arg,const struct mqtt_connect_client_info_t *info);
err_t mqtt_publish(mqtt_client_t *c,const char *topic,const void *data,u16_t size,unsigned qos,unsigned retain,done_t cb,void *arg);
err_t mqtt_subscribe(mqtt_client_t *c,const char *topic,unsigned qos,done_t cb,void *arg);
void mqtt_set_inpub_callback(mqtt_client_t *c,void (*start)(void *,const char *,u32_t),void (*data)(void *,const u8_t *,u16_t,u8_t),void *arg);
#endif

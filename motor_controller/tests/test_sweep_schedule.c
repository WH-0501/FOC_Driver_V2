/* Execute actual foreground scheduling/send functions extracted by the runner. */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define COMM_FRAME_MAX_LEN 64
typedef struct {uint32_t id;uint8_t len;bool bulk;const uint8_t *data;} comm_frame_t;
typedef struct {bool (*tx_ready)(void *,bool);} comm_transport_ops_t;
static const comm_transport_ops_t *s_transport;
static bool (*s_send)(const comm_frame_t *);
static uint8_t s_tx_cnt,s_tx_buf[64],s_reply_count,s_debug_mask;
static uint16_t s_report_clock,s_node_id=1;
static uint32_t s_report_elapsed,s_report_interval_us=1000;
static void *s_motor;
static bool active,record=true,ready=true;
static unsigned queued;
static uint32_t now,reports,debug,polls,flushes,wire;
static bool is_ready(void *ctx,bool bulk){(void)ctx;(void)bulk;return ready;}
static bool send_frame(const comm_frame_t *f){assert(f->len<=64 && f->bulk);++wire;return true;}
static bool foc_sweep_is_active(void *m){(void)m;return active;}
static bool sweep_record_enabled(void){return record;}
static uint32_t board_time_us(void){return now;}
static void can_fd_iap_poll(void){}
static void can_fd_debug_clock(void){}
static void can_fd_custom_flush_reply(void){++flushes;}
static void can_fd_custom_period_report(void){++reports;}
static void can_fd_debug_poll(void){++debug;}
static bool sweep_record_poll(uint32_t n,bool (*fn)(uint32_t,uint8_t,const uint8_t*,uint8_t))
{(void)n;(void)fn;++polls;if(queued){--queued;return true;}return false;}
static uint8_t can_fd_codec_pad_canfd_len(uint8_t n){return n<=8?8:64;}
#include "schedule_protocol.inc"
static void interval(void){unsigned i;for(i=0;i<20000;++i){now+=50;can_fd_custom_poll();}}
int main(void){
 comm_transport_ops_t ops={is_ready};uint8_t p[3]={1,2,3};
 assert(!comm_tx_ready(false));s_transport=&ops;assert(comm_tx_ready(false));
 ready=false;s_send=send_frame;memset(s_tx_buf,0xa5,sizeof(s_tx_buf));
 assert(!can_fd_custom_send_frame(1,0xbf,p,3));assert(!wire&&!s_tx_cnt&&s_tx_buf[0]==0xa5);
 ready=true;assert(can_fd_custom_send_frame(1,0xbf,p,3));assert(wire==1&&s_tx_cnt==1);
 assert(s_tx_buf[1]==0xbf&&!memcmp(s_tx_buf+2,p,3)&&!s_tx_buf[5]&&!s_tx_buf[7]);
 ops.tx_ready=NULL;assert(comm_tx_ready(false));
 active=true;interval();assert(reports==20&&debug==0&&polls==20000&&s_report_interval_us==1000);
 active=false;reports=debug=0;interval();assert(reports==1000&&debug==20000);
 active=true;s_debug_mask=1;reports=debug=0;interval();assert(reports==5&&!debug);
 s_debug_mask=0;s_report_interval_us=0;reports=debug=0;interval();assert(!reports&&!debug);
 s_reply_count=1;polls=0;interval();assert(!polls&&flushes==100000);
 s_reply_count=0;polls=0;queued=10;can_fd_custom_poll();assert(polls==3&&queued==7);
 polls=0;queued=1;can_fd_custom_poll();assert(polls==2&&!queued);
 puts("PASS bounded upload: 3-frame budget, stop immediately on empty/backpressure");
 puts("PASS foreground: busy before packing, byte payload/padding, 20 Hz supervision, debug pause/restore, disabled telemetry, reply priority");
}

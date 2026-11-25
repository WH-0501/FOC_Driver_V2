/* Execute the real transport functions with a deterministic CAN hardware model.
 * Scheduling windows below are synthetic, NOT measured MCU execution timings. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "comm_transport.h"
#include "sweep_record.h"
#define SUPPORT_CAN_FD
#define SUCCESS 1
#define ERROR 0
#define CAN_STB_STATUS_EMPTY 0
#define CAN_STB_STATUS_FULL 3
#define CAN_DLC_BYTES_64 15
#define CAN_ID_STANDARD 0
#define CAN_FRAME_DATA 0
#define CAN_FORMAT_FD 1
#define CAN_BRS_ON 1
#define CAN_TXBUF_PTB 0
#define CAN_TXBUF_STB 1
#define CAN_TRANSMIT_PTB 0x10
#define CAN_TRANSMIT_STB_ALL 2
typedef unsigned can_data_length_type;
typedef struct {uint32_t id;unsigned id_type,frame_type,data_length,fd_format,fd_rate_switch;uint8_t data[64];unsigned tx_timestamp,handle;} can_txbuf_type;
#define TRUE 1
#define CAN_ID_EXTENDED 1
typedef unsigned can_txbuf_select_type;
typedef int error_status;
static const uint8_t dlc_to_bytes[16]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
typedef struct {
 union {uint32_t ctrlstat;struct {
  unsigned busoff:1,reserved1:4,lbmi:1,lbme:1,reset:1;
  unsigned tsa:1,tsall:1,tsone:1,tpa:1,tpe:1,stby:1,lom:1,tbsel:1;
  unsigned tsstat:2,tsff:1,reserved2:1,tttbm:1,tsmode:1,tsnext:1,fd_iso:1;
  unsigned rstat:2,reserved3:1,rball:1,rrel:1,rov:1,rom:1,sack:1;
 } ctrlstat_bit;};
 uint32_t tbid;
 union {uint32_t tbfmt;struct {unsigned dlc:4,reserved1:12,ide:1,fdf:1,brs:1,reserved2:1,rmf:1,reserved3:11;} tbfmt_bit;};
 union {uint32_t tbtyp;struct {unsigned reserved:24,handle:8;} tbtyp_bit;};
 uint32_t reserved8,tbdat[16];
} can_type;
static can_type hardware;
#define CAN1 (&hardware)
static can_txbuf_type fifo[3],pending,primary;
static unsigned head,tail,depth,writes,kicks,held_commit,delivered,next_sample,control_frames;
static bool streaming,legacy;
void __DMB(void) {}
static unsigned read32(const uint8_t *p){return (unsigned)p[0]<<24|(unsigned)p[1]<<16|(unsigned)p[2]<<8|p[3];}
static void receive(const can_txbuf_type *f){
 if(!streaming)return;
 unsigned n=f->data[1],i,index=read32(f->data+4);
 assert(n && n<=6 && index>=next_sample);
 for(i=0;i<n;++i){
   float x=(float)(index+i),y=-x;unsigned bx,by;
   memcpy(&bx,&x,4);memcpy(&by,&y,4);
   assert(read32(f->data+8+i*8)==bx && read32(f->data+12+i*8)==by);
 }
 delivered+=n;next_sample=index+n;
}
static void commit(void){
 assert(CAN1->ctrlstat_bit.tsnext && depth<3);
 fifo[head++%3]=pending;++depth;
 CAN1->ctrlstat_bit.tsnext=0;CAN1->ctrlstat_bit.tsstat=depth;
}
#include "sweep_vendor_write.inc"
static can_txbuf_type snapshot(void){
 can_txbuf_type f={0};f.id=CAN1->tbid>>18;f.data_length=CAN1->tbfmt_bit.dlc;
 f.fd_format=CAN1->tbfmt_bit.fdf;f.fd_rate_switch=CAN1->tbfmt_bit.brs;
 memcpy(f.data,CAN1->tbdat,64);return f;
}
static int can_txbuf_transmit(can_type *c,unsigned type){
 if(c->ctrlstat_bit.stby)return ERROR;
 if(type==CAN_TRANSMIT_PTB){if(c->ctrlstat_bit.tpe||c->ctrlstat_bit.tpa)return ERROR;primary=snapshot();c->ctrlstat_bit.tpe=1;return SUCCESS;}
 assert(type==CAN_TRANSMIT_STB_ALL);
 if(!depth||c->ctrlstat_bit.tsall||c->ctrlstat_bit.tsone||c->ctrlstat_bit.tsa)return ERROR;
 c->ctrlstat_bit.tsall=1;++kicks;return SUCCESS;
}
#include "sweep_transport.inc"
static bool model_send(void *ctx,const comm_frame_t *frame){
 /* Model write-one control bits: writing zero does not cancel an in-flight
  * PTB/STB transmission. memcpy/register parity below tests the actual bytes. */
 uint32_t pending_bits=CAN1->ctrlstat&0x1f00u;
 bool ok=can_port_at32_send(ctx,frame);
 CAN1->ctrlstat|=pending_bits;
 if(ok){++writes;if(frame->bulk){pending=snapshot();if(!held_commit)commit();}}
 return ok;
}
static void reset(void){
 memset(&hardware,0,sizeof(hardware));head=tail=depth=writes=kicks=held_commit=delivered=next_sample=control_frames=0;streaming=false;
}
static bool send_sample(uint32_t node,uint8_t cmd,const uint8_t *p,uint8_t len){
 comm_frame_t f={node,len,!legacy,p};assert(cmd==0xbf);return model_send(NULL,&f);
}
static void scheduling_model(bool old,unsigned stall_us){
 reset();legacy=old;streaming=true;sweep_record_begin(0x1234,true);
 unsigned t,bus_remaining=0;bool in_primary=false;unsigned accepted=0;
 /* 20 kHz producer, foreground opportunity every 200 us, 220 us bulk frame.
  * Interrupt/foreground spacing exercises overlap, not a bus-speed benchmark. */
 for(t=0;t<4010000;t+=10){
   if(t<4000000 && t%50==0)sweep_record_push(t/50,(float)(t/50),-(float)(t/50));
   if(t==4000000)sweep_record_end();
   if(bus_remaining){bus_remaining-=10;if(!bus_remaining){
     if(in_primary){if(primary.id==1)receive(&primary);else ++control_frames;CAN1->ctrlstat_bit.tpe=0;}
     else {assert(depth);receive(&fifo[tail++%3]);--depth;CAN1->ctrlstat_bit.tsstat=depth;if(!depth)CAN1->ctrlstat_bit.tsall=0;}
   }}
   if(t%200==0 && !(t>=2000000 && t<2000000+stall_us)){
     can_port_at32_kick_bulk();
     for(unsigned b=0;b<3 && sweep_record_poll(1,send_sample);++b)++accepted;
     if(!old && t%50000==0 && !CAN1->ctrlstat_bit.tpe){uint8_t p=0;comm_frame_t f={2,1,false,&p};assert(model_send(NULL,&f));}
   }
   if(!bus_remaining){
     if(CAN1->ctrlstat_bit.tpe){in_primary=true;bus_remaining=primary.id==1?220:40;}
     else if(CAN1->ctrlstat_bit.tsall && depth){in_primary=false;bus_remaining=220;}
   }
 }
 assert(!depth&&!CAN1->ctrlstat_bit.tpe);
 assert(delivered==sweep_record_submitted());
 assert(delivered+sweep_record_dropped()==80000);
 if(!old&&!stall_us){assert(delivered==80000 && next_sample==80000 && control_frames>=80);}
 else assert(sweep_record_dropped()>0);
 printf("PASS synthetic %s, stall=%u us: received=%u/80000 dropped=%u, accepted frames=%u, control=%u\n",old?"single PTB":"STB pipeline",stall_us,delivered,sweep_record_dropped(),accepted,control_frames);
}
static void wire_parity(void){
 unsigned len,offset,bulk,id;
 uint8_t bytes[68];for(unsigned i=0;i<sizeof(bytes);++i)bytes[i]=(uint8_t)(i*13+17);
 for(bulk=0;bulk<2;++bulk)for(offset=0;offset<4;++offset)
 for(len=0;len<=64;++len)for(id=0;id<3;++id){
   reset();memset(CAN1->tbdat,0x7c,64);CAN1->tbtyp=CAN1->reserved8=~0u;
   CAN1->ctrlstat=0x80200060u;
   can_type vendor=*CAN1;can_txbuf_type tx={0};
   comm_frame_t f={id==0?0:id==1?0x7ff:0xabcde,(uint8_t)len,bulk!=0,bytes+offset};
   tx.id=f.id;tx.data_length=can_port_bytes_to_dlc(f.len);tx.fd_format=1;tx.fd_rate_switch=1;
   unsigned expected_dlc=0;while(dlc_to_bytes[expected_dlc]<len)++expected_dlc;
   assert(tx.data_length==expected_dlc);
   memcpy(tx.data,f.data,len);
   assert(can_txbuf_write(&vendor,bulk?CAN_TXBUF_STB:CAN_TXBUF_PTB,&tx)==SUCCESS);
   can_port_at32_write(&f);
   assert(CAN1->ctrlstat==vendor.ctrlstat && CAN1->tbid==vendor.tbid && CAN1->tbfmt==vendor.tbfmt && CAN1->tbtyp==vendor.tbtyp && CAN1->reserved8==vendor.reserved8);
   unsigned wire=(dlc_to_bytes[tx.data_length]+3u)&~3u;
   assert(!memcmp(CAN1->tbdat,vendor.tbdat,wire));
   for(unsigned i=len;i<wire;++i)assert(((uint8_t*)CAN1->tbdat)[i]==0);
 }
 puts("PASS 1560 register/payload comparisons with actual vendor writer: both buffers, all lengths 0..64, unaligned data, IDs, tails and DLC padding");
 reset();comm_frame_t f={1,65,true,bytes};assert(!model_send(NULL,&f)&&!writes);
 f.len=1;f.data=NULL;assert(!model_send(NULL,&f)&&!writes);
}
int main(void){
 wire_parity();
 uint8_t p[64];memset(p,0xa5,sizeof(p));comm_frame_t f={1,64,true,p};
 reset();CAN1->ctrlstat_bit.tpe=1;assert(can_port_at32_tx_ready(NULL,true));assert(!can_port_at32_tx_ready(NULL,false));
 for(unsigned i=0;i<3;++i)assert(model_send(NULL,&f));
 can_port_at32_kick_bulk();assert(depth==3&&writes==3&&kicks==1);assert(!model_send(NULL,&f)&&writes==3);
 assert(!memcmp(fifo[0].data,p,64));CAN1->ctrlstat_bit.tpe=0;f.bulk=false;assert(model_send(NULL,&f));assert(depth==3);
 reset();f.bulk=true;held_commit=1;assert(model_send(NULL,&f));assert(!depth&&writes==1&&!kicks);
 assert(!model_send(NULL,&f)&&writes==1);commit();can_port_at32_kick_bulk();assert(depth==1&&kicks==1&&writes==1);
 /* A commit arriving just after TSALL drained must get a fresh kick. */
 CAN1->ctrlstat_bit.tsall=0;can_port_at32_kick_bulk();assert(kicks==2);
 reset();CAN1->ctrlstat_bit.stby=1;assert(!model_send(NULL,&f)&&!writes);
 CAN1->ctrlstat_bit.stby=0;CAN1->ctrlstat_bit.tsa=1;assert(!model_send(NULL,&f)&&!writes);
 CAN1->ctrlstat_bit.tsa=0;CAN1->ctrlstat_bit.tpa=1;f.bulk=false;assert(!model_send(NULL,&f)&&!writes);
 assert(!model_send(NULL,NULL));
 puts("PASS transport: PTB/STB independence, FIFO full backpressure, one ownership transfer, delayed last commit, drain race, standby/abort guards");
 scheduling_model(true,0);scheduling_model(false,0);scheduling_model(false,2000);
 return 0;
}

"""Compile actual transport/IRQ functions against a sticky-RIF hardware FIFO model."""
from pathlib import Path
import argparse
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
p.add_argument('--build-dir', type=Path, required=True)
p.add_argument('--cc', default=r'C:/Qt/Tools/mingw1310_64/bin/gcc.exe')
p.add_argument('--reproduce-old', action='store_true')
a = p.parse_args()
port = (a.source_root/'project/APP/comm/transport/can_port_at32.c').read_text(encoding='utf-8-sig')
irq = (a.source_root/'project/src/at32m412_416_int.c').read_text(encoding='utf-8-sig')

def function(source, name):
    match = re.search(r'^(?:static )?[\w *]+\b'+name+r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    start = match.start()
    end = source.index('{', start)+1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]+'\n'

code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#define COMM_RX_QUEUE_DEPTH_MAX 64u
#define COMM_RX_QUEUE_DEPTH_DEFAULT 32u
#define COMM_FRAME_MAX_LEN 64u
#define CAN_PORT_RX_DMB() ((void)0)
#define CAN_PORT_RX_IRQ_BUDGET 6u
#define CAN_RXBUF_STATUS_EMPTY 0u
#define CAN_RIF_FLAG 0x80u
#define CAN_RAFIF_FLAG 0x10u
#define CAN_RFIF_FLAG 0x20u
#define CAN_ROIF_FLAG 0x40u
#define RESET 0
#define SUCCESS 1
#define CAN1_RX_IRQn 30
typedef unsigned can_data_length_type;
typedef struct { uint32_t id; can_data_length_type data_length; uint8_t data[64]; } can_rxbuf_type;
typedef struct { uint32_t id; uint8_t len; const uint8_t *data; } comm_frame_t;
typedef struct { unsigned flags; } fake_can;
static fake_can hardware;
#define CAN1 (&hardware)
static can_rxbuf_type fifo[64];
static unsigned first, last, reads, refill, pending_irq;
static uint8_t delivered[64];
static unsigned delivered_count;
static void arrival(uint8_t cnt) {
  assert(last-first<6); /* hardware capacity */
  can_rxbuf_type *f=&fifo[last++]; memset(f,0,sizeof(*f));
  f->id=1; f->data_length=3; f->data[0]=cnt; f->data[1]=0x40; f->data[2]=0;
  hardware.flags |= CAN_RIF_FLAG;
}
static unsigned can_rxbuf_status_get(fake_can *c) { (void)c;return last!=first; }
static int can_rxbuf_read(fake_can *c,can_rxbuf_type *out) {
  (void)c;if(first==last)return 0;
  *out=fifo[first++]; ++reads;
  if(refill){refill=0;arrival(0x77);}
  return SUCCESS;
}
static int can_interrupt_flag_get(fake_can *c,unsigned flag) {return (c->flags&flag)!=0;}
static void can_flag_clear(fake_can *c,unsigned flags) {c->flags &= ~flags;}
static void NVIC_SetPendingIRQ(int irq) {assert(irq==CAN1_RX_IRQn);pending_irq=1;}
static bool consume(const comm_frame_t *f,void *user) {
  (void)user; assert(f->id==1&&f->len==3); delivered[delivered_count++]=f->data[0]; return true;
}
'''
start = port.index('static can_rxbuf_type s_rx_pool')
end = port.index('static uint8_t can_port_dlc_to_bytes', start)
code += port[start:end]
for name in ('can_port_dlc_to_bytes','can_port_rx_acquire_slot','can_port_rx_commit_slot',
             'can_port_at32_rx_irq_handler','can_port_at32_poll_rx'):
    code += function(port, name)
code += 'static void comm_can_rx_isr(void) {can_port_at32_rx_irq_handler();}\n'
code += function(irq, 'CAN1_RX_IRQHandler')
code += r'''
static void reset_model(unsigned depth) {
  first=last=reads=refill=pending_irq=delivered_count=0;hardware.flags=0;
  memset(&s_rx_q,0,sizeof(s_rx_q));s_rx_depth=(uint8_t)depth;
}
static void dispatch(void) {pending_irq=0;CAN1_RX_IRQHandler();}
int main(int argc,char **argv) {
  (void)argv;
  reset_model(32);arrival(0x10);arrival(0x11);dispatch();
  can_port_at32_poll_rx(NULL,consume,NULL);
  if(argc>1) {
    assert(last-first==1&&delivered_count==1&&hardware.flags==0&&!pending_irq);
    arrival(0x12);dispatch();can_port_at32_poll_rx(NULL,consume,NULL);
    assert(last-first==1&&delivered_count==2&&delivered[1]==0x11);
    puts("REPRODUCED: original IRQ leaves one frame, next command releases only the previous frame");
    return 0;
  }
  assert(first==last&&delivered_count==2&&delivered[0]==0x10&&delivered[1]==0x11);
  puts("PASS two-frame burst fully consumed without another host command");
  reset_model(32);arrival(1);dispatch();
  assert(can_port_at32_poll_rx(NULL,consume,NULL)==1&&delivered[0]==1);
  dispatch();assert(reads==1&&s_rx_q.drop_count==0);
  puts("PASS single frame and spurious/empty interrupt");
  reset_model(32);for(unsigned i=0;i<6;i++)arrival((uint8_t)i);dispatch();
  assert(first==last&&can_port_at32_poll_rx(NULL,consume,NULL)==6);
  for(unsigned i=0;i<6;i++)assert(delivered[i]==i);
  puts("PASS full hardware FIFO, preserved order");
  reset_model(3);for(unsigned i=0;i<6;i++)arrival((uint8_t)i);dispatch();
  assert(first==last&&s_rx_q.drop_count==4);
  assert(can_port_at32_poll_rx(NULL,consume,NULL)==2);
  dispatch();assert(s_rx_q.drop_count==4);
  puts("PASS software overflow releases all hardware frames, no false empty-drop count");
  reset_model(32);for(unsigned i=0;i<6;i++)arrival((uint8_t)i);refill=1;dispatch();
  assert(reads==6&&last-first==1&&pending_irq);
  hardware.flags=0;dispatch(); /* NVIC continuation, no new hardware event */
  assert(reads==7&&first==last&&can_port_at32_poll_rx(NULL,consume,NULL)==7);
  assert(delivered[6]==0x77);
  puts("PASS bounded ISR resumes via NVIC without another bus frame");
  reset_model(32);arrival(2);refill=1;dispatch();
  assert(first==last&&(hardware.flags&CAN_RIF_FLAG));
  assert(can_port_at32_poll_rx(NULL,consume,NULL)==2);
  puts("PASS new receive flag during drain is not cleared afterwards");
  reset_model(4);
  for(unsigned i=0;i<20;i++){arrival((uint8_t)i);dispatch();assert(can_port_at32_poll_rx(NULL,consume,NULL)==1);}
  for(unsigned i=0;i<20;i++)assert(delivered[i]==i);
  puts("PASS software ring wraparound");
  return 0;
}
'''
w = a.build_dir.resolve()
w.mkdir(parents=True,exist_ok=True)
(w/'can_rx_fifo_test.c').write_text(code,encoding='utf-8')
subprocess.run([a.cc,'-std=c11','-O2','-Wall','-Wextra','-Wno-unused-function',
                str(w/'can_rx_fifo_test.c'),'-o',str(w/'can_rx_fifo_test.exe')],check=True)
subprocess.run([str(w/'can_rx_fifo_test.exe')]+(['old'] if a.reproduce_old else []),check=True)

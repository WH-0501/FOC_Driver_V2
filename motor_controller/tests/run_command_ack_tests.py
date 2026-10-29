"""Compile actual protocol reply queue/on_rx functions against a fake CAN port."""
from pathlib import Path
import argparse
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--build-dir', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
src = (root/'project/APP/comm/protocol/custom/can_fd_custom.c').read_text(encoding='utf-8')

def function(name):
    match = re.search(r'^static (?:void|bool) '+name+r'\([^;]*?\)\s*\{', src, re.M)
    start = match.start()
    end = src.index('{', start)+1
    depth = 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[start:end]+'\n'

code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define COMM_FRAME_MAX_LEN 64u
#define CAN_FD_IAP_ID 0x7FFu
#define CAN_FD_CMD_IAP_ENTER 0xF8u
#define CAN_FD_CMD_VERIFY_ELECTRICAL 0xB1u
#define CAN_FD_CMD_GET_SPEED_LIMITS 0xB5u
#define CAN_FD_CMD_SET_CURRENT_SOFT_SPEED 0xB6u
#define CAN_FD_CMD_COMMAND_ACK 0x7Eu
typedef struct { uint32_t id; uint8_t len; const uint8_t *data; } comm_frame_t;
static uint16_t s_node_id=3;
static unsigned blocked, sent, handled;
static uint8_t last_cmd,last_data[64];
static bool active;
static bool can_fd_custom_send_frame(uint32_t id,uint8_t cmd,const uint8_t *p,uint8_t n) {
  assert(id==3);if(blocked)return false;sent++;last_cmd=cmd;memcpy(last_data,p,n);return true;
}
static bool can_fd_iap_is_active(void) {return active;}
static bool can_fd_custom_is_for_this_node(uint32_t id) {return id==3||id==0||id==CAN_FD_IAP_ID;}
static void can_fd_iap_on_rx(const comm_frame_t *f) {(void)f;}
static void can_fd_custom_handle_command(uint8_t cmd,const uint8_t *p,uint8_t n) {(void)cmd;(void)p;(void)n;handled++;}
'''
start = src.index('static bool s_request_unicast;')
end = src.index('/**', start)
code += src[start:end]
code += function('can_fd_custom_flush_reply')+function('can_fd_custom_reply')+function('can_fd_custom_on_rx')
code += r'''
int main(void) {
 uint8_t payload[3]={23,0x40,1}; comm_frame_t f={3,3,payload};
 blocked=1;can_fd_custom_on_rx(&f);assert(handled==1&&sent==0&&s_reply_count==1);
 payload[0]=99; /* queued payload must be copied, not a pointer to RX scratch */
 blocked=0;can_fd_custom_flush_reply();assert(sent==1&&last_cmd==0x7E&&last_data[0]==23&&last_data[1]==0x40);
 f.id=0;can_fd_custom_on_rx(&f);assert(handled==2&&sent==1);
 can_fd_custom_reply(0x40,payload,1);assert(sent==1); /* no broadcast reply storm */
 f.id=3;payload[1]=0x7E;can_fd_custom_on_rx(&f);assert(handled==2&&sent==1);
 payload[1]=0x41;active=true;can_fd_custom_on_rx(&f);assert(sent==1);active=false;
 blocked=1;s_request_unicast=true;
 for(unsigned i=0;i<REPLY_QUEUE_DEPTH+1;i++)can_fd_custom_reply(0x40,payload,1);
 assert(s_reply_count==REPLY_QUEUE_DEPTH&&s_reply_drops==1);
 blocked=0;while(s_reply_count)can_fd_custom_flush_reply();
 assert(sent==REPLY_QUEUE_DEPTH+1);
 puts("PASS command ACK: unicast identity, broadcast suppression, busy retry, copied payload, bounded queue, no ACK loop");
 return 0;
}
'''
w = a.build_dir.resolve()
w.mkdir(parents=True, exist_ok=True)
(w/'ack_test.c').write_text(code, encoding='utf-8')
vc = Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
batch = w/'build.cmd'
batch.write_text(f'@echo off\ncall "{vc}" >nul\ncl /nologo /TC /std:c11 /utf-8 /W3 /WX ack_test.c /Fe:ack_test.exe\nif errorlevel 1 exit /b 1\nack_test.exe\nexit /b %errorlevel%', encoding='utf-8')
subprocess.run(['cmd', '/d', '/c', str(batch)], cwd=w, check=True)

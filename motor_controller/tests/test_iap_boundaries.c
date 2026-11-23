#include "iap_upgrade.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned erases, writes, resets, replies;
static uint8_t response[3];
void flash_unlock(void) {}
void flash_lock(void) {}
int flash_sector_erase(uint32_t address) {
  assert(address >= IAP_STAGING_BASE_ADDR && address < 0x08020000u);
  ++erases; return 0;
}
int flash_word_program(uint32_t address, uint32_t value) {
  (void)value; assert(address >= IAP_STAGING_BASE_ADDR && address < 0x08020000u);
  ++writes; return 0;
}
void nvic_system_reset(void) { ++resets; }
uint32_t dwt_get_ticks_us(void) { return 0; }
static bool send_reply(uint8_t *b, uint16_t n) {
  assert(n == 3); memcpy(response,b,3); ++replies; return true;
}
static void begin(uint32_t total, unsigned offset, unsigned compress) {
  uint8_t enter[2]={0,IAP_CMD_ENTER};
  uint8_t start[34]={1,IAP_CMD_START,'A','S','I','X',3,0};
  IAP_Comm_t *comm=IAP_Init();
  comm->send=send_reply;
  erases=writes=resets=replies=0;
  memset(response,0,sizeof(response));
  start[8]=(uint8_t)compress;
  start[13]=(uint8_t)offset;
  start[14]=(uint8_t)(total>>24); start[15]=(uint8_t)(total>>16);
  start[16]=(uint8_t)(total>>8); start[17]=(uint8_t)total;
  IAP_Process(enter,2); IAP_Process(start,sizeof(start));
}
static void reject(uint32_t total, unsigned offset, unsigned compress, unsigned error) {
  begin(total,offset,compress);
  assert(IAP_GetStatus()->error==error);
  assert(erases==0 && writes==0 && resets==0);
  IAP_Error(); /* foreground sends failure; no Flash cleanup for rejected START */
  assert(replies==1 && response[0]==IAP_CMD_STATUS && response[1]==IAP_STATE_ERROR && response[2]==error);
  assert(erases==0 && writes==0 && resets==0);
  assert(IAP_GetStatus()->state==IAP_STATE_IDLE);
}
int main(void) {
  reject(32+55297,32,0,ERROR_IAP_IMAGE_TOO_LARGE);
  reject(32+58000,32,0,ERROR_IAP_IMAGE_TOO_LARGE);
  reject(0xffffffffu,32,0,ERROR_IAP_IMAGE_TOO_LARGE);
  reject(32,32,0,ERROR_IAP_INVALID_PARAM);
  reject(0,32,0,ERROR_IAP_INVALID_PARAM);
  reject(64,0,0,ERROR_IAP_INVALID_PARAM);
  reject(64,32,1,ERROR_IAP_INVALID_PARAM);
  begin(32+55296,32,0);
  assert(IAP_GetStatus()->state==IAP_STATE_RECEIVING && IAP_GetStatus()->error==ERROR_IAP_SUCCESS);
  assert(erases==55 && writes==8 && resets==0);
  assert(replies==1 && response[2]==ERROR_IAP_SUCCESS);
  puts("PASS IAP: exact 54 KiB boundary accepted; oversized/invalid START has zero erase/write/reset, reports failure, permits retry");
  return 0;
}

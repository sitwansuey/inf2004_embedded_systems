/* Reference capture -> existing main navigation: no route or motor replacement. */
#include <assert.h>
#include <stdio.h>
#include "buddy3_core.h"
static s3_core_t core;
static s3_output_t output;
static uint32_t now;
static unsigned events,decoded,rejected;
static char letter;
static s3_navigation_t expected;
static unsigned front_mask;
static const uint16_t codes[5]={0x109U,0x049U,0x148U,0x019U,0x118U};
static void setup(bool retry) {
 s3_config_t cfg;s3_config_defaults(&cfg);cfg.line.line_mask=3U;cfg.barcode_sensor=2U;
 cfg.line.white[2]=0U;cfg.line.black[2]=1U;cfg.barcode.min_run_ms=20U;cfg.barcode_retry=retry;
 cfg.map_a=S3_NAV_LEFT;cfg.map_b=S3_NAV_RIGHT;cfg.map_c=S3_NAV_STRAIGHT;cfg.map_d=S3_NAV_UTURN;
 assert(s3_core_init(&core,&cfg));assert(s3_core_control(&core,S3_MODE_TRACK,true));
 now=events=decoded=rejected=0U;front_mask=1U;letter=0;
}
static void feed(bool black,unsigned duration) {
 for(unsigned n=0U;n<duration;n+=10U) {
  now+=10U;
  s3_sample_t sample={.time_ms=now,.valid_mask=7U,.raw={
   (front_mask&1U)?4095U:0U,(front_mask&2U)?4095U:0U,black?1U:0U}};
  s3_core_step(&core,&sample,&output);
  if(output.barcode_status==S3_BARCODE_OK){++decoded;letter=output.decoded.symbol;}
  if(output.barcode_status>S3_BARCODE_OK)++rejected;
  if(output.event.kind==S3_EVENT_NAVIGATION) {
   ++events;assert(output.event.command==expected);assert(output.event.symbol==letter);
  }
 }
}
static void frame(unsigned index,bool reverse,bool prefix,bool junction) {
 uint32_t widths[29];const uint16_t patterns[3]={0x094U,codes[index],0x094U};
 for(unsigned c=0U;c<3U;++c)for(unsigned n=0U;n<9U;++n)
  widths[c*10U+n]=(patterns[c]&(1U<<(8U-n)))?120U:40U;
 widths[9]=widths[19]=40U;
 if(reverse)for(unsigned n=0U;n<14U;++n){uint32_t v=widths[n];widths[n]=widths[28U-n];widths[28U-n]=v;}
 if(prefix){feed(true,20U);feed(false,40U);}
 for(unsigned n=0U;n<29U;++n){if(junction && n==10U)front_mask=3U;feed((n&1U)==0U,widths[n]);}
 feed(false,600U);
}
int main(void) {
 const s3_navigation_t maps[4]={S3_NAV_LEFT,S3_NAV_RIGHT,S3_NAV_STRAIGHT,S3_NAV_UTURN};
 for(unsigned n=0U;n<4U;++n)for(unsigned backwards=0U;backwards<2U;++backwards){
  setup(true);expected=maps[n];feed(false,250U);frame(n,backwards!=0U,true,n==0U);
  assert(decoded==1U && letter==(char)('A'+(int)n) && events==1U && core.awaiting_controller);
  assert(core.latched_error==S3_ERROR_NONE);
 }
 setup(true);feed(false,250U);feed(true,10U);feed(false,10U);
 assert(rejected==1U && !core.awaiting_controller && output.steering_allowed && core.latched_error==S3_ERROR_NONE);
 feed(false,250U);assert(core.barcode.state==S3_CAPTURE_READY);expected=S3_NAV_LEFT;frame(0U,false,false,false);assert(events==1U);
 setup(true);feed(false,250U);frame(4U,false,false,true);
 assert(letter=='E' && decoded==1U && !events && !core.awaiting_controller && output.steering_allowed);
 feed(false,200U);front_mask=1U;feed(false,250U);assert(core.barcode.state==S3_CAPTURE_READY);
 expected=S3_NAV_LEFT;frame(0U,false,false,false);assert(decoded==2U && events==1U);
 setup(false);feed(false,250U);feed(true,10U);feed(false,10U);assert(core.latched_error==S3_ERROR_BARCODE && !output.steering_allowed);
 setup(true);s3_sample_t bad={.time_ms=10U,.valid_mask=3U};s3_core_step(&core,&bad,&output);assert(core.latched_error==S3_ERROR_SENSOR);
 puts("PASS: reference capture (both directions/noise prefix/junction) preserves main A-D navigation; unmapped E stays unmapped; retry and sensor-fault behavior verified.");
 return 0;
}

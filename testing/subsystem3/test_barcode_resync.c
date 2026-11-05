/* Sampled capture tests: leading noise, complete framing and bounded retry. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "buddy3_barcode.h"

static const uint16_t codes[26]={
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0
};
static const s3_barcode_config_t cfg={200U,0U,10000U};
static unsigned cases;

static void frame(unsigned letter,bool reverse,uint32_t runs[29]) {
    const uint16_t patterns[3]={0x094U,codes[letter],0x094U};
    for(unsigned c=0;c<3;++c)for(unsigned i=0;i<9;++i)
        runs[c*10U+i]=(patterns[c]&(1U<<(8U-i)))?60U:20U;
    runs[9]=20U;runs[19]=20U;
    if(reverse)for(unsigned i=0;i<14;++i){uint32_t v=runs[i];runs[i]=runs[28U-i];runs[28U-i]=v;}
}
static void start(s3_barcode_t *b,uint32_t *now,s3_symbol_t *symbol) {
    s3_barcode_init(b);
    assert(s3_barcode_feed_resync(b,&cfg,false,*now,symbol)==S3_BARCODE_NONE);
    *now+=200U;
    assert(s3_barcode_feed_resync(b,&cfg,false,*now,symbol)==S3_BARCODE_NONE);
    *now+=10U;
    assert(s3_barcode_feed_resync(b,&cfg,true,*now,symbol)==S3_BARCODE_NONE);
}
static s3_barcode_status_t runs(s3_barcode_t *b,const s3_barcode_config_t *c,
    const uint32_t *widths,unsigned count,uint32_t *now,s3_symbol_t *symbol) {
    for(unsigned n=0;n<count;++n)for(uint32_t elapsed=10U;elapsed<=widths[n];elapsed+=10U) {
        *now+=10U;
        bool black=elapsed==widths[n]?(n&1U)!=0U:(n&1U)==0U;
        s3_barcode_status_t status=s3_barcode_feed_resync(b,c,black,*now,symbol);
        if(status!=S3_BARCODE_NONE)return status;
        assert(!symbol->symbol && b->count<=29U && b->resync_pairs<=S3_BARCODE_MAX_RESYNC_PAIRS);
    }
    return S3_BARCODE_NONE;
}
static s3_barcode_status_t finish(s3_barcode_t *b,const s3_barcode_config_t *c,
    uint32_t *now,s3_symbol_t *symbol) {
    uint32_t quiet=b->max_run*4U;if(quiet<c->quiet_ms)quiet=c->quiet_ms;
    for(uint32_t elapsed=10U;elapsed<quiet;elapsed+=10U) {
        *now+=10U;
        s3_barcode_status_t status=s3_barcode_feed_resync(b,c,false,*now,symbol);
        if(status!=S3_BARCODE_NONE)return status;
        assert(!symbol->symbol);
    }
    *now+=10U;return s3_barcode_feed_resync(b,c,false,*now,symbol);
}
static void complete(unsigned letter,bool reverse,unsigned prefix,uint32_t base) {
    uint32_t all[2U*S3_BARCODE_MAX_RESYNC_PAIRS+29U],expected[29],now=base;
    frame(letter,reverse,expected);
    for(unsigned n=0;n<prefix;++n){all[n*2U]=20U;all[n*2U+1U]=40U;}
    memcpy(all+prefix*2U,expected,sizeof(expected));
    s3_barcode_t b;s3_symbol_t symbol;start(&b,&now,&symbol);uint32_t began=now;
    assert(runs(&b,&cfg,all,prefix*2U+29U,&now,&symbol)==S3_BARCODE_NONE);
    assert(b.started==began && b.resync_pairs==prefix && b.count==29U);
    assert(!memcmp(b.runs,expected,sizeof(expected)));
    assert(finish(&b,&cfg,&now,&symbol)==S3_BARCODE_OK);
    assert(symbol.symbol==(char)('A'+letter) && symbol.reverse==reverse);
    now+=10U;assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    assert(!symbol.symbol && b.state==S3_CAPTURE_HOLD);
    s3_barcode_rearm(&b);now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    assert(b.state==S3_CAPTURE_READY);now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,true,now,&symbol)==S3_BARCODE_NONE);
    assert(b.resync_pairs==0U && b.count==0U);
    ++cases;
}
static void failure_cases(void) {
    uint32_t data[64],now=0;s3_barcode_t b;s3_symbol_t symbol;
    for(unsigned n=0;n<64;++n)data[n]=10U;
    start(&b,&now,&symbol);
    assert(runs(&b,&cfg,data,46U,&now,&symbol)==S3_BARCODE_OVERFLOW);
    assert(!symbol.symbol && b.state==S3_CAPTURE_HOLD && b.resync_pairs==8U && b.count==29U);
    s3_barcode_config_t short_cfg=cfg;short_cfg.timeout_ms=1000U;
    now=0;start(&b,&now,&symbol);uint32_t began=now;
    assert(runs(&b,&short_cfg,data,30U,&now,&symbol)==S3_BARCODE_NONE && b.resync_pairs==1U);
    while(now-began<990U){now+=10U;assert(s3_barcode_feed_resync(&b,&short_cfg,true,now,&symbol)==S3_BARCODE_NONE);}
    now+=10U;assert(s3_barcode_feed_resync(&b,&short_cfg,true,now,&symbol)==S3_BARCODE_TIMEOUT);
    assert(b.started==began && !symbol.symbol);
    /* Incomplete / damaged framing / damaged spacing must not become letters. */
    for(unsigned trial=0;trial<5;++trial) {
        uint32_t card[29];frame(5U,false,card);
        if(trial==1U)for(unsigned n=0;n<9;++n)card[n]=20U;
        if(trial==2U)for(unsigned n=20;n<29;++n)card[n]=20U;
        if(trial==3U)card[9]=100U;
        if(trial==4U)for(unsigned n=10;n<19;++n)card[n]=20U;
        data[0]=20U;data[1]=40U;memcpy(data+2,card,sizeof(card));
        now=0;start(&b,&now,&symbol);
        assert(runs(&b,&cfg,data,trial==0U?27U:31U,&now,&symbol)==S3_BARCODE_NONE);
        assert(finish(&b,&cfg,&now,&symbol)!=S3_BARCODE_OK && !symbol.symbol);
    }
    /* A discarded long black prefix must not delay the quiet exit. */
    frame(0U,false,data+2);data[0]=1000U;data[1]=40U;
    now=0;start(&b,&now,&symbol);
    assert(runs(&b,&cfg,data,31U,&now,&symbol)==S3_BARCODE_NONE);
    assert(b.max_run==60U && finish(&b,&cfg,&now,&symbol)==S3_BARCODE_OK);
    assert(symbol.symbol=='A');
    now=0;start(&b,&now,&symbol);
    assert(s3_barcode_feed_resync(&b,&cfg,true,now,&symbol)==S3_BARCODE_NOISE);
    now=0;start(&b,&now,&symbol);
    s3_barcode_config_t floor_cfg=cfg;floor_cfg.min_run_ms=20U;
    assert(runs(&b,&floor_cfg,data,1U,&now,&symbol)==S3_BARCODE_NONE);
    now+=10U;assert(s3_barcode_feed_resync(&b,&floor_cfg,true,now,&symbol)==S3_BARCODE_NOISE);
    s3_barcode_abort(&b);assert(b.resync_pairs==0U && !b.count);
}
static void recorded_f_prefix(void) {
    const uint32_t measured[2][29]={
        {30,190,30,80,140,70,130,80,20,80,20,80,120,70,130,160,30,70,30,80,30,190,30,70,130,60,140,70,30},
        {20,200,30,90,120,80,120,80,40,60,30,70,130,70,120,180,30,80,20,80,30,190,30,80,130,70,120,80,20}
    };
    /* Valid F capture plus observed stray pulse lengths: not reconstruction
     * of the missing tail of an overflow log. */
    for(unsigned fixture=0;fixture<2;++fixture)for(unsigned reverse=0;reverse<2;++reverse) {
        uint32_t data[31]={20U,80U},now=0;s3_barcode_t b;s3_symbol_t symbol;
        for(unsigned n=0;n<29;++n)data[n+2U]=measured[fixture][reverse?28U-n:n];
        start(&b,&now,&symbol);
        assert(runs(&b,&cfg,data,31U,&now,&symbol)==S3_BARCODE_NONE);
        assert(finish(&b,&cfg,&now,&symbol)==S3_BARCODE_OK);
        assert(symbol.symbol=='F' && symbol.reverse==(reverse!=0U) && b.resync_pairs==1U);
    }
}
int main(void) {
    for(unsigned letter=0;letter<26;++letter)for(unsigned reverse=0;reverse<2;++reverse)
        for(unsigned prefix=0;prefix<=S3_BARCODE_MAX_RESYNC_PAIRS;++prefix)
            complete(letter,reverse!=0U,prefix,(letter&1U)?UINT32_MAX-500U:0U);
    failure_cases();recorded_f_prefix();
    printf("Barcode resync: %u A-Z/direction/prefix cases; quiet/timeout/wrap/budget/malformed/held/rearm and measured-F prefix PASS\n",cases);
    return 0;
}

/* Reuse the independent Code39 fixture generator and sampled feed helpers. */
#define main resync_fixture_suite_main
#include "test_barcode_resync.c"
#undef main

static const uint32_t recorded_a[29]={
    20,190,20,80,120,80,120,70,30,70,130,70,30,70,30,
    180,20,90,120,80,30,180,30,80,130,80,130,70,30
};
static unsigned completion_cases;

static void complete_at(const uint32_t widths[29],char letter,bool reverse,
                        uint32_t base,uint32_t expected_white)
{
    s3_barcode_t b;s3_symbol_t symbol;uint32_t now=base;
    start(&b,&now,&symbol);
    assert(runs(&b,&cfg,widths,29U,&now,&symbol)==S3_BARCODE_NONE);
    assert(b.count==29U && b.state==S3_CAPTURE_RUNNING && !symbol.symbol);
    uint32_t last_edge=now;
    for(uint32_t elapsed=10U;elapsed<expected_white;elapsed+=10U){
        now+=10U;
        assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
        assert(!symbol.symbol);
    }
    now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_OK);
    assert(now-last_edge==expected_white && symbol.symbol==letter && symbol.reverse==reverse);
    assert(b.state==S3_CAPTURE_HOLD);
    now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE && !symbol.symbol);
    s3_barcode_rearm(&b);now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    assert(b.state==S3_CAPTURE_READY);
    ++completion_cases;
}

static void invalid_and_strict(void)
{
    s3_barcode_t b;s3_symbol_t symbol;uint32_t now=0,damaged[29];
    memcpy(damaged,recorded_a,sizeof(damaged));damaged[0]=120U;
    start(&b,&now,&symbol);
    assert(runs(&b,&cfg,damaged,29U,&now,&symbol)==S3_BARCODE_NONE);
    uint32_t edge=now;now+=380U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    assert(b.completed_window_checked && b.state==S3_CAPTURE_RUNNING && !symbol.symbol);
    now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    assert(b.completed_window_checked && !symbol.symbol);
    /* Another edge clears the cached check and still permits bounded resync. */
    now+=10U;
    assert(s3_barcode_feed_resync(&b,&cfg,true,now,&symbol)==S3_BARCODE_NONE);
    assert(!b.completed_window_checked && b.resync_pairs==1U && b.count==28U);
    /* A malformed candidate with no further edges keeps the old rejection deadline. */
    now=0;start(&b,&now,&symbol);
    assert(runs(&b,&cfg,damaged,29U,&now,&symbol)==S3_BARCODE_NONE);
    edge=now;now+=750U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    now=edge+760U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_BAD_PATTERN);
    assert(!symbol.symbol && b.state==S3_CAPTURE_HOLD);
    /* Strict API timing remains four longest runs. */
    now=0;start(&b,&now,&symbol);
    assert(runs(&b,&cfg,recorded_a,29U,&now,&symbol)==S3_BARCODE_NONE);
    edge=now;now+=380U;
    assert(s3_barcode_feed(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    now=edge+760U;
    assert(s3_barcode_feed(&b,&cfg,false,now,&symbol)==S3_BARCODE_OK && symbol.symbol=='A');
    /* Partial capture, pause/abort and absolute timeout still cannot emit a letter. */
    now=0;start(&b,&now,&symbol);const uint32_t pulse[1]={120U};
    assert(runs(&b,&cfg,pulse,1U,&now,&symbol)==S3_BARCODE_NONE);edge=now;now+=380U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE);
    now=edge+480U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_INCOMPLETE && !symbol.symbol);
    now=0;start(&b,&now,&symbol);
    assert(runs(&b,&cfg,recorded_a,29U,&now,&symbol)==S3_BARCODE_NONE);
    s3_barcode_abort(&b);now+=1000U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_NONE && !symbol.symbol);
    now=0;start(&b,&now,&symbol);
    assert(runs(&b,&cfg,recorded_a,28U,&now,&symbol)==S3_BARCODE_NONE);now+=500U;
    assert(s3_barcode_feed_resync(&b,&cfg,true,now,&symbol)==S3_BARCODE_NONE && !symbol.symbol);
    memcpy(b.runs,recorded_a,sizeof(b.runs));b.count=29U;b.max_run=190U;
    b.started=0;b.last_edge=9620U;b.last_sample=9990U;now=10000U;
    assert(s3_barcode_feed_resync(&b,&cfg,false,now,&symbol)==S3_BARCODE_TIMEOUT && !symbol.symbol);
}

int main(void)
{
    const uint32_t units[4]={20U,40U,50U,80U};
    for(unsigned letter=0;letter<26;++letter)for(unsigned reverse=0;reverse<2;++reverse)
        for(unsigned scale=0;scale<4;++scale){
            uint32_t widths[29];frame(letter,reverse!=0U,widths);
            for(unsigned n=0;n<29;++n)widths[n]=widths[n]*units[scale]/20U;
            uint32_t white=units[scale]*6U;if(white<cfg.quiet_ms)white=cfg.quiet_ms;
            complete_at(widths,(char)('A'+letter),reverse!=0U,
                        letter&1U?UINT32_MAX-500U:0U,white);
        }
    complete_at(recorded_a,'A',false,0U,380U);
    uint32_t reverse_a[29];for(unsigned n=0;n<29;++n)reverse_a[n]=recorded_a[28U-n];
    complete_at(reverse_a,'A',true,UINT32_MAX-500U,380U);
    invalid_and_strict();
    printf("Completion: %u A-Z/direction/scale/wrap and recorded-A cases;380ms vs760ms; strict/invalid/partial/resync/abort/timeout gates PASS\n",completion_cases);
}

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "edr.h"
typedef struct { unsigned char bytes[16]; size_t length; bool valid; unsigned int id, data; } vector_t;
#include "edr_test_vectors.h"
int main(void)
{
    unsigned int word, id, data;
    for (word=0; word<65536U; ++word) {
        unsigned int sid=word & 15U, value=word >> 4U;
        bool expected = sid != 0U && (sid!=3U || value<=180U) &&
            (sid!=12U || value<=127U) && (sid!=11U ||
            ((value & 7U)<=6U && ((value>>3U)&127U)<=100U && (value>>10U)<=2U));
        uint8_t bytes[3]={(uint8_t)(word & 255U),(uint8_t)(word >> 8U),0xa5U};
        uint8_t output[3]={0xa5U,0xa5U,0xa5U};
        uint16_t packed=0xbeefU;
        id=99U; data=99U;
        assert(edr_decode(bytes,2U,&id,&data)==expected);
        assert(edr_pack(sid,value,&packed)==expected);
        assert(edr_encode(sid,value,output,2U)==expected);
        if (expected) {
            assert(id==sid && data==value && packed==word);
            assert(memcmp(bytes,output,2U)==0 && output[2]==0xa5U);
        } else {
            assert(id==99U && data==99U && packed==0xbeefU);
            assert(output[0]==0xa5U && output[1]==0xa5U);
        }
    }
    for (size_t i=0; i<sizeof(vectors)/sizeof(vectors[0]); ++i) {
        const vector_t *v=&vectors[i];
        assert(edr_decode(v->bytes,v->length,&id,&data)==v->valid);
        if (v->valid) assert(id==v->id && data==v->data);
    }
    {
        uint16_t packed=123U;
        uint8_t bytes[2]={0xa5U,0xa5U},command=99U,power=99U,state=99U;
        int32_t delta=99;
        assert(!edr_pack(1U,4096U,&packed) && packed==123U);
        assert(!edr_pack(1U,0U,NULL));
        assert(!edr_encode(1U,0U,bytes,1U) && bytes[0]==0xa5U);
        assert(!edr_encode(1U,0U,NULL,2U));
        assert(!edr_decode(NULL,2U,&id,&data));
        assert(!edr_decode(bytes,2U,NULL,&data));
        assert(!edr_decode(bytes,2U,&id,NULL));
        assert(edr_motor_pack(1U,30U,1U,&packed) && packed==1265U);
        assert(edr_motor_unpack(packed,&command,&power,&state));
        assert(command==1U && power==30U && state==1U);
        assert(!edr_motor_pack(7U,30U,1U,&packed));
        assert(!edr_motor_unpack(0xffffU,&command,&power,&state));
        assert(edr_count_delta(0U,0xffffffffU,&delta) && delta==1);
        assert(!edr_count_delta(0x80000000U,0U,&delta));
        assert(edr_age_is_fresh(3U,0xfffffffeU,5U));
        assert(!edr_age_is_fresh(3U,0xfffffffeU,4U));
    }
    puts("EDR16 C: exhaustive words, golden bytes, motor, boundaries and local arithmetic PASS");
    return 0;
}

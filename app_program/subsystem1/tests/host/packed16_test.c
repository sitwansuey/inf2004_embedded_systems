#include <assert.h>
#include "../../packed16.h"
int main(void) {
    unsigned int id, value, got_id, got_value;
    uint16_t word;
    unsigned char bytes[2];
    for (id = 0; id < 16; ++id) {
        for (value = 0; value < 4096; ++value) {
            int valid = id >= 1 && id <= 15 && (id != 3 || value <= 180) &&
                (id != 12 || value <= 127) && (id != 11 ||
                ((value & 7U) <= 6 && ((value >> 3) & 127U) <= 100 && (value >> 10) <= 2));
            word = 0xbeef;
            assert(s1_pack16(id, value, &word) == valid);
            if (!valid) { assert(word == 0xbeef); continue; }
            s1_pack16_bytes(word, bytes);
            assert(((unsigned int)bytes[0] | ((unsigned int)bytes[1] << 8)) == word);
            assert(s1_unpack16(word, &got_id, &got_value));
            assert(got_id == id && got_value == value);
        }
    }
    assert(!s1_pack16(1, 4096, &word));
    assert(!s1_pack16(1, 1, 0));
    assert(!s1_unpack16(0, &got_id, &got_value));
    return 0;
}

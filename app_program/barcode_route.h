#ifndef CSC02_BARCODE_ROUTE_H
#define CSC02_BARCODE_ROUTE_H
#include <stdbool.h>
#include <stdint.h>
#include "subsystem3/buddy3_core.h"

#define BARCODE_MAP_BYTES 4U /* forward, U-turn, left, right; four distinct A-Z */
#define BARCODE_MAP_JSON_MAX 128U
#define BARCODE_JUNCTION_MS 30U
enum { BARCODE_MAP_OK, BARCODE_MAP_BUSY, BARCODE_MAP_INVALID,
       BARCODE_MAP_QUEUE_FULL, BARCODE_MAP_STALE };
typedef struct {
    uint8_t letters[BARCODE_MAP_BYTES];
    char pending;
    bool configured, junction_latched, timing, candidate_black, fault;
    bool default_forward; /* Course profile: no mapped instruction means straight. */
    uint32_t since_ms, last_ms;
    uint32_t capture_result_id;
} barcode_route_t;
bool barcode_map_valid(const uint8_t letters[BARCODE_MAP_BYTES]);
/* Strict length-delimited JSON: exactly forward/uturn/left/right, no escapes,
 * duplicate/extra keys, trailing data or duplicate characters. Any key order. */
bool barcode_map_parse(const uint8_t *json, unsigned length, uint8_t letters[BARCODE_MAP_BYTES]);
unsigned barcode_route_configure(barcode_route_t *route, const uint8_t letters[BARCODE_MAP_BYTES], bool armed);
void barcode_route_clear(barcode_route_t *route);
bool barcode_route_store(barcode_route_t *route, char symbol);
/* A junction seen during capture belongs to that physical patch. Ignore it
 * until normal debounced clearance; a completed-on-junction result is retained
 * so the controller cannot miss the terminal sampling tick. */
void barcode_route_observe_capture(barcode_route_t *route,bool fresh,unsigned mask,
                                  bool scanning,uint32_t result_id,unsigned result_mask);
s3_navigation_t barcode_route_step(barcode_route_t *route, uint32_t now_ms,
                                  bool fresh, unsigned black_mask, bool tracking);
#endif

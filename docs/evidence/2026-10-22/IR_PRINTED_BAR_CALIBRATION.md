# Printed barcode calibration mismatch, 14:49/14:50 +08:00, 22 October 2026

The stationary GP28 check fails geometry/calibration review. Both recordings
are valid acquisitions:802 distinct complete scans each, every interval10ms,
mask7/drop0, no malformed/error record.16 controller records remain STOP,
disarmed,power0,fault0. No command was sent. A valid acquisition is not a
valid calibration or successful barcode decoding.

| Label | GP28 raw range | Mean | Median | Current normalized darkness | Label mismatches |
|---|---|---:|---:|---|---:|
| WHITE | 147..198 | 171.626 | 172 | 0..0 | 0 / 802 |
| Printed BLACK bar | 783..1034 | 890.397 | 889 | 183..263 | 802 / 802 |

Current GP28 endpoints are WHITE213/BLACK3326 with400/600 hysteresis.
The entire printed-BLACK capture is below400 and is interpreted as white.
Median contrast is717. BLACK range251 exceeds the existing stability
limit717/4=179.25; white range51 passes. BLACK also drifts upward during
the recording, unlike the largely stationary other two ADC channels.
Every sample is retained; no outlier removal or endpoint update was applied.

## User clarification and previous evidence

The user says the new BLACK target is a printed black bar; the original
BLACK calibration target was the table. The actual sensor-to-paper gap is
unknown. The previous source notes did not identify that material. The
3326 reference must therefore be described as a table-derived black point,
not a verified printed-ink endpoint. The earlier161/161 stationary
classification and362 labelled-sample native replay remain results for
their original labelled targets; they do not validate actual printed bars
at a changed height/angle. This clarification does not invalidate the
electronic acquisition or the algorithm's execution on those observations.

Moving Z recordings previously reach GP28 raw3913/3920, whereas this held
printed bar reaches only1034. Surface, sensor distance/angle, spot coverage
or handling differences need inspection. These measurements cannot establish
one sole cause. Replacing3326 by889 without establishing a reproducible
printed-card plane would turn the observed setup drift into a new reference.
No endpoint, decoder tolerance, navigation map or firmware is changed.

## Next physical information

Before another scan, obtain a side-on photo showing GP28's optical sensing
face and the barcode card in its normal reading position. Include the gap,
mounting angle and which printed bar is under the sensing window; a ruler
beside the gap is useful if available. This is to establish the setup and
sensor model/optical coverage, not to claim precision from a photo. The
robot remains motor-locked with motor power off; leave wiring and trim
screws unchanged. Recalibrate on the actual printed card only after the
intended fixed geometry and sufficient black/white coverage are established.

No extra sweep or reflash is needed for this inspection. Current sampling
firmware346DC0 remains byte-identical, staticRAM26040/telemetry50 bytes.
Onboard decoding/motion and physical symbol validation remain outstanding.
No CR2/CR3, push, PR or remote modification. Documentation/provenance updates
are local only and do not require an ARM rebuild.

Evidence: task outputs BARCODE_PLANE_1450/ preserve original logs, results,
warmups and pre-update metadata. BARCODE_PLANE_1450_ANALYSIS.json verifies
all1604 records, normalized ranges, drift and other-channel observations.
The PC plane-check fixture passes were synthetic; this real plane check
does not pass and no physical success is substituted for it.

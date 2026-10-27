# Buddy 4 integration checkpoint

This checkpoint preserves the tested local integration based on published main
0269a02, the supplied integrated handoff and the initial Buddy 4 encoder/terrain
adapter. CR1 compliance remains incomplete: internal application fields still
need range-based packing, a Buddy 4 output contract and updated measurements.
Existing test/build results are recorded in testing/subsystem4/verification-20261027.json.
No new physical validation is claimed. The code remains motor-locked by default.

Main advanced to f8107be (Buddy 5 PR #7) before publication of this checkpoint.
Those newer changes have not been reconciled into this tested tree. The draft
PR deliberately records that integration gap and is not ready to merge.
Further Buddy 4 work must retain this checkpoint and distinguish its validation
from later code changes. Owner review of recovered shared/B1/B2/B3/B5 work is
required before merging. Publishing this draft is not merging or flashing.

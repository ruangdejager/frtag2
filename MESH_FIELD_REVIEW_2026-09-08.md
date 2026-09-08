# Mesh field review — 2026-09-08

Handover notes. Everything below was measured or verified in this session. HEAD is **bebbe2f (v2.3.5)**,
committed and clean. `MESH_DIAG_COUNTERS` is **enabled** in the worktree (uncommitted) for the field test.

---

## 1. Framing fact: the herd is a daytime mesh

Split the primary's 97 campaigns (2026-09-05..07) by hour and they are two different populations:

| | n | med union | med waves | hit wave cap | union ≥6 |
|---|---|---|---|---|---|
| Day 08:00–15:59 | 33 | 6 | 6 | 18/33 | 18 |
| Night 17:00–07:59 | 60 | 2 | 3 | 5/60 | 2 |

221D/2D94/F15 log `DReq heard=0` in **100%** of campaigns 17:00–07:00 and ~0% of 08:00–15:00.
1316 sits beside the primary and hears all 98. Livestock tags: in range grazing, out of range overnight.

**Consequence: the effective sample for any deep-herd claim is ~33 campaigns, not 97.** The
"52/97 campaigns returned union=2" figure is mostly night campaigns where the herd is genuinely
absent — not a discovery failure. Night behaviour (3 waves, union 2, ~25 s) is the min-waves plus
two-barren rule working correctly.

## 2. Measured baseline

**DReq → first-beacon-back round trip** (n=138, joined on dreq id across logs):

| wave | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| median | 1 | 5 | 6 | 5 | 6 | 7 |
| max | 3 | 8 | 9 | 11 | 8 (n=7) | 25 |

Overall median 3 s, p90 8 s. **Only 1 of 138 samples exceeds 12 s.** The 2/3/4/8/**21** s table
justifying `ALLOWANCE 4000` and `CAP 24000/27000` is not reproduced — its ring-5 datapoint is node
241F, which has no log here.

> **CENSORING — read before touching any floor constant.** A wave cannot end before its floor, so a
> node answering after the wave ended cannot appear in this distribution. It is right-censored **by
> the floor itself** and therefore **cannot refute a long floor**. Any verdict that treats it as an
> unbiased sample is invalid. Narrowing the floor before measuring re-censors it at a lower bound.

**Wave duration** (gap between consecutive DReq sends): 1→2 med 13 / 2→3 med 12 / 3→4 med 16 /
4→5 med 24 / 5→6 med 24. Durations sit **on** the scaled floor — the primary burns the floor rather
than bailing early. Campaign span first→last DReq: 3 waves med 25 s, 4 waves 44 s, 5 waves 68 s,
6 waves 96 s (max 107).

**Relay amplification** — per-campaign `forwarded=` max: 1316 **58**, 221D 48, 2D94 30, F15 30.
Three-day totals 1539 / 796 / 333 / 376. Primary always 0. Beacon *origination* is already bounded
(max 8 per episode), so the "48 beacons in 171 s" still cited in `MeshNetwork.h` is pre-2.3.1 and
**stale**. The multiplier is the airtime problem, not origination.
Counter-evidence: `cadTmo` is 0 everywhere except 221D (13 over 3 days), `txDrop` 0–1. **At ten nodes
there is no measured carrier-sense deafness.**

**Beacon/ack**: ack latency med 7–9 s, p90 12–19 s, max 26 s. Beacons/episode med 1–2, max 4–8.
Awake/campaign med 15 s (silence bail), p90 ~128 s, max 185 s. "Late D-Ack" fires 43× across 4 nodes.

**TimeSync**, aligned to the primary's window (secondary logs start 2026-09-04, so unaligned rates
are wrong): 1316 99/98, 221D 27/97, 2D94 23/83, F15 22/95. **Every early bail had `DReq heard=0`** —
this is the reachability effect of §1, not a TimeSync fault. Conditional on hearing anything,
delivery is **79–85%**. Clock skew primary→secondary is **0 s**, so cross-log arithmetic is valid.
Primary sends TimeSync at campaign start +34 s min / 61 s median / 132 s max; a tag hearing nothing
bails at 15 s.

**fr9 link is healthy and fast**: session (`Wait for RDY` → `Farmranger released`) n=97,
median **1 s**, max **3 s**, zero sessions over 5 s. 97/97 LogData uploads `verdict='O'`. Zero
`FWREQ` waits. The "~55 s worst case fr9 budget" in `DeviceDiscovery.h` is an AT-timeout bound that
**never occurred**. So 2.3.4's 15→10 s margin raid took slack that was genuinely there — safe. The
comment should be relabelled as an untested bound, not a measured cost.

## 3. Off-by-one: wave floor vs budget model

`MESHNETWORK_u8GetMaxDiscoveredWave()` returns the wave a node **answered**, so during wave k the
floor is `8000 + k*4000` — **one ALLOWANCE step deeper** than `MESH_WAVE_BUDGET_MS` models.
Field confirms it: wave 1→2 measures min 12 s / med 13 s, not the modelled 8 s. In shallow campaigns
the index pins at 1, so every wave floors at 12 s. `MESH_WAVE_BUDGET_MS` is therefore not a true
upper bound. Proposed fix (unapplied): clamp the ring index to `u8WaveCount - 1`.

## 4. What v2.3.5 (bebbe2f) changed

1. **Barren predicate reads new nodes, not beacons.** `bBeaconSeenThisWave` keyed off
   `MESHNETWORK_u32GetLastBeaconHeardTick`, which is stamped *inside* the `FORWARD_bHasSeen` early
   return — so a **deduped duplicate** beacon reset the barren run and kept campaigns extending.
   Scales with herd size (58 forwards at ten nodes), so the early exit could stop firing at 30–50.
   Now `bNoNewThisWave = (u16UnionNow == u16WaveUnion0)`. Un-acked nodes unaffected — they hold the
   *wave* open via `bUnackedHold`, evaluated first.
2. **FOTA auto-arm hoisted out of the once-per-wake gate** (`MeshNetwork.c` ~1929, between the
   primary-role guard and the gate). Both primaries wake on the same slot; if only one holds the
   staged image and the tag heard the imageless one first, it latched
   `bTimeSyncAcceptedThisWake` and never read the other's staged version. Arms for the *next*
   campaign — does not guarantee same-wake OtaPrep capture, since the AppTask notify still comes
   from the accepted TimeSync.
3. **Neighbour-table-full now logs** instead of dropping a node silently. Flag set under the mutex,
   logged after release (DBG_LOG can block on UART/flash; that mutex is taken per beacon).

Instrumentation added: per-wave line gains `barren=`, `prevBarren=`, `beaconSeen=`; wave-cap line
gains `barren=`/`prevBarren=`. **`beaconSeen=` is the old predicate kept purely as an observable —
where it disagrees with `barren=` is the dedupe echo rate, measured directly.**

**Verified**: both configs build clean, zero warnings. `.bss` byte-identical to pre-change —
production **64944** (8 B free), diag **63608** (1344 B free); all additions are stack locals.
`.fw_info` at 0x08005200 reads `02 00 03 00 05 00`. FOTA arm move is a pure relocation
(production text 117564 → 117560).

## 5. FOTA-again safety (the overriding requirement)

- Arming is **self-healing**: re-evaluated on every TimeSync a secondary hears, retries indefinitely.
- Distribution is **broadcast + join**, not addressed from the neighbour table — so shrinking the
  union cannot shrink FOTA reach.
- Watchdog: IWDG live at **8.192 s**, refreshed by the 1 Hz heartbeat off the RTC ISR
  (`Platform/src/platform.c:79`). A genuine hang resets and recovers with the image intact.
  **Caveat: the heartbeat is independent of the mesh tasks, so a blocked parser task would NOT be
  caught** — it would sit alive with the mesh dead.
- Logger cannot block: full ring drops the whole line and saturates a counter (`DbgLog.c:126`).
  No heap use anywhere in the log path.
- **The one way to lose FOTA**: the arm gate is a strict `u32StagedVer > VERSION_u32Get()`. A unit
  already at ≥ the offered version silently ignores it. Field units are on 2.3.4 (20304) so 20305 is
  newer. Recoverable, never permanent — ship a higher patch and it arms.
- Version numbering is confusing: **2.3.6 was committed before 2.3.5**. HEAD (2.3.5) contains all of
  2.3.3 + 2.3.4 + 2.3.6 code plus the three changes above. Nothing was reverted.

## 6. Protocol design intent (authoritative, from Ruan)

A tag belongs to **no** primary. Any primary's DReq may start it beaconing; any primary records and
acks what it hears; per-primary unions are merged **server-side**, so partial coverage per primary is
fine. Once acked by **any** primary the tag stops. A primary cannot know the other already heard a
tag — **accepted, not a defect**.

Verified at HEAD: beaconing start has no origin gate; the beacon→table path keys on device id only;
`vStopBeaconingByOrigin`'s active-beaconing branch has **no** origin comparison (the origin check
survives only to attribute a *late* ack after the node already stopped).

**Known gap, unfixed:** re-anchor **is** origin-gated (`bSamePrimary`), so a tag already beaconing
for A won't re-anchor onto B's DReq and keeps advertising A's wave counter in `dreqWaveDisc`. B
records it, and B's listen floor is driven off A's depth. Low impact while primaries stay in step on
the shared slot; diverges when one ends early on barren waves.

## 7. Adjudication status — INCOMPLETE

59 discrete changes were enumerated across 2.3.3/2.3.4/2.3.6. **Only 12 were ever adjudicated**, and
**none of those 12 passed adversarial verification** — every refuter died on a session limit, so the
earlier run's "0 refuted" means the refuters never ran, not that nothing was refuted. The 12 cluster
almost entirely on 2.3.3's wave/campaign timing.

Surviving verdicts from that partial run worth revisiting (all single-pass, unverified):

- `barren-predicate-is-any-beacon` — CONTRADICTED/RETUNE. **Acted on in v2.3.5.**
- `wave-budget-assert-retune` — CONTRADICTED/RETUNE. The §3 off-by-one. **Not applied.**
- `discovery-window-205s` — UNSUPPORTED/RETUNE, proposed 205 → 165 s on the fr9 measurement. **Not applied.**
- `wave-allowance-4000` — CONTRADICTED/REVERT to 2000, confidence **medium**. **Conflicts** with
  `wave-floor-cap-24000` (UNSUPPORTED/KEEP-BUT-INSTRUMENT), which says keep the cap wide so the
  censored distribution becomes observable. Do not act on either without resolving §2's censoring.

**Never adjudicated — the substantive 2.3.6 behavioural set:**
`dedupe-ring-fold-to-16bit`, `dedupe-ring-grow-to-48`, `dedupe-ring-clear-per-campaign`,
`tx-jitter-density-scaling`, `dack-ids-8-to-12`, `ack-tries-counter`,
`unacked-hold-bounded-by-ack-tries`, `beacon-relay-role-read-once`, plus 2.3.4's
`dreq-origin-second-airing`, `dreq-copy2-return-not-gated`, `dreq-forward-budget-reallocation`,
`per-wave-ceiling-removed`, `unacked-hold-timebox`/`30-to-33`, `wave-floor-cap-24-to-27`,
`campaign-budget-135-to-140`, `deadline-tick-source`, `dreq-origin-airings-constant-unused`,
`dack-sender-id-documented-dead`.

**Start with the dedupe-ring trio.** The v2.3.5 barren fix was caused by dedupe hits resetting the
barren run, and 2.3.6 changes that same ring's key width (32→16-bit XOR fold), size (24→48) and
clearing schedule (init-only → per campaign). Those interact directly and only one half is checked.
Specifically unverified: the ~1/1400 collision claim, what the ring's slots are shared between, and
whether a collision can do worse than "one relay not made".

## 8. Highest-value untaken action

`2.3.4`'s **DReq double-airing** is the change the field evidence most supports and it is already in
the field: §1 shows tags hear **zero** DReqs in two-thirds of campaigns, which is the dominant
failure, and a second copy per wave targets exactly that. Its mechanism (receive side counts forwards
per dreq id, so a node hearing both copies spends its existing two-relay budget on them) is
**unverified in code**.

## 9. Decensoring the round trip — the one measurement that unlocks the floor constants

Nothing at HEAD distinguishes "a node answered late" from "a node never answered". Log, at the
new-node detection point in `DeviceDiscovery.c`, one line per new union member with the offset from
that wave's DReq: `wave %u dreq=%08X newnode=%X off=%lu floor=%lu hops=%u rssi=%d`. Offset alone
settles it and costs no `.bss` (all stack). Then build the histogram of `off` per wave index. If no
new node ever arrives past 16 s, the 4000 step catches nothing. **Run it with the floor left wide**,
or the observation window is the one being tested rather than the one being defended.

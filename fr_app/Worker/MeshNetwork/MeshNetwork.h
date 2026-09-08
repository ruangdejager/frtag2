/*
 * MeshNetwork.h
 *
 * LoRa mesh network layer — types, packet definitions and public API.
 *
 * Uses CMSIS-RTOS v2 throughout.
 * All TickType_t references replaced with uint32_t (1 tick == 1 ms).
 */

#ifndef WORKER_MESHNETWORK_MESHNETWORK_H_
#define WORKER_MESHNETWORK_MESHNETWORK_H_

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

/* Build-configuration flags. These MUST be defined before anything that tests
 * them: MESH_DIAG_COUNTERS below selects MESH_MAX_NEIGHBORS, and when this
 * block sat further down the header the test silently took the production
 * branch - the diag build compiled, linked on the last 8 bytes of RAM, and
 * carried a 120-entry table it was supposed to have traded away. */
/*
 * Extra per-campaign diagnostic counters for a field-test build. OFF for
 * production, like MESH_LOG_VERBOSE above - but for a different reason. These
 * are not verbose (they add one aggregate line per campaign, nothing
 * per-packet); they cost RAM, and .bss is byte-exact full.
 *
 * What they exist to answer, none of which the production stats line can:
 *   - fwdBeacon vs fwdDreq/fwdAck/fwdTs: "forwarded=" is one lumped number, so
 *     beacon relay amplification - the term that should stop scaling with herd
 *     size now the dedupe ring holds 48 - cannot be separated from the rest.
 *   - dedupeHit: beacons correctly suppressed as already-seen. The direct read
 *     on whether the 16-bit fold bought real history.
 *   - bpSkipBeacon: beacon relays refused because the TX queue was already
 *     deep. The cleanest single measure of node saturation, and deliberately
 *     NOT in the production tally (see u16StatTxDropped, where counting
 *     back-pressure would make the number climb when things are working).
 *
 * Enabling this costs ~8 B of .bss, which does not fit alongside
 * MESH_MAX_NEIGHBORS at 120 - see there. */
#define MESH_DIAG_COUNTERS

/* ---- Timing constants ---- */
/* Beacon retry cadence while awaiting a D-Ack. Was one fixed 3500 ms period.
 *
 * A fixed period made an UNACKED node the loudest thing on the mesh for the
 * whole 205 s window: field logs show one hop-3 tag emitting 48 beacons in
 * 171 s, and the forwarder in front of it relaying 89 packets in the same
 * campaign. Every one of those transmissions costs the relay up to
 * MESH_TX_JITTER_MAX_MS of queue jitter plus up to LORA_TX_CARRIER_WAIT_MS of
 * carrier sense, and carrier sense runs the chip in CAD, not RX - so the relay
 * was deaf for the majority of the window and dropped both the D-Ack meant for
 * the node behind it and its own TimeSync. That is congestion collapse, and
 * the node beaconing hardest is what drives it.
 *
 * The cadence now backs off within one beaconing episode:
 *   interval(n) = min(BASE + n * STEP, MAX),  n = beacons already sent
 * so the first answers to a DReq are still prompt (that is when the primary is
 * listening and an ack is most likely) and the tail of an unheard node decays
 * to MAX instead of hammering. Same 171 s window: ~17 beacons instead of 48.
 * n resets to 0 every time beaconing (re-)starts - a new wave, a re-anchor
 * onto a newer dreq, or a re-arm - so each wave gets a fast first beacon; see
 * MESHNETWORK_vStartBeaconing.
 *
 * BASE is deliberately above both MESH_TX_JITTER_MAX_MS and
 * MESH_DREQ_FWD2_DELAY_MAX_MS (asserted in MeshNetwork.c) so a queued packet
 * still cannot slip past the NEXT beacon - the invariant the old single
 * constant carried. MAX is NOT bounded by MESH_DISCOVERY_IDLE_MS and cannot
 * be: the gap the primary observes is the period plus both ends' jitter and
 * carrier sense, up to ~6.5 s more, so no affordable idle window covers a 12 s
 * cadence. Wave end therefore no longer relies on beacon silence alone - see
 * MESH_DISCOVERY_IDLE_MS below. */
#define MESH_BEACON_BASE_MS           5000U
#define MESH_BEACON_STEP_MS           2000U
#define MESH_BEACON_MAX_MS            12000U

/* Primary D-Ack cadence. 2000 -> 4000: every tick emits one D-Ack carrying up
 * to MESH_MAX_ACK_IDS_PER_PACKET ids, but the sniffer logs show the packet was
 * never the constraint - across five campaigns not one D-Ack carried more than
 * 2 ids (81 with one id, 35 with two), because at 2 s the tick keeps outrunning
 * the arrival rate. Each of those near-empty acks is then re-flooded ~3x by the
 * forwarders. Holding twice as long roughly halves the ack packet count and
 * doubles the ids per packet for the same information.
 *
 * The extra hold is free only because MESH_BEACON_BASE_MS (5000) now exceeds
 * it: a node waiting one more ack period was going to be silent for that
 * period anyway, so no additional beacon is provoked. Do not raise this above
 * MESH_BEACON_BASE_MS - asserted in MeshNetwork.c. */
#define MESH_PRIMARY_ACK_INTERVAL_MS  4000U

/* Beacon silence that ends one DReq wave on the primary. 7000 -> 3000 -> 5000:
 * the history matters, because both previous values were wrong for the same
 * reason - beacon silence was the ONLY wave-end signal, so this one constant
 * had to serve as both "the herd has finished answering" and "nobody is still
 * mid-cadence", and no value does both.
 *
 * 7000 made 7 s of dead air the bulk of a ~100 s campaign. 3000 was below the
 * old 3500 ms beacon period, so a lone node's own inter-beacon gap
 * (3500 +/- jitter = 2.0..5.0 s) routinely read as silence and its wave was
 * declared over while it was still beaconing - visible in the sniffer logs as
 * 4 of 8 waves starting while beacons for the previous dreq were still
 * arriving, and at its worst as a campaign that ended after ONE wave and 3 s
 * with 0 neighbours while a 1-hop secondary was mid-cadence.
 *
 * Three changes remove the overload, and only then is a value pickable:
 *   - a wave cannot end before MESH_DISCOVERY_MIN_WAVE_MS (the DReq has to
 *     reach the air and a reply has to get back before silence means
 *     anything);
 *   - a wave does not end while the primary still has un-acked neighbours
 *     (MESHNETWORK_bHasUnackedNeighbors) - those are by definition nodes that
 *     are still beaconing, which is the condition this constant used to have
 *     to infer from timing and could not;
 *   - APP_PRIMARY_MIN_WAVES keeps the campaign alive through a barren wave.
 * So this is now only the tail timer: how long to keep a wave open after the
 * answers stop and everyone heard has been acked. 5000 covers one relay hop's
 * jitter at each end with margin.
 *
 * Must stay above MESH_TX_JITTER_MAX_MS (a beacon is queued with up to that
 * much jitter) - asserted in MeshNetwork.c. It deliberately does NOT try to
 * cover MESH_BEACON_MAX_MS; see there. */
#define MESH_DISCOVERY_IDLE_MS        5000U

/* Floor on one DReq wave. The wave clock starts when the DReq is ENQUEUED, and
 * between enqueue and air a packet waits out its own jitter (up to
 * MESH_TX_JITTER_MAX_MS), anything already queued ahead of it, and the radio
 * layer's carrier sense (up to LORA_TX_CARRIER_WAIT_MS, 5 s). The reply then
 * pays the same on the way back. With no floor, MESH_DISCOVERY_IDLE_MS could
 * elapse before the DReq had even been transmitted - which is exactly the 3 s
 * / 0 neighbour campaign in the field logs. 8000 covers both ends' jitter plus
 * a realistic carrier-sense wait; the absolute worst case is not covered by
 * any affordable floor, which is what the un-acked and min-wave rules above
 * are for. */
#define MESH_DISCOVERY_MIN_WAVE_MS    8000U

/* Added to the floor above for every ring of herd depth the primary has ALREADY
 * proven this campaign (MESHNETWORK_u8GetMaxDiscoveredWave).
 *
 * A flat floor is wrong because what it has to cover is not fixed: it is the
 * round trip out to the frontier and back, and that grows - superlinearly -
 * with depth. Measured at a sniffer beside the primary, across two campaigns:
 *
 *     ring   round trip (DReq sent -> first beacon back)
 *     1      2 s
 *     2      3 s
 *     3      4 s
 *     4      8 s     (2D94, hop 8)
 *     5      21 s    (241F, hop 9)
 *
 * Ring 5 is the datapoint that set these numbers. 241F's wave-5 beacon arrived
 * 21 s after the DReq - 5 s after a 16 s floor (8000 + 4*2000) had already
 * expired, the wave had been declared barren, and the campaign had moved on to
 * TimeSync. That tag, and the ring behind it, is exactly what the deep waves
 * exist to reach, so the floor has to cover the deep round trip or the wave
 * that finds the frontier ends while the frontier is still answering.
 *
 * Scaling means the wave looking for ring N+1 waits in proportion to how far
 * out ring N already is: with ring 4 on the table before wave 5 the floor is
 * 8000 + 4*4000 = 24000 ms (the cap below), and 241F's 21 s beacon lands with
 * 3 s to spare.
 *
 * 4000 (was 2000): the 2000 step covered the 2/3/4/8 s inner rings but left the
 * wave-5 floor at 16 s against a 21 s round trip. 4000 puts the floor ahead of
 * the measured edge and saturates at the cap by ring 4.
 *
 * A tight herd still pays little - one ring proven leaves the floor at
 * 12000 ms - which is the whole point of scaling it rather than raising the
 * flat value: the campaigns that would need a 24 s floor are exactly the ones
 * that never see it. APP_PRIMARY_CAMPAIGN_MAX_MS bounds the total either way. */
#define MESH_DISCOVERY_WAVE_ALLOWANCE_MS 4000U

/* Cap on the scaled floor, so no ring count can push the floor past
 * MESH_DISCOVERY_UNACKED_HOLD_MS and leave no room for the idle tail
 * underneath it. Asserted against both in MeshNetwork.c.
 *
 * 18000 -> 24000: the ring-5 round trip measured at 21 s (see the ALLOWANCE
 * comment above), so an 18 s cap left the deepest wave 3 s short of the very
 * tag it existed to hear. 24000 clears it; the ceiling and campaign budget
 * both moved to keep the idle tail fitting underneath.
 *
 * 24000 -> 27000: MESH_DREQ_ORIGIN_AIRINGS now airs the primary's DReq twice,
 * and a node rescued by copy 2 answers up to MESH_DREQ_FWD2_DELAY_MAX_MS
 * (2600 ms) later than one that heard copy 1. Everywhere but the deepest wave
 * the floor has seconds of slack to absorb that; at the cap it had 3 s, so
 * copy 2 would have left a ring-5 rescue 0.4 s of margin - which is the same
 * coin flip the flat floor used to lose, just one packet further along. 27000
 * puts the deep round trip AND its copy-2 rescue (21 + 2.6 = 23.6 s) back at
 * ~3.4 s, i.e. restores the margin the cap was chosen to give.
 *
 * Only the ring-5+ floor moves (24 -> 27 s); rings 1-4 ramp below the cap and
 * are untouched, so this is paid by the deepest wave of the deepest herds and
 * by nothing else. It does cost 3 s of MESH_WAVE_BUDGET_MS, which is why
 * APP_PRIMARY_CAMPAIGN_MAX_MS moved 135 -> 140 s with it. */
#define MESH_DISCOVERY_MIN_WAVE_CAP_MS  27000U

/* How long the UN-ACKED hold alone may keep a wave open. Formerly
 * MESH_DISCOVERY_WAVE_MAX_MS, a flat ceiling on the whole wave, which was
 * wrong in a way worth recording.
 *
 * Of the three reasons a wave stays open, only one can run forever. The listen
 * floor is bounded (MESH_DISCOVERY_MIN_WAVE_CAP_MS). Ongoing beacons are
 * self-limiting - the herd stops when it is acked. But "un-acked neighbours
 * exist" can stay true indefinitely: NEIGHBOR_vAddOrUpdate clears bAcked on
 * every re-beacon, so a node the primary can hear but whose acks never reach it
 * re-arms that condition forever. So THAT is what needs the time box, and it is
 * the only thing this bounds.
 *
 * As a ceiling on the whole wave it also cut off waves that were still being
 * answered, and the cost was not the one lost beacon. Every node mid-cadence
 * hears the next wave's DReq, takes the re-anchor branch in
 * MESHNETWORK_vHandleDReq, and MESHNETWORK_vStartBeaconing resets its
 * u8NodeBeaconSeq to 0 - so the entire mid-cadence herd's backoff collapses
 * back to MESH_BEACON_BASE_MS at once. That is precisely the congestion the
 * backoff exists to prevent, and the ceiling only ever fired when the un-acked
 * hold had kept the wave open, i.e. exactly in the congested case. A wave now
 * ends only once the air has actually gone quiet for MESH_DISCOVERY_IDLE_MS;
 * APP_PRIMARY_CAMPAIGN_MAX_MS is the single hard stop.
 *
 * Must exceed the scaled floor plus one idle tail, or the hold would expire
 * before the floor and quiet conditions it qualifies are even evaluable and the
 * un-acked gate would hold nothing at all: 27000 + 5000 = 32000, so 33000
 * keeps a ~1 s cushion. Asserted in MeshNetwork.c. */
#define MESH_DISCOVERY_UNACKED_HOLD_MS 33000U
/* Dedup window for BEACON and ACK message ids, and the bound on how many times
 * one beacon can be relayed across the herd. 24 -> 48 slots, at the same 96
 * bytes, by storing a 16-bit fingerprint per slot instead of the full 32-bit
 * id (see MESH_FP_FOLD below and ForwardRing_t).
 *
 * Why it had to grow. A beacon relay is gated by role, back-pressure, and this
 * ring - and unlike every other packet class it has no explicit airing cap
 * (contrast MESH_DREQ_MAX_FORWARDS, MESH_TIMESYNC_AIRINGS, MESH_DACK_AIRINGS).
 * The ring IS the cap: "relay each beacon once per node". That only holds while
 * the id is still in the ring when the next copy arrives.
 *
 * At 24 slots it stops holding at herd scale, and the arithmetic is not close.
 * The slots are shared between received beacon ids, received ack ids, and this
 * node's own beacon and ack ids. A 40-node herd pushes 80-120 beacon ids plus
 * the acks through a campaign, and the comment on MESH_DREQ_DEDUPE_SIZE already
 * records the consequence measured at 28 nodes: an id "is evicted within a
 * fraction of a second of busy traffic". That is why DReq ids were moved to
 * their own store; beacon and ack ids never got the same treatment. An evicted
 * beacon id is re-relayed by the same node when the next copy arrives, so the
 * per-node bound silently becomes per-node-per-lap.
 *
 * That matters more than it looks, because the forwarder population only ever
 * GROWS during a campaign: MESHNETWORK_bStopBeaconingLocked is the sole path
 * into NODE_ROLE_FORWARDER and nothing leaves it before vResetNodeRole. So
 * acking a node does not only silence a beaconer, it promotes a relay. Total
 * beacon airtime is roughly originations x (1 + forwarders): originations fall
 * as acks land while forwarders rise, so the product peaks mid-campaign rather
 * than decaying. At nine nodes one unit already relayed 89 packets and spent
 * ~70% of the window deaf in CAD, missing its own TimeSync. The ring is the
 * only thing standing between that and a multiplier that grows with the herd.
 *
 * 48 slots is ~3-5 s of history in the busy phase against ~1-2 s at 24, which
 * is what a relay needs to cover the spread of copies of one beacon (TX jitter
 * plus carrier sense at each hop).
 *
 * R5 (meshOptimise) wanted 64 slots at 32 bits, i.e. +128 B, which never fit
 * the RAM budget. Folding gets most of that for free instead. */
#define FORWARD_RING_SIZE             48U

/* 32-bit message id -> 16-bit ring fingerprint.
 *
 * Ids are (deviceId16 << 16) | counter16 (MESHNETWORK_u32GenerateGlobalMsgID),
 * so XOR-ing the halves mixes the device id INTO the stored value. Keeping the
 * low half alone would have been cheaper to read but collides systematically:
 * u16MsgCounter is seeded from the RNG per boot, so two devices can sit on
 * neighbouring counters for a whole deployment and shadow each other's every
 * packet. Folding makes a collision depend on both halves, i.e. random.
 *
 * A collision means a node treats an unseen message as seen: one relay not
 * made, or on the primary one beacon not recorded. The odds are ~48/65536 (~1
 * in 1400) for an arrival against a full ring, the node re-beacons regardless
 * so it is self-healing, and that rate sits far below the CRC and header error
 * rate already present in the field logs. Halving the stored width to double
 * the history is a good trade at 1-in-1400; it would not be at 1-in-20. */
#define MESH_FP_FOLD(id) ((uint16_t)(((uint32_t)(id) >> 16) ^ ((uint32_t)(id) & 0xFFFFU)))

/* DReq ids get their own dedup store rather than sharing the ring above. The
 * ring carries beacon and ack ids too, and one campaign pushes far more of
 * those through it than it has slots (28 nodes x up to 6 beacons per wave),
 * so a DReq id is evicted within a fraction of a second of busy traffic. That
 * is survivable while only forwarders relay DReqs, but the wave-1 flood has
 * every node relaying — an evicted id would let the same DReq be re-forwarded
 * on each lap around the mesh. 8 entries covers APP_PRIMARY_MAX_WAVES with
 * margin and cannot be displaced by beacon/ack churn. */
#define MESH_DREQ_DEDUPE_SIZE         8U

/* Forwards permitted per DReq id. A DReq is the one packet the whole campaign
 * hangs on: lose it and a node never learns a campaign is running, never
 * beacons, and is not counted for the rest of the window. One relay per node
 * gave that packet a single chance to survive a collision, so each id now gets
 * two — the dedupe store counts forwards instead of just remembering the id.
 *
 * The two copies are driven by two separate RECEPTIONS of the same id (a flood
 * arrives from several neighbours), so the spacing is whatever the mesh gives,
 * plus MESH_DREQ_FWD2_EXTRA_*_MS below to keep the pair off one another. A node
 * that only ever hears an id once still forwards once, exactly as before.
 *
 * This applies to both relay paths — the wave-1 flood and the forwarder relay
 * of waves 2+. What stays unchanged is WHO relays: wave 1 by every node, waves
 * 2+ only by forwarders. */
#define MESH_DREQ_MAX_FORWARDS        2U

/* Airings of the primary's OWN DReq, per wave. The constant above governs
 * RELAYS; this one governs the origination, and until now the two disagreed:
 * every node was allowed two copies of a DReq it passed on, while the primary
 * that started the wave sent exactly one. So the single most consequential
 * transmission of a wave was the only one in the protocol with no redundancy
 * left, and the TimeSync and D-Ack comments below, which both describe
 * themselves as using "the same two-copy scheme as a DReq origination", were
 * describing something that did not exist. They do now.
 *
 * What the lone copy costs when it is lost is the whole wave, not one node's
 * reception: the primary's transmission is the root of the flood, so nothing
 * downstream has anything to relay. On wave 1 that is the entire campaign's
 * "a campaign is running, stay awake" signal to the whole herd; on waves 2+ it
 * is the only thing that moves the frontier one ring further out. Every other
 * packet in a campaign now gets two chances - the beacon (cadence), the D-Ack
 * (MESH_DACK_AIRINGS), the TimeSync (MESH_TIMESYNC_AIRINGS), a relayed DReq
 * (MESH_DREQ_MAX_FORWARDS). This closes the last single-copy path.
 *
 * Mechanism is identical to those: one encode, two enqueues - ordinary jitter
 * for copy 1, MESH_DREQ_FWD2_DELAY_[MIN,MAX] for copy 2, so the pair cannot
 * share one congestion window or one fade.
 *
 * NO AMPLIFICATION, and this is the part worth being sure of. Both copies
 * carry the same dreq id, and the receive side is governed by a per-id FORWARD
 * COUNT (DREQ_bClaimForward), not by "have I seen this". So a node that hears
 * both copies spends its existing two-relay budget on them instead of on one
 * primary copy plus one peer's relay: the number of relays it emits is
 * unchanged. The only added airtime in the whole mesh is the primary's own
 * extra ~10-byte transmission, once per wave. Nor can copy 2 disturb a node
 * that already answered copy 1 - MESHNETWORK_vStartBeaconing refuses a restart
 * for the dreq id it is already beaconing, so the backoff cadence and the
 * latched trigger RSSI both survive it untouched.
 *
 * The one place this needed paying for: a node that hears ONLY copy 2 answers
 * up to MESH_DREQ_FWD2_DELAY_MAX_MS (2600 ms) later than one that heard copy 1,
 * and the wave-listen floor has to cover that. Everywhere but the deepest wave
 * it has seconds of slack; at MESH_DISCOVERY_MIN_WAVE_CAP_MS it had 3 s, so a
 * ring-5 rescue would have landed 0.4 s inside the floor - a coin flip, and
 * the deeper the ring the likelier it is to be lost, since the round trip grows
 * superlinearly while the cap does not. The cap was therefore raised 24 -> 27 s
 * (and the campaign budget 135 -> 140 s) so the deep rescue keeps the same ~3 s
 * margin the direct reception has. Raising the cap is the correct knob and the
 * only one: the base and the per-ring allowance already have slack at every
 * ring that is not at the cap. */
#define MESH_DREQ_ORIGIN_AIRINGS      2U

/* 120 (was 128): freed 160 B of .bss for the superOptimise fixes on a part
 * whose RAM was byte-exact full. Still far beyond a realistic per-primary
 * fleet — D-Acks carry MESH_MAX_ACK_IDS_PER_PACKET ids per
 * MESH_PRIMARY_ACK_INTERVAL_MS (12 per 4 s), so even 120 nodes need ~40 s of
 * ack airtime per campaign. */
#ifdef MESH_DIAG_COUNTERS
/* The diag counters need ~8 B that .bss does not have, so a field-test build
 * buys them by capping the table lower. 64 still clears a 30-50 node herd with
 * headroom, and each entry saved is 24 B of .bss plus 24 B of the stack array
 * DeviceDiscovery allocates. If a test herd ever exceeds 64 this has to be
 * rethought rather than quietly truncating the union. */
#define MESH_MAX_NEIGHBORS            64
#else
#define MESH_MAX_NEIGHBORS            120
#endif

/* A beaconing node used to stop (become forwarder) after MESH_MAX_BEACONS_PER_
 * CAMPAIGN (6) beacons, on the theory that it bounded airtime when a D-Ack was
 * silently lost. What it actually bounded was the node's chance of being found:
 * six beacons is ~17.5 s of a 205 s window, after which a node the primary was
 * still hunting for went mute and was not counted. A per-wave re-arm was bolted
 * on to claw some of that back, which only made the giving-up harder to follow.
 *
 * A secondary now beacons until it is ACKED, or until the campaign's 205 s hard
 * cap (APP_DISCOVERY_WINDOW_TIMEOUT_MS in DeviceDiscovery.h) ends the window —
 * that cap was always the real bound and is now the only one. The ack path
 * (MESHNETWORK_vStopBeaconingByOrigin) is unchanged, so a node that IS heard
 * still stops on the first ack and costs no more airtime than before.
 *
 * (A parallel 30 s duration cap once sat alongside the count; at the 3.5 s
 * beacon interval 6 beacons take ~21 s, so the count always won and the timer
 * never once decided anything. It was already removed.)
 *
 * What DID need bounding was the RATE, not the count - see the backoff at
 * MESH_BEACON_BASE_MS above. An unheard node still beacons to the end of the
 * window, but at a decaying cadence, so it keeps its chance of being found
 * without deafening the relay in front of it. */

/* TX jitter window - wide enough to de-correlate many nodes answering one DReq.
 * Max stays well under MESH_BEACON_BASE_MS (the SHORTEST beacon interval, and
 * therefore the binding one) so a jittered beacon never slips past the next
 * interval. Asserted in MeshNetwork.c. */
#define MESH_TX_JITTER_MIN_MS         20U
#define MESH_TX_JITTER_MAX_MS         1500U

/* Ceiling the jitter window GROWS to when the node finds itself in a crowd,
 * and the step it grows by per DReq copy heard. See
 * MESHNETWORK_u32GetTxJitterMs.
 *
 * A fixed 1500 ms window is sized for a handful of answerers and collapses at
 * herd scale. 40 nodes answering one DReq is 40 x ~33 ms = ~1.3 s of
 * transmission offered into a 1.5 s window - ~88% load, and collision
 * probability climbs roughly with the square of that. Carrier sense keeps it
 * CORRECT, but it does so by serialising the contention, and carrier sense runs
 * the chip in CAD rather than RX (LORA_CAD_ONLY, no IRQ_RX_DONE mapped). So the
 * cost of a too-narrow window is not lost packets, it is DEAFNESS - the
 * documented cause of one unit missing the primary's TimeSync in 39 of 47
 * campaigns. Widening the window attacks that directly: fewer nodes attempt at
 * once, so fewer sit in CAD.
 *
 * 4000 is chosen as the largest value that needs no other constant to move.
 * Both existing invariants still hold with 1000 ms of margin each:
 *   MESH_BEACON_BASE_MS (5000)   > 4000  - a queued packet cannot slip past
 *                                         the next beacon
 *   MESH_DISCOVERY_IDLE_MS (5000) > 4000 - a jittered beacon cannot be
 *                                         mistaken for silence
 * Both are asserted in MeshNetwork.c. At 40 nodes this is ~1.3 s into a 4 s
 * window, ~33% offered load instead of ~88%.
 *
 * The density signal is u16StatDReqHeard, not beacons heard, and the choice
 * matters: on wave 1 every node relays the DReq twice, so a node in a dense
 * herd hears many copies within the first seconds - BEFORE it answers. A beacon
 * count only rises after the burst it is supposed to spread. STEP 60 reaches
 * the ceiling at ~42 DReq copies heard, which is the right order for a 40-node
 * herd; a lone node stays at 1500 and loses nothing.
 *
 * Known limit: the FIRST beacon of an episode is fired immediately on the
 * trigger, when the local count may still be low, so it is the one transmission
 * this cannot spread. Beacons 2..n get the full window. Fixing that needs the
 * primary to advertise herd size in a DReq hint byte - there is room and the
 * length-gated field pattern is established - but that is a wire change and is
 * deliberately not done here. */
#define MESH_TX_JITTER_BUSY_MS        4000U
#define MESH_TX_JITTER_STEP_MS        60U

/* Send delay for the SECOND forward of a DReq id. REPLACES the normal jitter
 * window above for that one packet (it is not added to it), so the two copies
 * cannot land inside one congestion window — a collision that killed copy 1 is
 * then unlikely to kill copy 2, which is the entire point of sending two.
 *
 * The window deliberately does not overlap [MIN, MAX] above, so the pair is
 * always separated by at least (1600 - 1500) = 100 ms and typically ~1 s. The
 * ceiling matters: the mesh TX queue is drained in order and each item blocks
 * on its own ready-tick, so a deferred forward holds up whatever is queued
 * behind it. 2600 ms worst case stays under MESH_BEACON_BASE_MS, keeping the
 * existing guarantee that a queued beacon never slips past its next interval.
 * Asserted in MeshNetwork.c. */
#define MESH_DREQ_FWD2_DELAY_MIN_MS   1600U
#define MESH_DREQ_FWD2_DELAY_MAX_MS   2600U

/* TimeSync is aired TWICE per node - once by the primary that originates it and
 * once by every node that relays it - using the same two-copy scheme as a DReq:
 * the ordinary jitter for copy 1 and the non-overlapping window above for
 * copy 2. Same reasoning, and a stronger case for it: TimeSync is the one
 * packet that ends a secondary's campaign, sets its clock and tells it a
 * firmware image is waiting, and it goes out exactly once per campaign, at the
 * moment the mesh is at its noisiest. Field logs: 5 of 21 device-wakes never
 * saw it, and in the worst campaign none of the three secondaries did.
 *
 * The receive-side dedup is untouched - it is keyed on the UTC value, so a node
 * decodes one TimeSync per campaign no matter how many copies reach it, and
 * still relays exactly two. No fabricated timestamp, no second origination
 * path, no extra state: total airings are bounded at 2 per node per UTC, the
 * same budget MESH_DREQ_MAX_FORWARDS already grants a DReq. */
#define MESH_TIMESYNC_AIRINGS         2U

/* A primary D-Ack is aired TWICE, the same two-copy scheme as a DReq/TimeSync
 * origination: the ordinary jitter places copy 1, MESH_DREQ_FWD2_DELAY_[MIN,MAX]
 * places the non-overlapping copy 2, so a collision that eats one copy is
 * unlikely to eat the other.
 *
 * The case is strongest for the FIRST ack of a campaign, which is uniquely
 * unprotected. A D-Ack is relayed only by nodes already in NODE_ROLE_FORWARDER,
 * and a node becomes a forwarder only once it has itself been acked - so at the
 * instant the first ack goes out no forwarders exist yet, and it gets a single,
 * unrelayed airing. That lone packet is the one that silences the whole first
 * ring; a field log showed it lost, and the first ring re-beaconed until the
 * second ack (which by then had relays behind it) finally landed.
 *
 * No amplification: both copies carry the same u32AckMsgId, which the receive-
 * side dedup (MESHNETWORK_vHandleDAck) and the primary's own FORWARD_vAdd key
 * on, so every node applies one ack and relays a given id once regardless of how
 * many copies reach it. The extra airing only ever reaches the primary's direct
 * earshot - exactly the ring that has no relay support - and costs one extra
 * primary transmission (~30 ms) per ack. Total airings stay bounded at 2. */
#define MESH_DACK_AIRINGS             2U

/*
 * Per-packet verbose text logging. During a campaign every node hears every
 * neighbour's (re)transmissions, so the per-packet plumbing lines — the
 * "Queued TX"/"TX (len=)" duplicates of "Transmitting ..." and the
 * "... seen before" dedup hits — multiply O(N) across the fleet and are the
 * dominant flash-log/ring-buffer noise. Off by default: the event narrative
 * (received/forwarded/sent, beacon start/stop, timesync applied) and a single
 * "Transmitting <type> len=" per TX are still logged. Define this for bench
 * debugging of the TX queue / dedup behaviour.
 */
// #define MESH_LOG_VERBOSE


/* ---- Packet types (wire, first byte) ---- */
typedef enum {
    MeshPktType_Reserved  = 0,
    MeshPktType_DReq      = 1,
    MeshPktType_DBeacon   = 2,
    MeshPktType_DAck      = 3,
    MeshPktType_TimeSync  = 4,
    MeshPktType_FrKernel  = 5,   /* FrKernel command / response */
    /* OTA firmware distribution (DIRECT LoRa, never mesh-forwarded) —
     * formats and state machines in fr_app/Worker/OtaUpdate. */
    MeshPktType_OtaPrep    = 6,  /* primary bcast: session announcement    */
    MeshPktType_OtaPrepAck = 7,  /* secondary: joins the session           */
    MeshPktType_OtaChunk   = 8,  /* primary bcast: one image chunk         */
    MeshPktType_OtaPoll    = 9,  /* primary: request one target's report   */
    MeshPktType_OtaReport  = 10, /* secondary: missing bitmap / verdict    */

    MeshPktType_BasicBeacon = 11 /* secondary in basic mode: TX-only beacon
                                   * at ~10 s (± jitter). Carries cached
                                   * last-known GPS + age (no live fix),
                                   * no dreq/hops/wave. See
                                   * MeshPktBasicBeacon_t below. */
} MeshPktType_e;

/* ---- Wake-up interval enum ---- */
typedef enum {
    WAKEUP_INTERVAL_15_MIN  = 1,
    WAKEUP_INTERVAL_30_MIN  = 2,
    WAKEUP_INTERVAL_60_MIN  = 3,
    WAKEUP_INTERVAL_120_MIN = 4,
    WAKEUP_INTERVAL_240_MIN = 5,
    WAKEUP_INTERVAL_MAX_COUNT = 5
} WakeupInterval;

/* ---- Discovery mode enum ----
 *   ADVANCED: full mesh campaign — DReq waves, per-node beacon-on-DReq,
 *             forwarding. Original behavior.
 *   BASIC   : each secondary independently TX-beacons at ~10 s (± jitter);
 *             the primary passively listens for 60 s every 15 min and
 *             accumulates a RAM store, flushed to the fr9 at each
 *             WakeupInterval boundary. No DReq campaign, no forwarding.
 *
 * The mode is a system-wide setting owned by fr9 (movementAlarm.
 * nightZoneLevels.holdSecond.value), fetched by the primary via
 * AT+SETREQ, and distributed to secondaries in every TimeSync — a
 * secondary applies the received mode before its next scheduled wake.
 * Cold-boot default on every node is ADVANCED (safe fallback with full
 * mesh info) until the first TimeSync flips it. */
typedef enum {
    DISCOVERY_MODE_ADVANCED = 0,
    DISCOVERY_MODE_BASIC    = 1
} DiscoveryMode_e;

/* ---- On-wire packet structs ---- */
typedef struct {
    uint32_t u32DreqId;
    uint32_t u32DeviceId;
    uint16_t u16BatMv;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint32_t u32BeaconMsgId;
    uint8_t  dreqWaveDisc;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* sender's VERSION_SW_PATCH — logged per-neighbor
                              * so "did unit X pick up the latest OTA" is
                              * answerable from the fr9 flash log without
                              * touching each device with `tag <ID> fwver`. */
    /* Which node's DReq produced i16Rssi above. i16Rssi is the reading of the
     * ONE DReq that triggered this beaconing episode - not the strongest heard
     * during it (that used to drift as later, unrelated receptions came in;
     * it is now latched at the trigger and frozen — see bBestDreqLatched in
     * MeshNetwork.c) - and the DReq that gave it may have come from the
     * primary directly or from any relaying peer, so the reading alone does
     * not say where this node's good path actually is. Carried as the 16-bit
     * device id of the immediate sender of that DReq (see MESH_DREQ_LEN_SRC).
     *
     * bValid is a separate flag rather than "u16 != 0" because 0x0000 is a
     * reachable device id, and because a beacon relayed by an older forwarder
     * arrives with the field genuinely absent — which must not be reported as
     * a real id of zero. */
    bool     bBestRssiSrcValid;
    uint16_t u16BestRssiSrcId;
    bool     bGpsValid;      /* true if i32Lat/Lon hold a real fix */
    int32_t  i32LatUDeg;     /* latitude  in microdegrees (10^-6 deg) */
    int32_t  i32LonUDeg;     /* longitude in microdegrees (10^-6 deg) */
} MeshPktDBeacon_t;

/* 8 -> 12. At 8 ids per MESH_PRIMARY_ACK_INTERVAL_MS the primary can silence
 * only 2 nodes/s, so a 40-50 node herd needs 20-25 s of ack passes before the
 * last ring is even addressed - and every node still un-acked is still
 * beaconing over the top of the ones that are. 12 ids puts a full D-Ack at
 * 10 + 4*12 = 58 B, still inside MESH_TX_MAX_PACKET_SIZE (64) with the
 * _Static_assert in MeshNetwork.c holding the line, and cuts the passes by a
 * third for 16 bytes of wire on a packet that is mostly header anyway. */
#define MESH_MAX_ACK_IDS_PER_PACKET 12
typedef struct {
    uint32_t u32AckMsgId;
    uint32_t u32DreqId;
    /* NOT on the wire. MESHNETWORK_bEncodeDAck never writes it, and the
     * receiver never looks for it: the acking primary is identified by the top
     * half of u32DreqId instead. Populated on the send path out of habit. Kept
     * only so the struct layout is not disturbed; do not start trusting it
     * without adding it to the encoder AND a length gate on the decoder. */
    uint32_t u32SenderId;
    uint8_t  u8AckCount;
    uint32_t u32AckedIds[MESH_MAX_ACK_IDS_PER_PACKET];
} MeshPktDAck_t;

/* Basic-mode beacon (see MeshPktType_BasicBeacon). Populated on the
 * secondary from cached last-known GPS + its age (no live fix); no dreq,
 * no hops, no wave. u32BeaconMsgId is monotonic per-boot per-device and
 * lets the primary's RAM store discard stale re-arrivals (newer-msgid
 * wins). */
typedef struct {
    uint32_t u32DeviceId;
    uint32_t u32BeaconMsgId;
    uint16_t u16BatMv;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* sender's VERSION_SW_PATCH */
    bool     bGpsValid;      /* true if i32Lat/Lon hold a cached fix */
    int32_t  i32LatUDeg;     /* latitude  in microdegrees (10^-6 deg) */
    int32_t  i32LonUDeg;     /* longitude in microdegrees (10^-6 deg) */
    uint32_t u32GpsAgeS;     /* age in seconds of that cached fix (only
                              * meaningful when bGpsValid) */
    int16_t  i16Rssi;        /* only set on receive */
} MeshPktBasicBeacon_t;

typedef struct {
    uint32_t     u32UtcTimestamp;
    WakeupInterval tWakeupInterval;
    uint32_t     u32StagedFwVersion;  /* MMmmpp of the image in the primary's
                                        * ext-flash scratchpad, 0 if none
                                        * valid — see FOTA_bGetMeta(). Lets
                                        * every secondary that hears the
                                        * end-of-campaign TimeSync learn
                                        * whether an update is available
                                        * without a dedicated OtaPrep round
                                        * trip. */
    DiscoveryMode_e eMode;            /* system-wide discovery mode — see
                                        * DiscoveryMode_e above */
    bool         bGpsEnabled;         /* system-wide GPS-active flag. false
                                        * turns every GPS_vRequestFix() into
                                        * a no-op (cached last-known fix
                                        * keeps aging). */
} MeshPktTimeSync_t;

/* ---- Forward ring ---- */
/* Slots hold a 16-bit FINGERPRINT of the message id, not the id, which is what
 * buys 48 slots out of the same 96 bytes the old 24 x uint32 occupied - see
 * FORWARD_RING_SIZE and MESH_FP_FOLD. */
typedef struct {
    uint16_t u16Ring[FORWARD_RING_SIZE];
    uint8_t  u8Head;
    uint8_t  u8Count;
} ForwardRing_t;

/* ---- Neighbor entry (internal) ----
 * Field order is load-bearing. The declared fields below occupy 21 bytes and
 * the struct is 24 (verified against .bss.tNeighborTable = 0xb40 = 120 x 24 in
 * the linked map), so there are 3 bytes of tail padding the compiler inserts
 * whatever we do. u16BestRssiSrcId is placed last deliberately: it lands at
 * offset 22 inside that existing padding and sizeof stays 24, which is the only
 * reason this field is affordable at all.
 *
 * That headroom is now spent. .bss is byte-exact full on this part — the linked
 * image leaves 8 bytes of the 64K RAM region — so a SECOND 2-byte field here,
 * or widening this one to uint32_t, pushes sizeof to 28 and costs 480 B, which
 * will not link. The static assert in MeshNetwork.c is what catches that. */
typedef struct {
    uint32_t u32DeviceId;
    int32_t  i32LatUDeg;     /* latitude  in microdegrees */
    int32_t  i32LonUDeg;     /* longitude in microdegrees */
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8HopCount;
    uint8_t  u8DreqWaveDiscovered;
    uint8_t  u8FwPatch;        /* sender's VERSION_SW_PATCH from beacon */
    uint8_t  u8MoveState : 1;  /* 0 = moving, 1 = still */
    uint8_t  bGpsValid   : 1;  /* 1 = i32Lat/LonUDeg hold a fix */
    /* D-Acks this campaign that have LISTED this node, saturating at 3. Free:
     * it takes two of the six bits left over in the byte the two flags above
     * already occupy, so sizeof is unchanged and the 24-byte assert still
     * holds. See MESH_ACK_TRIES_MAX for what it is for. */
    uint8_t  u8AckTries  : 2;
    bool     bAcked;
    uint16_t u16BestRssiSrcId; /* node whose DReq gave i16Rssi; 0 = not reported
                                * (older peer, or relayed via an older node) */
} NeighborEntry_t;

/* ---- Node role ---- */
typedef enum {
    NODE_ROLE_UNKNOWN   = 0,
    NODE_ROLE_BEACONING = 1,
    NODE_ROLE_FORWARDER = 2
} NodeRole_e;

/* ---- Device role ---- */
typedef enum {
    DEVICE_ROLE_UNKNOWN   = 0,
    DEVICE_ROLE_PRIMARY   = 1,
    DEVICE_ROLE_SECONDARY = 2
} DeviceRole_e;

/* ---- Discovered neighbor (application interface) ----
 * Callers stack-allocate MESH_MAX_NEIGHBORS of these (DeviceDiscovery.c), so
 * sizeof is a 120x stack cost, not just a RAM one. u16BestRssiSrcId sits after
 * bGpsValid to fall in the 2-byte hole already there ahead of i32LatUDeg;
 * sizeof stays 24. */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8Wave;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* neighbor's VERSION_SW_PATCH from beacon */
    bool     bGpsValid;
    uint16_t u16BestRssiSrcId; /* node whose DReq gave i16Rssi; 0 = not reported */
    int32_t  i32LatUDeg;     /* latitude  in microdegrees */
    int32_t  i32LonUDeg;     /* longitude in microdegrees */
} MeshDiscoveredNeighbor_t;

/* ---- Public API ---- */
void MESHNETWORK_vInit(void);
void MESHNETWORK_vParserTask(void *pvParameters);

bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId);
void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval,
                               uint32_t u32StagedFwVersion,
                               DiscoveryMode_e eMode,
                               bool bGpsEnabled);

/* Discovery-mode + GPS-enable state. Set by the primary from the fr9's
 * AT+SETREQ response and by every secondary from the last received
 * TimeSync. Getters return the currently-applied value; cold-boot default
 * is ADVANCED + GPS enabled. */
void            MESHNETWORK_vSetDiscoveryMode(DiscoveryMode_e eMode);
DiscoveryMode_e MESHNETWORK_eGetDiscoveryMode(void);
void            MESHNETWORK_vSetGpsEnabled(bool bEnabled);
bool            MESHNETWORK_bGetGpsEnabled(void);

/* Stop whatever dreq this node is currently beaconing (secondary campaign end —
 * the caller doesn't know the node's internal beacon dreq id). */
void MESHNETWORK_vStopBeaconingSelf(void);
bool MESHNETWORK_bIsBeaconing(void);     /* true while this node is beaconing */

/* True once this node has heard a DReq this campaign — directly or relayed,
 * any wave. Positive proof that a campaign is running and that this node is
 * inside its footprint, which is what the wave-1 flood exists to deliver to
 * nodes the frontier has not reached yet. DeviceDiscovery uses it to keep the
 * radio on instead of timing out on APP_SECONDARY_SILENCE_MS while it waits
 * for its ring's turn. A node that has heard nothing keeps the old behaviour
 * exactly, so out-of-range units cost no extra power. Cleared by
 * MESHNETWORK_vResetNodeRole() at campaign end. */
bool MESHNETWORK_bCampaignHeard(void);
/* Drop pending TX. Called on campaign wake and again at campaign end.
 * bKeepTimeSync re-queues a pending TimeSync instead of dropping it: pass true
 * at campaign end (a secondary's campaign ENDS on the TimeSync it must still
 * relay), false on the wake flush (a relay that survived deep sleep would air
 * a 15-minute-old timestamp). See the implementation. */
void MESHNETWORK_vFlushTxQueue(bool bKeepTimeSync);

/* Primary: does the neighbour table hold anyone this campaign has not put in a
 * D-Ack yet? True means at least one node is still beaconing at us and has no
 * reason to stop, so the wave it answered must not be declared over - that is
 * the direct signal MESH_DISCOVERY_IDLE_MS used to have to guess at from
 * timing. NEIGHBOR_vAddOrUpdate clears bAcked on every re-beacon, so this also
 * goes true again for a node whose acks are not reaching it. */
/* How many D-Acks may list a node before the wave-listen loop stops treating
 * it as a reason to hold the wave open. Bounded by the 2-bit field, so 3 max.
 *
 * NEIGHBOR_vAddOrUpdate clears bAcked on every re-beacon - correctly, because a
 * node that is still beaconing is asking to be acked again. But nothing bounded
 * that: a node the primary can hear whose acks never reach it, or one beaconing
 * for a DIFFERENT primary (both primaries in a herd wake on the same UTC slot,
 * so this is the normal case, not an edge one), re-arms the condition forever.
 * MESHNETWORK_bHasUnackedNeighbors() was then permanently true and every wave
 * burned its full MESH_DISCOVERY_UNACKED_HOLD_MS waiting for an ack that cannot
 * land, which at 30-50 nodes is every wave of every campaign.
 *
 * Three tries, then the wave stops waiting for that node. The primary keeps
 * LISTING it in later acks regardless - that costs 4 bytes in a packet that is
 * already going out, and one ack that finally lands still silences 20+ beacons.
 * What the counter bounds is only how long the wave defers to it. */
#define MESH_ACK_TRIES_MAX            3U

bool MESHNETWORK_bHasUnackedNeighbors(void);

/* Primary: how many unique nodes are in the neighbour table right now. Cheap
 * enough to poll (no table copy, unlike MESHNETWORK_bGetDiscoveredNeighbors),
 * which is what the wave loop needs to tell "this wave is still discovering
 * nodes" from "this wave is hearing the same nodes re-beacon". */
uint16_t MESHNETWORK_u16GetNeighborCount(void);

/* Beacons heard so far this campaign (packets, duplicates and relays included -
 * it is incremented ahead of the dedupe). Snapshotting it at each wave boundary
 * is how the per-wave log line reports beacons-per-wave accurately, rather than
 * counting the 500 ms polls that noticed a change and undercounting bursts. */
uint16_t MESHNETWORK_u16GetBeaconsHeard(void);

/* Primary: how many table rows are still un-acked. The bool sibling above
 * short-circuits and is what the poll loop uses; this one is for the per-wave
 * log line. */
uint16_t MESHNETWORK_u16GetUnackedCount(void);

/* Primary: one line naming every row this campaign never managed to ack, with
 * its ack-try count. Directly measures whether MESH_ACK_TRIES_MAX and the
 * cross-primary ack are doing their job; call at campaign end. */
void MESHNETWORK_vLogUnackedNeighbors(void);

/* The TX jitter ceiling currently in force on this node - grows with local DReq
 * density, see MESH_TX_JITTER_BUSY_MS. Logged in the campaign stats line so the
 * field logs record the window that produced them. */
uint32_t MESHNETWORK_u32GetTxJitterCeilingMs(void);

/* Primary: the highest wave number that has discovered anything this campaign,
 * i.e. how many rings deep the herd has PROVEN to be; 0 before anything
 * answers. The wave loop scales its listen floor by this - see
 * MESH_DISCOVERY_WAVE_ALLOWANCE_MS.
 *
 * Reads u8DreqWaveDiscovered, which the table keeps at the FIRST wave that
 * found each node, so this is a true ring count. Deliberately NOT the hop
 * count: a beacon's hop field is the depth of whichever relayed COPY arrived
 * last, which routinely overshoots the ring - a 1-ring node's beacon reaches
 * the primary at hop 2, a 4-ring node's at hop 6-8 - and scaling a timer by an
 * inflated number would make every tight herd pay for a spread one. */
uint8_t MESHNETWORK_u8GetMaxDiscoveredWave(void);

/* TX-worker hooks for the runtime radio range test (RadioTestMode.c), whose
 * beaconing secondary has no task of its own and runs on the TX worker.
 * bTxWorkerReady lets it refuse to enter where there is no worker to run on
 * (an ENABLE_RADIO_TEST build skips MESHNETWORK_vInit); vWakeTxWorker breaks
 * the worker out of the indefinite wait it is in by the time the test's own
 * gates have silenced every other path to it. */
bool MESHNETWORK_bTxWorkerReady(void);
void MESHNETWORK_vWakeTxWorker(void);
bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer,
                                         uint16_t u16MaxEntries,
                                         uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearDiscoveredNeighbors(void);

/* Basic-mode beacon TX (secondary). Reads local BAT / MOVE / VERSION_SW_PATCH
 * / GPS_bGetLastKnownFix and TX-only broadcasts a MeshPktType_BasicBeacon.
 * No RX, no wait — sends and returns. Called from the basic-mode 10 s
 * wake in DeviceDiscovery. */
void MESHNETWORK_vSendBasicBeacon(void);

/* Basic-mode primary RAM store: one entry per unique DeviceId, updated
 * on each MeshPktType_BasicBeacon received during the 15-min listen
 * window (newer BeaconMsgId wins, older is ignored). Flushed to the fr9
 * at each WakeupInterval boundary via FARMRANGER_bLogBasicData and then
 * cleared. Column set differs from the advanced-mode neighbor table (no
 * hops / wave / RSSI, has GPS-age) so it uses its own struct + row
 * format on the Farmranger side. */
/* Field order + the u8MoveState/bGpsValid bitfields keep this at 24 bytes
 * (vs. 28 with naive ordering) — same tight-packing this file already
 * applies to NeighborEntry_t, needed because this part's RAM is byte-exact
 * full (see FORWARD_RING_SIZE's comment above). tBasicNeighborTable
 * [MESH_MAX_BASIC_NEIGHBORS] makes every byte here cost 32x. Adding
 * i16Rssi bumped this from 24 to 28 bytes (worst-case pad). */
typedef struct {
    uint32_t u32DeviceId;
    uint32_t u32BeaconMsgId;  /* update key: incoming replaces stored only
                                * when incoming > stored */
    int32_t  i32LatUDeg;
    int32_t  i32LonUDeg;
    uint32_t u32GpsAgeS;
    uint16_t u16BatMv;
    int16_t  i16Rssi;         /* best (least-negative) RSSI heard across
                                * all beacons from this device this cycle */
    uint8_t  u8FwPatch;
    uint8_t  u8MoveState : 1;
    uint8_t  bGpsValid   : 1;
} MeshBasicNeighbor_t;

/* Cap on the basic-mode RAM store (small deliberately — basic mode is
 * for small remote groups). Callers stack-allocate a copy at this size
 * before calling MESHNETWORK_bGetBasicNeighbors, so it lives in the
 * public header. */
#define MESH_MAX_BASIC_NEIGHBORS    32U

bool MESHNETWORK_bGetBasicNeighbors(MeshBasicNeighbor_t *pBuffer,
                                    uint16_t u16MaxEntries,
                                    uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearBasicNeighbors(void);

/* Cheap count of unique nodes currently in the basic-mode RAM store —
 * avoids stack-copying the whole table just to log a count. */
uint16_t MESHNETWORK_u16GetBasicNeighborCount(void);

uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void);

void          MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval);
WakeupInterval MESHNETWORK_tGetWakeupInterval(void);
uint8_t       MESHNETWORK_u8GetWakeupInterval(void);

/* Called by the parser when a FrKernel packet arrives.
 * Override in FrKernel.c when FRKERNEL_INTERFACE_LORA is defined. */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len);

/* Enqueue a small OTA response (PrepAck/Report, <= the mesh TX item size)
 * through the jittered mesh TX queue. Parser-context safe — used by the
 * OtaUpdate packet handlers to answer the primary without a new TX path. */
bool MESHNETWORK_bSendOtaResponse(const uint8_t *buf, uint16_t len);

uint32_t MESHNETWORK_u32GetLastBeaconHeardTick(void);
uint32_t MESHNETWORK_u32GetLastDiscoveryPktTick(void);  /* any DReq/DBeacon/DAck/TimeSync */

uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void);
void     MESHNETWORK_vUpdatePrimaryLastSeen(void);

void MESHNETWORK_vStartPrimaryAck(void);
void MESHNETWORK_vStopPrimaryAck(void);

/* Reset per-wake "first TimeSync only" gate.
 * Call at the top of each wake cycle from DeviceDiscovery. */
void MESHNETWORK_vResetTimeSyncAccepted(void);

void MESHNETWORK_vResetNodeRole(void);
void MESHNETWORK_vIncrDreqWaveCnt(void);
void MESHNETWORK_vResetDreqWaveCnt(void);

/* One-line traffic summary (DReq/beacons/acks heard, messages forwarded)
 * since the last MESHNETWORK_vResetDreqWaveCnt() — call at campaign end. */
void MESHNETWORK_vLogCampaignStats(const char *pcTag);

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */

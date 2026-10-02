# What's supported

This page covers what I've printed, what's enabled but untested, and what doesn't work yet.

## Printers

It should work on any printer with two or more nozzles, as long as it uses the same kind of prime tower as the Bambu printers, and it uses two of those nozzles per print. It isn't tied to one printer model, but I've only been able to print on my own.

| Printer | Status | Notes |
|---|---|---|
| Bambu Lab H2D | Print-tested | Everything was built and tested on this printer. |
| Bambu Lab H2D Pro | Enabled, not print-tested | Same flush handling as the H2D. |
| Bambu Lab H2C | Enabled, not print-tested | See the H2C note under known issues. |
| Bambu Lab X2D | Enabled, not print-tested | The second extruder is Bowden. The MN presets carry Bowden speed columns held to Bambu's own limits. |
| Snapmaker J1 (IDEX) | Enabled, not print-tested | Feature Split and Body Split checked without a printer. |
| Prusa XL (2 to 5 toolheads) | Enabled, not print-tested | Uses two of its toolheads, picked on step 2. Feature Split and Body Split checked. |
| Snapmaker U1 | Enabled, not print-tested | Uses two of its four toolheads. Feature Split and Body Split checked. |
| RatRig V-Core 4 IDEX, Volumic IDRE, Snapmaker A350 Dual and Artisan, Z-Bolt Dual, Lulzbot Taz Pro Dual | Enabled, not print-tested | Feature Split checked. Body Split runs on the same engine path but I haven't checked it on these. |
| Flashforge Creator 5, WonderMaker ZR Ultra, custom toolchanger profiles | Enabled, not print-tested | Two toolheads, Feature Split checked. Tool changes take 7 to 10 s, so coarse infill only pays on bigger prints. |
| iQ TiQ2 | Enabled, not print-tested | Feature Split checked, with the fix its tool-change G-code needed. |
| IDEX copy and mirror modes | Refused | Both heads print at the same time, so there's nothing to hand off. |
| Printers that share one nozzle between extruders (Sovol SV02, Raise3D Pro3 shared mode, Adventurer 3) | Not offered | There's only one nozzle. |

On the non-Bambu printers, setup switches the prime tower to the same tower generator the Bambu printers use and turns off the purge and ramming settings it replaces. Step 4 lists every change. Their filament profiles often give both nozzles the same flow limit, so a bigger coarse flow limit makes a real difference to the saving.

"Enabled" means setup offers mixed-nozzle slicing on that printer and the engine has been checked on it without a printer attached: slicing both modes, the flush lines and the per-nozzle settings. It doesn't mean I've seen a print come off it. If you have one of these, a report of how it went, good or bad, is the most useful thing you can send me.

Single-nozzle printers and printers with one extruder aren't affected. Everything behaves like OrcaSlicer on them.

## Nozzle pairs

Any pair from 0.2, 0.4, 0.6 and 0.8 mm, with the smaller nozzle as the fine one. Either side of the printer can hold the fine nozzle.

Each nozzle's layer limits come from the Bambu printer profiles:

| Nozzle | Layer height range |
|---|---|
| 0.2 | 0.04 to 0.14 mm |
| 0.4 | 0.08 to 0.28 mm |
| 0.6 | 0.12 to 0.42 mm |
| 0.8 | 0.16 to 0.56 mm |

| Fine | Coarse | Test status |
|---|---|---|
| 0.2 | 0.4 | Print-tested on the H2D |
| 0.2 | 0.6 | Print-tested on the H2D |
| 0.2 | 0.8 | Print-tested on the H2D |
| 0.4 | 0.6 | Sliced and checked, not print-tested |
| 0.4 | 0.8 | Sliced and checked, not print-tested |
| 0.6 | 0.8 | Sliced and checked, not print-tested |

Two nozzles of the same size aren't a mixed-nozzle setup. That's a normal dual-nozzle print, and Cadence handles it the way OrcaSlicer does.

The fine height has to fit the fine nozzle's range, and the fine height times N has to fit the coarse nozzle's. So a 0.2 / 0.8 pair at 0.08 mm can use N = 2 to 7 (0.16 to 0.56 mm), and a 0.4 / 0.6 pair at 0.12 mm can use N = 1 to 3, which in practice means 2 or 3. Setup only lists combinations that fit.

## Flow types

Each nozzle can be Standard or High Flow, independently. A common setup is a Standard 0.2 on the fine side and a High Flow 0.6 or 0.8 on the coarse side for the bulk. Setup shows the flow type of each nozzle and keeps it. You change it in the Printer panel.

On an H2C whose rack mixes Standard and High Flow hotends, each material uses the settings for the flow type it actually prints with.

## Materials

Anything your printer prints on a single nozzle. The two nozzles can print the same material or two different ones. Each material keeps its own settings for the nozzle it's on, including its volumetric speed limit, and setup offers to update those for you.

Some things are still up to you:

- Abrasive filaments (CF, GF) need a hardened nozzle, the same as always.
- In Body Split, whether two different materials stick to each other is down to the materials. Interlocking beams help the joint mechanically, but they won't make PLA bond to PETG.
- A support interface in a different material works across the two nozzles, for example a PETG interface on the fine nozzle under a PLA part with a PLA support body on the coarse nozzle. That makes three filaments in the print, and the prime tower grows deeper to fit the extra changes. Whether the interface releases cleanly is down to the two materials, as in OrcaSlicer.

## Presets

Setup picks one of these for you based on the nozzle pair, the mode and the fine layer you choose. You can also pick them by hand in the process list, or tick "Keep my current process preset instead" in More options to stay on your own.

They're starting points. Setup still slices every legal N at your fine layer and ranks them, so you aren't limited to the N a preset was named for.

### Feature Split presets

| Pair | Preset | Fine | Coarse | N |
|---|---|---|---|---|
| 0.2 / 0.4 | MN Feature 0.2-0.4 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.2 / 0.6 | MN Feature 0.2-0.6 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.2 / 0.6 | MN Feature 0.2-0.6 0.08-0.40 | 0.08 | 0.40 | 5 |
| 0.2 / 0.6 | MN Feature 0.2-0.6 0.10-0.30 Standard | 0.10 | 0.30 | 3 |
| 0.2 / 0.6 | MN Feature 0.2-0.6 0.10-0.40 | 0.10 | 0.40 | 4 |
| 0.2 / 0.8 | MN Feature 0.2-0.8 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.2 / 0.8 | MN Feature 0.2-0.8 0.08-0.32 | 0.08 | 0.32 | 4 |
| 0.2 / 0.8 | MN Feature 0.2-0.8 0.08-0.40 | 0.08 | 0.40 | 5 |
| 0.2 / 0.8 | MN Feature 0.2-0.8 0.10-0.40 | 0.10 | 0.40 | 4 |
| 0.2 / 0.8 | MN Feature 0.2-0.8 0.12-0.24 | 0.12 | 0.24 | 2 |
| 0.4 / 0.6 | MN Feature 0.4-0.6 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.4 / 0.8 | MN Feature 0.4-0.8 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.4 / 0.8 | MN Feature 0.4-0.8 0.08-0.32 | 0.08 | 0.32 | 4 |
| 0.4 / 0.8 | MN Feature 0.4-0.8 0.08-0.40 | 0.08 | 0.40 | 5 |
| 0.4 / 0.8 | MN Feature 0.4-0.8 0.12-0.24 | 0.12 | 0.24 | 2 |
| 0.6 / 0.8 | MN Feature 0.6-0.8 0.18-0.54 | 0.18 | 0.54 | 3 |
| 0.6 / 0.8 | MN Feature 0.6-0.8 0.24-0.48 | 0.24 | 0.48 | 2 |


### Body Split presets

| Pair | Preset | Fine | Coarse | N |
|---|---|---|---|---|
| 0.2 / 0.4 | MN Body 0.2-0.4 Detail 0.08-0.16 | 0.08 | 0.16 | 2 |
| 0.2 / 0.4 | MN Body 0.2-0.4 Standard 0.10-0.20 | 0.10 | 0.20 | 2 |
| 0.2 / 0.4 | MN Body 0.2-0.4 Faster 0.12-0.24 | 0.12 | 0.24 | 2 |
| 0.2 / 0.6 | MN Body 0.2-0.6 Fine 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.2 / 0.6 | MN Body 0.2-0.6 Fine 0.10-0.30 | 0.10 | 0.30 | 3 |
| 0.2 / 0.6 | MN Body 0.2-0.6 Detail 0.12-0.24 | 0.12 | 0.24 | 2 |
| 0.2 / 0.6 | MN Body 0.2-0.6 Standard 0.12-0.36 | 0.12 | 0.36 | 3 |
| 0.2 / 0.6 | MN Body 0.2-0.6 Faster 0.14-0.42 | 0.14 | 0.42 | 3 |
| 0.2 / 0.8 | MN Body 0.2-0.8 Detail 0.08-0.24 | 0.08 | 0.24 | 3 |
| 0.2 / 0.8 | MN Body 0.2-0.8 Standard 0.08-0.32 | 0.08 | 0.32 | 4 |
| 0.2 / 0.8 | MN Body 0.2-0.8 Faster 0.10-0.40 | 0.10 | 0.40 | 4 |
| 0.4 / 0.6 | MN Body 0.4-0.6 Detail 0.12-0.24 | 0.12 | 0.24 | 2 |
| 0.4 / 0.6 | MN Body 0.4-0.6 Standard 0.16-0.32 | 0.16 | 0.32 | 2 |
| 0.4 / 0.6 | MN Body 0.4-0.6 Faster 0.20-0.40 | 0.20 | 0.40 | 2 |
| 0.4 / 0.8 | MN Body 0.4-0.8 Detail 0.16-0.32 | 0.16 | 0.32 | 2 |
| 0.4 / 0.8 | MN Body 0.4-0.8 Standard 0.20-0.40 | 0.20 | 0.40 | 2 |
| 0.4 / 0.8 | MN Body 0.4-0.8 Faster 0.24-0.48 | 0.24 | 0.48 | 2 |
| 0.6 / 0.8 | MN Body 0.6-0.8 Detail 0.18-0.36 | 0.18 | 0.36 | 2 |
| 0.6 / 0.8 | MN Body 0.6-0.8 Standard 0.24-0.48 | 0.24 | 0.48 | 2 |
| 0.6 / 0.8 | MN Body 0.6-0.8 Faster 0.28-0.56 | 0.28 | 0.56 | 2 |

All the MN presets are in the Bambu Lab vendor folder and match on the nozzle pair, not the printer model. Each one has Direct Drive and Bowden columns in Standard and High Flow. The Body presets ship with interlocking beams on, one beam row per band.

## Known issues

### Some settings are refused

The band grid only works if every layer in a band can be printed the way the fine layers would have been. Settings that break that are refused before slicing, with a plain reason that names the setting. You change the setting or turn mixed-nozzle slicing off for that plate. These are refused while mixed-nozzle slicing is on:

- variable or adaptive layer height, and any non-regular layering
- printing by object instead of by layer
- spiral vase
- Arachne walls (classic walls are needed)
- sparse infill patterns that don't fit the bands. Feature Split takes any pattern except Lightning and Locked Zag. Body Split only takes grid, triangles, stars and aligned rectilinear.
- sparse infill density of 0 or 100 percent, multiline infill, and a nonzero minimum sparse infill area
- Orca's own "combine infill" and top shell thickness overrides
- elephant foot compensation (set it to 0)
- wiping into the object or its infill
- hole-to-polyhole, separated infill, per-model centred infill, and counterbore hole bridging
- dynamic filament mapping and custom or cyclic tool ordering
- resonance avoidance and adaptive volumetric speed
- in Feature Split, a skirt from the draft shield option
- in Body Split, overlapping model parts
- different N on different objects on the same plate
- smooth timelapse, when one nozzle can't lay the shared first layer and the tower has to lag behind the part
- slice-data caching

Some of these may come back as the code matures, but I'd rather it refuses than prints something wrong. The messages are in plain words. Where the engine has its own code for a refusal, it's on a separate line under the message, for example "Details: SRL-A30". If you hit one, that line is useful in a bug report.

### Supports

Supports and support interfaces work in Feature Split and Body Split. The nozzles come from the filaments you pick for Support/raft base and Support/raft interface. The details and the reasons are in [How it works](HOW-IT-WORKS.md#supports). Refused on purpose:

- the support body (Support/raft base) at Default
- the interface on a bigger nozzle than the support body
- tree supports with the support body on the coarse nozzle (for now, I'm working on it)
- a raft with the support body on the coarse nozzle, and any raft in Body Split (for now)
- a separate prime tower filament

Rough edges I know about:

- With the support body on the coarse nozzle, the supports use more filament than OrcaSlicer's would. On models with lots of small contact areas, the support body on the fine nozzle can be just as fast.
- The prime tower can get small blobs in its purge area where coarse and fine layers overlap. The part isn't affected.
- The coarse support body can get a few thin layers (around 0.1 mm) just under the interface. They print fine, but they're thinner than that nozzle's usual minimum.
- With the support body on the coarse nozzle, the part can get a wider brim than OrcaSlicer would give it.
- Setup's last step doesn't yet say which nozzle prints the supports. Check the two support filament settings before slicing.
- OrcaSlicer itself prints one top interface layer when two are set. Cadence inherits that.

### Small prints and the prime tower

On small parts the prime tower can use more filament and time than the part. On one small test part with supports, the tower took about 9 m of filament against about 1.2 m for the part and its supports. Feature Split can be slower than one nozzle on small or short prints. The Detail and speed step shows the time for one nozzle only, so compare before you apply. If one nozzle wins, setup offers to print with one nozzle instead.

### Interlocking beams need joint height

A Body Split joint needs a little height for the beams to fit: about 1.2 to 1.4 mm at N = 3 where the joint ends at an open top surface. A shorter joint gets fewer beams or none, and setup doesn't warn you about that yet. A warning is planned, along with better beam placement for joints that start higher up a part.

Beams also don't know whether a joint is meant to move. For hinges or anything else that should come apart, set joining to "Off, for a part meant to move" in More options.

### Ranking takes time on large models

Every coarse layer on the Detail and speed step is timed by a full slice of the plate, including the tower and nozzle changes, plus one more slice for one nozzle only. On a big model, that can take several minutes. You can pick a row or move on before they're done. The times are also only for the process preset as it was when setup ran.

### Estimates, not measurements

All the times in setup and in the "Why fine or coarse?" report are the slicer's estimates. The ones I quote from my own prints say whether they're measured or estimated. One open question: on one large model, putting a High Flow hotend on the coarse side came out about 22 minutes slower in the estimate than Standard. I haven't worked out why yet.

### H2C hotend rack

The H2C has one hotend on the left and a six-hotend rack on the right. Cadence treats the rack side as one fixed diameter for the whole print, the same as the H2D's right nozzle. A change that keeps the mounted hotend gets the same ram and prime as on the H2D. A change that swaps the rack to another hotend goes through Bambu's own nozzle change routine. This has only been checked in the slicer, but I'd really love for someone to test this on their H2C and let me know how it works out.  

### Body Split with more than two materials

Body Split is built around one material per nozzle. You can pin a part to a specific material slot in More options, but more than two materials in one Body Split print hasn't been tested much.

### Printers I haven't printed on

H2D Pro, H2C and X2D are enabled but not print-tested. The per-nozzle flush lines and the X2D's Bowden columns were added for launch and checked in the slicer only. An automated check slices every H2D, H2D Pro, H2C and X2D machine preset and confirms each nozzle's flush line uses that nozzle's own diameter.

### Windows

The first Windows build isn't out yet. It'll be added to the release once it's built and tested on at least one machine.

# What's supported

This page covers what I've printed, what's enabled but untested, and what doesn't work yet.

## Printers

It should work on any printer with two or more nozzles, as long as it uses the same kind of prime tower as the Bambu printers. On a two-nozzle printer it uses both, and on a tool changer it can use several fine tools plus one coarse tool. It isn't tied to one printer model, but I've only been able to print on my own.

| Printer | Status | Notes |
|---|---|---|
| Bambu Lab H2D | Print-tested | Everything was built and tested on this printer. |
| Bambu Lab H2D Pro | Enabled, not print-tested | Same flush handling as the H2D. |
| Bambu Lab H2C | Enabled, not print-tested | Use one nozzle size on the left and one size in the rack. See the H2C note under known issues. |
| Bambu Lab X2D | Enabled, not print-tested | The second extruder is Bowden. The MN presets carry Bowden speed columns held to Bambu's own limits. |
| Snapmaker J1 (IDEX) | Enabled, not print-tested | Feature Split and Body Split checked without a printer. |
| Prusa XL (2 to 5 toolheads) | Enabled, not print-tested | Several fine toolheads plus one coarse toolhead (see Tool changers below). |
| Snapmaker U1 | Enabled, not print-tested | Several fine toolheads plus one coarse toolhead, with Cadence profiles for three fine nozzles plus one coarse: 0.2 + 0.4, 0.2 + 0.6, 0.2 + 0.8, 0.4 + 0.6 and 0.4 + 0.8 (see Tool changers below). |
| RatRig V-Core 4 IDEX, Volumic IDRE, Snapmaker A350 Dual and Artisan, Z-Bolt Dual, Lulzbot Taz Pro Dual | Enabled, not print-tested | Feature Split checked. Body Split runs on the same engine path but I haven't checked it on these. |
| Flashforge Creator 5, WonderMaker ZR Ultra, custom toolchanger profiles | Enabled, not print-tested | Two toolheads, Feature Split checked. Tool changes take 7 to 10 s, so coarse infill only pays on bigger prints. |
| iQ TiQ2 | Enabled, not print-tested | Feature Split checked, with the fix its tool-change G-code needed. |
| IDEX copy and mirror modes | Refused | Both heads print at the same time, so there's nothing to hand off. |
| Printers that share one nozzle between extruders (Sovol SV02, Raise3D Pro3 shared mode, Adventurer 3) | Not offered | There's only one nozzle. |

### Tool changers

On a tool changer like the Prusa XL or Snapmaker U1, a print can use more than two tools. Text and detail colors can each go on their own fine tool, and one coarse tool does the infill. Every tool needs to be one of the two sizes, and only one tool can have the coarse size. Anything else is refused with a message that names the toolhead.

A tool change only needs a short re-prime on the prime tower, the same as a nozzle change on the H2D, instead of a full purge. On each layer the fine tools print before the coarse one. That's what makes multicolor with mixed nozzles a lot more worth it here than on a two-nozzle printer.

Body Split still pairs one fine side with one coarse side, but the fine side can be several fine tools.

There are a few tower options under "Tool changer prime tower" in the process settings: ramming per material, a faster or sparser tower fill, and cooling idle tools to standby. They're all off by default, only show on printers with more than two tools, and haven't been print-tested yet.

If the plate has more colors than the printer has tools, setup tells you and asks before it goes on.

### Snapmaker U1

Cadence uses one coarse tool per print, so its U1 profiles have three fine nozzles and one coarse one: 0.2 + 0.4, 0.2 + 0.6, 0.2 + 0.8, 0.4 + 0.6 and 0.4 + 0.8, with the fine nozzles on tools 1 to 3 and the coarse one on tool 4. That gives you three colors for detail plus the infill. Pick the one that matches the hotends you have in. OrcaSlicer's own 0.4 + 0.6 profile (two of each) works too: run setup on it, which turns off Detect overhang walls for Feature Split. That profile counts tool changes as free, so its time estimates come out a bit low.

None of these have been print-tested. Snapmaker says mixing nozzle sizes in one print isn't supported on the U1 yet, and I don't have one to check whether the printer will take it. If you try it, start with something small and let me know how it goes.

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
- PLA as the main material on one nozzle and PETG on the other (for example PLA walls on the 0.2 and PETG infill on the 0.6) is refused. The prime tower keeps materials that don't stick to each other apart, and it can't do that on every level here. Use the same kind of material on both nozzles. A PETG support interface is different and works (see below).
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
- OrcaSlicer's mixed filaments (color mixing), belt printers, and IDEX/IQEX parallel printing
- resonance avoidance and adaptive volumetric speed
- in Feature Split, a skirt from the draft shield option
- in Body Split, model parts that overlap a lot. A small part sunk into a much bigger one, like text sunk into a base, is allowed: the small part keeps the shared space and you get a note after slicing.
- different N on different objects on the same plate
- on a tool changer, a third nozzle size or a second tool with the coarse size
- smooth timelapse, when one nozzle can't lay the shared first layer and the tower has to lag behind the part
- slice-data caching

Some of these may come back as the code matures, but I'd rather it refuses than prints something wrong. The messages are in plain words. Where the engine has its own code for a refusal, it's on a separate line under the message, for example "Details: SRL-A30". If you hit one, that line is useful in a bug report.

### Supports

Supports and support interfaces work in Feature Split and Body Split. The nozzles come from the filaments you pick for Support/raft base and Support/raft interface. The details and the reasons are in [How it works](HOW-IT-WORKS.md#supports). Refused on purpose:

- the support body (Support/raft base) at Default
- the interface on a bigger nozzle than the support body
- Slim, Strong and Hybrid tree supports with the support body on the coarse nozzle. Normal supports work there, and Organic trees do when the interface is on the fine nozzle.
- Organic trees with the support body on the coarse nozzle and the interface at Default or on the coarse nozzle
- any raft in Body Split (for now). In Feature Split a raft can have its body on either nozzle, except as below.
- support the two nozzles can't print with the filaments loaded, for example a PLA support body next to a PETG interface when the fine nozzle has no PLA. The message says what to load where.
- a separate prime tower filament

Rough edges I know about:

- With the support body on the coarse nozzle, the supports use more filament than OrcaSlicer's would. On small parts a coarse support body is often slower than printing everything on one nozzle, because of the extra nozzle changes. Setup's times show it.
- The prime tower can get small blobs in its purge area where coarse and fine layers overlap. The part isn't affected.
- Under round overhangs, like the bottom of a sphere, the coarse nozzle can lay some support and interface layers at about 0.1 mm, thinner than its usual minimum. Sliced and checked, not print-tested.
- With the support body on the coarse nozzle, the part can get a wider brim than OrcaSlicer would give it.
- With Organic trees on the coarse nozzle, the trunks are coarse and the thin tips and twigs print on the fine nozzle, so both nozzles print part of the tree.
- A raft with its body on the coarse nozzle starts at that nozzle's thinnest layer (0.12 mm on a 0.6), higher than the first layer you set, and you get a warning. Sliced and checked, not print-tested yet.
- A raft with its body on the coarse nozzle fails to slice on the MN Feature 0.2-0.6 0.08-0.24 preset, with an internal prime tower message. Use another preset for a raft for now.
- OrcaSlicer itself prints one top interface layer when two are set. Cadence inherits that.
- Setup suggests 3 interface layers for a different-material interface, whatever the layer height. At 0.08 mm fine layers that makes a very thin PETG layer, and the tree tips can show through on the underside. Use 4 interface layers when the fine layers are under 0.1 mm.

### Feature Split uses setup's split

Feature Split prints the walls, top and bottom surfaces and solid infill on the fine nozzle, and the sparse infill on the coarse nozzle at its own layer height. That's the split setup applies.

Two options under Layer height (Advanced) can move more to the coarse nozzle. Both are off by default and haven't been print-tested yet:

- "Solid infill at the coarse height" moves the solid infill deep inside a part to the coarse nozzle. Outer walls, top and bottom surfaces, bridges and the two fine layers under each top surface stay on the fine nozzle.
- "Inner walls at the coarse height" prints a band's inner walls with the coarse nozzle, behind the fine outer wall. The outer wall stays fine on every layer, and so do inner walls where the wall slopes or a top or bottom surface is close.

### Small prints and the prime tower

On small parts the prime tower can use more filament and time than the part. On one small test part with supports, the tower took about 9 m of filament against about 1.2 m for the part and its supports. Feature Split can be slower than one nozzle on small or short prints. The Detail and speed step shows the time for the same plate without the coarse nozzle, colors kept, so compare before you apply. If that wins, setup offers to print with one nozzle instead.

Two things make the tower bigger than you might expect:

- A support interface in a different material (PETG under PLA) gets its own part of the tower for the full height, even if it's only used for a few layers near the top.
- A first layer thinner than the coarse nozzle's minimum (0.12 on a 0.6) means the fine nozzle lays the tower's first layer and brim, which is slow. On my 0.2/0.6 test plate, a 0.12 first layer was about 8 minutes faster.

Feature Split can also be slower than one nozzle when the support body is on a different nozzle from the part, because most layers then change nozzle. Setup's times show it, so check before you apply.

### Interlocking beams need joint height

A Body Split joint needs a little height for the beams to fit: about 1.2 to 1.4 mm at N = 3 where the joint ends at an open top surface. A shorter joint gets fewer beams or none, and setup doesn't warn you about that yet. A warning is planned, along with better beam placement for joints that start higher up a part.

Beams also don't know whether a joint is meant to move. For hinges or anything else that should come apart, set joining to "Off, for a part meant to move" in More options.

### Ranking takes time on large models

Every coarse layer on the Detail and speed step is timed by a full slice of the plate, including the tower and nozzle changes, plus one more slice of the same plate without the coarse nozzle. On a big model, that can take several minutes. You can pick a row or move on before they're done. The times are also only for the process preset as it was when setup ran.

### High precision G-code

OrcaSlicer 2.5.0 added "High precision G-code (experimental)". It hasn't been tried with Feature Split or Body Split. The prime tower keeps the standard number of decimals either way.

### Estimates, not measurements

All the times in setup and in the "Why fine or coarse?" report are the slicer's estimates. The ones I quote from my own prints say whether they're measured or estimated. One open question: on one large model, putting a High Flow hotend on the coarse side came out about 22 minutes slower in the estimate than Standard. I haven't worked out why yet.

### H2C hotend rack

The H2C has one hotend on the left and a six-hotend rack on the right. Cadence treats the rack side as one fixed diameter for the whole print, the same as the H2D's right nozzle. A change that keeps the mounted hotend gets the same ram and prime as on the H2D. A change that swaps the rack to another hotend goes through Bambu's own nozzle change routine. This has only been checked in the slicer, but I'd really love for someone to test this on their H2C and let me know how it works out. All the hotends in the rack need to be the same size for now.  

### Text and small parts in Body Split

Text, inlays and other small parts on the fine nozzle work as their own part, painted, or as a modifier. A few limits:

- A small part sunk into a bigger one is kept only when the overlap is small: at most 10 percent of the bigger part, and the small part at most half its size. Anything more is refused as overlapping parts.
- Objects that also use modifiers, negative parts or paint use the older, stricter overlap check.
- The coarse nozzle can't reach into sharp inside corners of small letters, so the top around text can keep a few tiny gaps. v1.1 lays that top with one wall and a proper top fill, which closes most of them. A few can stay where letters sit very close together.

### Body Split with more than two materials

Body Split is built around one material per nozzle. You can pin a part to a specific material slot in More options, but more than two materials in one Body Split print hasn't been tested much.

### Printers I haven't printed on

H2D Pro, H2C and X2D are enabled but not print-tested. The per-nozzle flush lines and the X2D's Bowden columns were added for launch and checked in the slicer only. An automated check slices every H2D, H2D Pro, H2C and X2D machine preset and confirms each nozzle's flush line uses that nozzle's own diameter.

### Windows

The Windows build is made by GitHub Actions from the same source as the Mac build. It's tested on one machine, a Surface Book. The installer isn't signed, so SmartScreen warns the first time.

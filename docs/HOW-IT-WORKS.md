# How it works

This page is for people who already know how slicing works and want to know what Cadence does differently. If you just want to print something, the [README](../README.md#first-print) has the short version.

In short, a multi-nozzle printer with two different nozzle sizes gets one shared Z grid, each region of the model is sliced only at its own nozzle's planes, and the slicer checks that every coarse band actually saves time before it keeps it.

## Why bother with two different nozzles

A single global layer height is a compromise when the nozzles differ. Fine enough for a 0.2 means the 0.8 is wasted, and thick enough for the 0.8 means the 0.2 can't do its job. OrcaSlicer and PrusaSlicer already let you mix nozzle sizes on multi-nozzle printers, and PrusaSlicer's "combine infill every n layers" and Cura's infill layer thickness let infill print thicker than the walls. I wanted each nozzle to really have its own layer height, with its own limits, for any feature or any part, not only infill.

Reasons you might want that:

- A 0.2 for text, edges, small features and top surfaces, and a 0.6 or 0.8 for the bulk, so you're not paying fine-nozzle time for infill nobody sees.
- Bigger nozzles lay wider, thicker lines that bond better layer to layer. Body Split lets a structural part print coarse and strong where needed while letting regions that might need more fidelity, like hinges or text, print with the smaller nozzle.
- Small threads, hinges, snap fits, clips or embossed text on an otherwise large, simple part.
- Abrasive or filled filaments (CF, GF) do better through a bigger nozzle, and the other nozzle can stay small for detail in a different material. With Body Split, each part can be its own material as well as its own layer height.
- Fast, coarse support bodies with a fine interface where they touch the part, including a different interface material.
- A High Flow hotend on the coarse side for the bulk, a standard one on the fine side.

## The layer grid

You pick a fine layer height *h* for the small nozzle and a whole number *N*. The big nozzle prints at *N × h*. At 0.08 mm and N = 7, that's 0.56 mm.

The slicer builds one shared Z grid where every Nth fine plane is also a coarse plane. Each region of the model belongs to one nozzle and gets sliced only at that nozzle's planes, so the two sets of layers always meet at the same height every N fine layers. Between those meeting planes, the fine nozzle prints N layers and the coarse nozzle prints one. I call the stretch between two meeting planes a band.

The upper surface of each band is flat before the next band starts, so nothing prints into a gap.

For now, N is a single number per plate. If you have several objects on one plate, they all use the same N so their band boundaries line up. This may change in the future.

Variable and adaptive layer height don't work with this, since the whole point is a regular grid. The slicer refuses them with a clear reason for why.

## Each nozzle keeps its own limits

Each nozzle keeps its own minimum and maximum layer height, line widths and flow limits. Nothing about the fine nozzle's settings gets forced onto the coarse one or the other way round.

The coarse height has to fit the coarse nozzle, so N is capped. A 0.8 tops out at 0.56 mm, which allows N = 7 at 0.08 but only N = 4 at 0.12. The fine height has to fit the fine nozzle in the same way.

Each material also keeps the limits it has for the nozzle it's on. When you set up mixed-nozzle slicing, setup offers to update each material's settings for its nozzle, so a material on the 0.8 doesn't carry the 0.2's volumetric speed cap. If it didn't, the coarse nozzle would crawl and most of the time saving would disappear.

The setup refuses combinations a nozzle can't print and says which setting is the problem.

## Nozzle changes and the prime tower

Switching happens at band boundaries, not every layer, so a band costs one change pair at most. Within a band the coarse nozzle prints its one layer, and the fine nozzle prints its N.

The prime tower only needs to reach the last change, so it stops growing once the switching stops. It sizes its own footprint for stability, and builds its levels at the coarse height where it can. The flush and ramming use each nozzle's own diameter and limits, so the 0.2 isn't asked to push a 0.8's flow and the 0.8 isn't starved by a 0.2's.

On Bambu printers this uses Bambu's own prime tower, with per-nozzle flush lines so the printer's change routine knows which physical nozzle it's dealing with. On other multi-tool printers, setup switches the tower to the same generator and the printer's own tool change G-code does the change. Alignment between the two nozzles comes from the printer's own nozzle offset calibration, the same as any two-nozzle print. The slicer works in one coordinate frame and doesn't add an offset of its own.

## It checks that switching actually saves time

Using the coarse nozzle isn't free. Every coarse band needs a nozzle change pair and tower levels, and on small or sparse layers that can cost more than the coarse nozzle saves.

For every band, the slicer estimates the time the coarse nozzle saves against the nozzle change and the tower levels that band forces. Bands that don't pay stay on the fine nozzle.

It then checks the whole plan against printing everything fine. If mixed isn't faster, it prints everything fine and tells you why. A band high up the model that can't pay for the extra tower height it would need is dropped too. That's the usual reason the coarse nozzle stops partway up a tall, thin model: above some height, each band saves less than the tower costs to keep climbing.

You can see every one of these decisions after slicing. "Why fine or coarse?" in the sidebar, or "Why fine or coarse here?" in the preview legend, opens a report with:

- a summary line, such as where the coarse nozzle was used and why it stopped
- the layer ranges, grouped where consecutive bands got the same answer
- for each band, the fine time, the coarse time, the switch and tower cost, the net, and the reason: Saves time, Costs more, Not eligible (too small to pay for a switch), or Fallback (the comparison couldn't be priced, so the band kept what the geometry stage decided)

These are estimates from the slicer's own timing, not measurements. The slice total also includes motion and cooling that the per-band numbers leave out.

## Two ways to split a print

### Feature Split

One model, split by feature. By default the fine nozzle prints walls, top and bottom surfaces and solid infill, and the coarse nozzle prints sparse infill. Any feature can go on either nozzle, supports included (see [Supports](#supports)).

In practice you get parts that look like they were printed with the fine nozzle at fine layer heights, with substantial time savings compared with single-nozzle prints.

A few settings have to fit the band grid. In Feature Split, sparse infill can be any pattern except lightning or locked zag (gyroid is my preferred infill for everything, and all test prints have used it), and some Orca options that reshape infill or walls per layer don't work here. The full list is under known issues in [Supported](SUPPORTED.md#known-issues). Each one is refused with a plain reason that names the setting.

### Body Split

Split by part. Each part, or painted region, is assigned to a nozzle and printed entirely at that nozzle's layer height, walls and infill both. A single-part object can't be split, so either paint the regions you want fine with the Color Painting tool, or use Split to parts. My preferred method is to design features as separate bodies in Fusion 360, and then export the entire part as a .3mf, and pick "one object with multiple parts" in the pop-up when loading it in the slicer. In body split, sparse infill has to be a pattern that prints the same on every layer: grid, triangles, stars, or aligned rectilinear.

This is the mode for a coarse structural body with a fine detailed part on it, or for two materials where each wants its own nozzle. The part that sits on the bed can be on either nozzle. When it's the coarse one, it prints at its own coarse layer height from the bed up.

Where parts on different nozzles touch, the slicer adds interlocking beams: a lattice aligned to the band grid, so each body reaches into the other and the joint doesn't depend on a flat butt bond. Each beam row is one band tall.

A joint needs a little height for the beams to fit: about 1.2 to 1.4 mm at N = 3 where the joint ends at an open top surface. A shorter joint gets fewer or no beams, and setup doesn't warn about it yet. If you change a layer height in Plate Settings after setup, the beam depth is worked out again for the new bands on the next slice.

At each nozzle change, the arriving nozzle travels straight to its part's wall, still retracted, and only unretracts once it's there. That keeps the change from leaving a blob on the seam.

Painting on a fine part with the fine nozzle's own filament keeps that paint on the fine nozzle. It doesn't add nozzle changes anywhere else on the plate.

## Supports

Supports work in both Feature Split and Body Split. Which nozzle prints them comes from the Support settings, the same two settings as in OrcaSlicer: the filament for the support body (Support/raft base) and the filament for the interface (Support/raft interface). Whichever nozzle a filament is on prints that part of the support. Setup doesn't change any of your support settings, though setting interface filament to something like PETG for PLA prints (one of the best benefits of dual nozzle printing) automatically pulls up the same OrcaSlicer pop-up with suggested interface settings changes. When doing this, the prime tower automatically adjusts to allow for the 3 different materials used in the print. 

If the interface is a different material, like PETG under PLA, set top interface layers to 3. OrcaSlicer prints one of those layers in the support's own material, so 3 gives you two PETG layers, which peel off in one piece. With one layer the film is thin and fiddly to remove.

A few details:

- No interface material goes on the bed or on the tower's first layer. The first layer is the part's own material.
- Small bits of support body the coarse nozzle can't lay, like the support's first layer and the solid layer right under the interface, go to the fine nozzle. They print in the body's material if that material is loaded on the fine nozzle, not in the interface material.
- An interface left at Default follows the body's filament.
- Tree supports work with the support body on the fine nozzle. On the coarse nozzle they're refused for now, because a leaning branch doesn't stack cleanly in thick layers yet. I'm working on it.
- Rafts work in Feature Split with the support body on the fine nozzle. They're refused with the body on the coarse nozzle, and in Body Split, for now.

Some combinations are refused on purpose, with a message that names the setting:

- The support body at Default. At Default the support prints with whichever filament happens to be active, which could be on either nozzle from one layer to the next. Pick a specific filament.
- An interface on a bigger nozzle than the body. The interface is the part that touches the model, so putting it on the coarser nozzle defeats the point. Set it to Default, or to a filament on the body's nozzle or the finer one.
- Tree supports with the body on the coarse nozzle. Put the support body on the fine nozzle's filament, or use Normal supports.
- A raft with the support body on the coarse nozzle, or any raft in Body Split. Put the support body on the fine nozzle's filament, or turn the raft off.
- A separate prime tower filament. It would add a third filament to every layer of the tower.

You can also put the whole support on the fine nozzle. On some models the print then ends up on the fine nozzle only, the same as one nozzle.

## The first layer

Both nozzles share the first layer. If one nozzle can't lay the base layer or the first layer height, its part starts on the bed with a thicker first cell, and the prime tower lags behind the part to give each nozzle a layer it can print. That's what happens in Body Split when the part on the bed is on the coarse nozzle. Smooth timelapse and wrapping detection need the tower level with the part, so in that case they're refused.

After the start purge line, the nozzle retracts and lifts before its first travel, so it doesn't drag across the purge line on its way to the tower.

## Setup and the ranking by full slices

The setup wizard asks what prints fine, which material goes on each nozzle, and then the fine and coarse layers. The fine layer list comes from the fine nozzle's own presets. Then it slices every legal N in the background and shows you the estimated time and nozzle-change count for each, against the same print all fine.

Each option is a full slice, tower and nozzle changes included, so the numbers include everything the per-band check does. On a big model the times fill in over a few minutes. You can move on at any point, and a row without a time says why.

The fastest row is tagged Fastest. When several rows land within about 2 percent of each other they're all tagged "About as fast", and the default pick is the one with the fewest nozzle changes, since that means less tower and priming. If every row is slower than one nozzle only, the page says "On this model, switching nozzles does not save time" and offers "Print with one nozzle instead".

The times are for the process preset as it stands when you run setup. If you change speeds or infill afterwards, run setup again or just slice and check.

## When it's off

With mixed-nozzle slicing off, which is the default for every project, Cadence slices like OrcaSlicer at the base commit named in NOTICE.md. The mixed-nozzle code only runs when a plate is set to Feature Split or Body Split.

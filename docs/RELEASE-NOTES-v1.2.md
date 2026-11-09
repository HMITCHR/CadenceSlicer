# Cadence Slicer v1.2

A big one this time: Cadence is now built on OrcaSlicer 2.5.0, and tool changers like the Snapmaker U1 can use more than two tools in one print. Paint works the same as in v1.1.2.

## New

- **Built on OrcaSlicer 2.5.0.** You get nearly everything new in [OrcaSlicer 2.5.0](https://github.com/OrcaSlicer/OrcaSlicer/releases/tag/v2.5.0). The new Design tab is the one thing I left out for now. Update checks are still off, and macOS 12 or later is needed now (it was 11.3).
- **Tool changers can use more than two tools.** On a tool changer like the Snapmaker U1 or Prusa XL, text and detail colors can each go on their own small-nozzle tool, with one big-nozzle tool doing the infill. A tool change only needs a short re-prime on the tower instead of a purge, so multicolor with mixed nozzles makes a lot more sense here than on a two-nozzle printer. Every tool needs to be one of the two sizes, and only one tool can have the big nozzle. Not print-tested yet, so if you have one I'd love a report.
- **Snapmaker U1 profiles for more nozzle pairs.** Cadence adds U1 profiles with three small nozzles and one big one, since Cadence uses one big-nozzle tool per print: 0.2 + 0.4, 0.2 + 0.6, 0.2 + 0.8, 0.4 + 0.6 and 0.4 + 0.8, with the small nozzles on tools 1 to 3 and the big one on tool 4. So you get three colors for detail plus the infill. OrcaSlicer's own 0.4 + 0.6 U1 profile works too, and setup now turns off Detect overhang walls for Feature Split on it. Snapmaker says mixing nozzle sizes in one print isn't supported on the U1 yet, and I haven't been able to check whether the printer accepts it, so start with something small.
- **Tool changer tower options, off by default.** Ramming per material, a faster or sparser tower fill, and cooling idle tools to standby and heating them back up in time. They only show on printers with more than two tools, and they're there for test prints.
- **Two new Feature Split options, off by default.** "Solid infill at the coarse height" moves the solid infill deep inside a part to the big nozzle, and "Inner walls at the coarse height" moves the inner walls behind the outer wall to the big nozzle. Outer walls, top and bottom surfaces and bridges stay on the small nozzle either way. Not print-tested yet.
- **A fairer comparison in setup.** Setup now compares against the same plate with your colors and paint kept, all on the small nozzle. In v1.1.2 that time left the paint out, so on a painted plate one nozzle could look a lot faster than it really was.
- **Setup uses the material that's on the big nozzle**, and asks you to pick one if there isn't one yet, instead of grabbing the next slot.
- On a tool changer, setup tells you when the plate has more colors than the printer has tools, and asks before it goes on.

## Things that changed with OrcaSlicer 2.5.0

- **New projects start with one filament.** OrcaSlicer 2.5.0 starts a new H2D project with one filament slot instead of two. Cadence's setup adds a second slot (a copy of the first) when you open it, so just pick it for the big nozzle on the Materials step.
- **Older projects keep their paint depth.** 2.5.0 adds settings for how deep paint goes into a part. Projects from v1.1.x open with the same paint depth as before in Feature Split and Body Split. New projects take it from the preset, which matches the shell layers on the MN presets. If you raise the top or bottom shell layers, raise "Top paint penetration layers" and "Bottom paint penetration layers" too, or set them to 0 so they follow the shells.
- **Features left on Default now use the part's own filament.** Projects made with setup aren't affected. A Feature Split project that only set the sparse infill filament used to be refused, and now it slices with the solid infill on the small nozzle.
- **Missing filament colors get filled in with white.** If an older project saved fewer colors than it has slots, the empty ones turn white. That can bump up the purge between slots and keep more of the print on the small nozzle. Setting the slot back to its real color fixes it.
- **With Cadence off, a saved prime tower now prints.** A project with the prime tower on and "Independent support layer height" on used to slice without the tower. OrcaSlicer 2.5.0 builds it, so those prints take longer now.
- **Klipper time estimates** are about 2 to 3% higher, with pretty much the same G-code.

## Projects from v1.1.x

OrcaSlicer 2.5.0 refreshes most settings from the presets when it opens a project. Cadence projects keep the settings Cadence needs (the nozzle pair, each nozzle's material settings and retraction, the feature filaments, the prime tower settings and the paint depth), in both the app and the command line.

## Fixes

- **PETG support interface next to PLA.** The PETG block on the prime tower could get PLA laid in the middle of it, and the tower could split there. Each block now stays on its own material, with no extra nozzle changes.
- **Fine skins in Body Split.** A fine body starting a gap above the bottom skins could stop the slice with an internal prime tower error. It now tells you before slicing and suggests another big-nozzle layer height or turning Fine skins off.
- **Printer sync.** Synchronize AMS Filament Information now matches each material against the AMS that feeds its nozzle, only lists the materials you're actually using, and keeps their slot numbers. The send dialog no longer shows the "Regroup and slice" hint in Feature Split and Body Split. With Cadence off it all works like OrcaSlicer.
- **Setup opens on the right mode.** A painted or multi-part plate in a Feature Split project used to open setup on Body Split.
- **Turn off puts the prime tower back.** Turning Cadence off now sets the tower settings setup changed back to what the preset had, so an Off plate prints the tower its preset asks for. Anything you changed yourself is left alone.
- **Tower line width.** When the time check keeps a whole Feature Split plate on the small nozzle, the tower could be drawn with the big nozzle's line width on some plates with three or more filaments. It uses the small nozzle's now.
- "Why fine or coarse?" no longer shows a negative time when the tower purge saves time.
- The new OrcaSlicer 2.5.0 features that don't work with Feature Split or Body Split yet (mixed filaments, belt printers and IDEX/IQEX parallel printing) are refused with a message, and Feature Split also refuses Cyclic tool ordering. They all work as normal with Cadence off.

## Not in this release

- **Paint coloring the part's own walls.** Still being worked on, and it'll come in a later update. Paint works the same as in v1.1.2 for now.

## Known issues

- Feature Split uses setup's default split: walls, top and bottom surfaces and solid infill on the small nozzle, sparse infill on the big one. The two new options above can move more to the big nozzle, but they're off by default and not print-tested yet.
- In Feature Split a painted side gets its own set of walls in the paint color, and then a second full set of the part's own walls behind it. On my 4-wall test cube the painted side is 8 walls thick: 4 in the paint color, then 4 in the base color. The fix is the paint change above that's held for a later update.
- Multicolor is slow with Feature Split on printers limited to two nozzles. Purging on a fine nozzle (0.2 in this case) is very slow, so multicolor with Feature Split is often slower than just splitting the colors across two same sized nozzles. On a tool changer each color can sit on its own tool, so a color change is a quick re-prime instead.
- Organic trees with the support body on the big nozzle need the interface on the small nozzle. Slim, Strong and Hybrid trees need the support body on the small nozzle.
- A raft with its body on the big nozzle fails to slice on the MN Feature 0.2-0.6 0.08-0.24 preset. Body Split can't print on a raft yet.
- Under round overhangs the big nozzle can lay some support layers at about 0.1 mm, thinner than its usual minimum.
- On small parts, support on the big nozzle can be slower than one nozzle. Setup's times show it.
- PLA on one nozzle and PETG on the other is refused, since the prime tower can't join them. A PETG support interface is fine.
- OrcaSlicer 2.5.0's "High precision G-code (experimental)" hasn't been tried with Feature Split or Body Split yet.
- Every so often a slice can get stuck right at the start. It's in OrcaSlicer's own code, not Cadence's. If the progress bar doesn't move, close the app and slice again.
- In Feature Split and Body Split, picking Auto in the Filament grouping dialog stops the slice. Leave it on Custom. In Feature Split the message talks about the brim and skirt having no nozzle, but Auto is the actual cause.
- With Cadence off, a plate with two different nozzle sizes can still come out slightly different from OrcaSlicer 2.5.0 (one fan speed on the prime tower, for example). Same-size nozzles aren't affected.
- I've only printed on my H2D. The other dual-nozzle Bambu printers and the tool changers are enabled but untested. On the H2C, use one nozzle size on the left and one size in the rack.

Downloads: Mac (Apple silicon) DMG and Windows installer below. Same install steps as before, in the README, except macOS 12 or later is needed now.

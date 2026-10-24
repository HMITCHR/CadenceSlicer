# Cadence Slicer v1.1.2

A small update with paint fixes for Feature Split and one prime tower fix.

## Fixes

- **Paint in Feature Split keeps the big nozzle's infill.** A painted color used to take over the sparse infill behind the paint as well as the walls. In v1.1.1 that could stop a painted part from slicing at all, with a message that the outer walls and the sparse infill were on the same nozzle. Now the paint color only goes on what the small nozzle prints (walls, top and bottom surfaces and solid infill). Sparse infill and anything on the big nozzle keep the part's own filament.
- **Paint only goes as deep as the walls in Feature Split.** Paint used to spread right through the inside of the part and split the infill into one region per color, and the narrow ones went to the small nozzle. Now it stops at the depth of the walls, or at "Maximum width of a segmented region" if you've set that smaller. A layer that's painted all the way round keeps the paint all the way in, so the part's own color doesn't add a filament change there. This is only a partial fix: the painted side still gets extra walls (see Known issues).
- **The time check on painted parts.** Paint with two colors on the small nozzle changes filament every layer, and that time is the same whether the big nozzle prints the infill or not. The time check counted it anyway, so on some painted parts it decided the big nozzle wasn't worth it and printed everything on the small one. It now leaves out the time both choices pay.
- **No drop at the change point on the way to the prime tower.** After some filament changes in Feature Split (mostly with the support body on the big nozzle, and on painted parts), the nozzle dropped to the tower's height right where the change finished, pushed out a bit of filament there, then travelled to the tower at that height. With a part under that spot, the nozzle could hit its top and drag through it. Now it moves over to the tower first and lowers there.

## Known issues

- Feature Split uses setup's default settings for now. More is planned for v1.2.
- In Feature Split a painted side gets its own set of walls in the paint color, and then a second full set of the part's own walls behind it. On my 4-wall test cube the painted side is 8 walls thick: 4 in the paint color, then 4 in the base color. This comes from how OrcaSlicer splits painted areas into their own regions, and it isn't new in v1.1.2. The goal is for paint to color the part's own walls instead, so a painted side with 4 walls set has 4 walls in the paint color. That's being worked on for a later release.
- Multicolor is slow with Feature Split on printers limited to two nozzles. Purging on a fine nozzle (0.2 in this case) is very slow, so multicolor with Feature Split is often slower than just splitting the colors across two same sized nozzles. On tool changers, or printers like the H2C or the Snapmaker U1, each color can sit in its own hotend, so there's no purge on a color change. That should make mixed nozzles with multicolor make a lot more sense there, and let the coarse nozzle printing infill actually save time.
- On a painted plate, setup's "One nozzle only" time leaves the paint out, so one nozzle can look a lot faster than it really is.
- Organic trees with the support body on the big nozzle need the interface on the small nozzle. Slim, Strong and Hybrid trees need the support body on the small nozzle.
- A raft with its body on the big nozzle fails to slice on the MN Feature 0.2-0.6 0.08-0.24 preset. Body Split can't print on a raft yet.
- Under round overhangs the big nozzle can lay some support layers at about 0.1 mm, thinner than its usual minimum.
- On small parts, support on the big nozzle can be slower than one nozzle. Setup's times show it.
- PLA on one nozzle and PETG on the other is refused, since the prime tower can't join them. A PETG support interface is fine.
- I've only printed on my H2D. The other dual-nozzle Bambu printers are enabled but untested.

Downloads: Mac (Apple silicon) DMG and Windows installer below. Same install steps as before, in the README.

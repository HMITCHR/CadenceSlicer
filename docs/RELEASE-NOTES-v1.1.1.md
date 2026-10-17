# Cadence Slicer v1.1.1

A small update with a prime tower fix and a few smaller ones.

## Fixes

- **The nozzle no longer drops into the part to start the prime tower.** In Feature Split, when the time check leaves the big nozzle with nothing to print but the plate still changes filament (a PETG support interface, say), the prime tower could start late. The nozzle then went straight down beside the finished part, and on the next layer straight down through it. This has been there since v1.0.2. The tower now starts on the first layer, the way OrcaSlicer builds it, so those prints take longer than before (about 47% longer in the estimate for a test part) and the nozzle stays above the part (checked in the slicer, not yet on a printer). If you have a Feature Split file with a support interface in another material, slice it again.
- **Fewer lumps on the big nozzle's top around fine parts (Body Split).** Where the big nozzle fills the top around small text or other fine parts, the gap fill and the small pockets of top fill now print slower and with a little less flow. Fine text comes out much better raised on top of a coarse surface. When the text is flush with the top, the coarse nozzle has to fill the gaps around the letters, and there's a limit to how small a gap it can fill, so you can get holes or a poor surface finish. Raised text also keeps the big nozzle's top right up to its letters.
- **Timelapse in Body Split.** The send dialog no longer warns about the timelapse, and every photo is taken the same way, so a long print doesn't jump between two camera styles.
- **No empty first layer on 0.2/0.8 Body Split.** When the first layer was thinner than the big nozzle can lay (0.2/0.8, and 0.2/0.6 with the Fine preset), the G-code started with an empty layer and the part and tower began on layer 2. They now start on layer 1.
- **Clearer messages.** When the big nozzle has nothing to print, setup now says why (the inside is all solid infill at 100% density, or there's no sparse infill) and what to change. Setup no longer shows internal check names. A Feature Split print with the outer walls and sparse infill on the same nozzle says which nozzle and what to set back. Support interface advice now fits the support type, and the Body Split message for mixed filaments in one part points to Fine skins.
- **Two nozzles of the same size.** Feature Split now says that it needs two nozzles of different sizes.
- **Fixes from tester reports.** Setup works on a project with a locked plate, and running setup over the whole project gives every plate its Feature Split materials. Step 3's times now match what Slice gives, and setup moves a prime tower that was sitting inside the parts clear of them. Feature Split keeps a 100% density part's solid infill at the fine layer height, and the time check prices that plate right.
- **Turn off** now puts the hidden "No sparse layers" setting back to off, so the stock prime tower starts on the first layer.
- **Smooth timelapse** in Feature Split is refused before slicing when the big nozzle can't lay the tower's layers, with a message saying to switch to Traditional.
- Step 4 now says when the preset switch lowers sparse infill density, wall loops or shell layers, and how to keep yours.
- **Docs.** The README, How it works and Supported now describe the Feature Split that setup applies (walls, top and bottom surfaces and solid infill on the small nozzle, sparse infill on the big one) and what works with supports and rafts.

## Not in this release

- Filling the small gaps around sunk letters. It filled them on a test print but left strings and blobs, so it's held back.

## Known issues

- Feature Split uses the split setup applies for now. More is planned for v1.2.
- Organic trees with the support body on the big nozzle need the interface on the small nozzle. Slim, Strong and Hybrid trees need the support body on the small nozzle.
- A raft with its body on the big nozzle fails to slice on the MN Feature 0.2-0.6 0.08-0.24 preset. Body Split can't print on a raft yet.
- Under round overhangs the big nozzle can lay some support layers at about 0.1 mm, thinner than its usual minimum.
- On small parts, support on the big nozzle can be slower than one nozzle. Setup's times show it.
- PLA on one nozzle and PETG on the other is refused, since the prime tower can't join them. A PETG support interface is fine.
- I've only printed on my H2D. The other dual-nozzle Bambu printers are enabled but untested.

Downloads: Mac (Apple silicon) DMG and Windows installer below. Same install steps as before, in the README.

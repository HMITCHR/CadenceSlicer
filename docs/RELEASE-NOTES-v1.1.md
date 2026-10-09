# Cadence Slicer v1.1

Two big additions this time: tree supports can print on the big nozzle, and you can put small text on a part that's otherwise printed with the big nozzle. Plus a lot of prime tower and support fixes that came out of test prints and a few hundred test slices.

## New

- **Organic tree supports on the big nozzle.** The trunks and thick branches print on the big nozzle at its layer height, and the thin tips go to the small one. Works in Feature Split and Body Split. Slim, Strong and Hybrid trees still need the small nozzle for now.
- **Fine text on a coarse part (Body Split).** Add text as its own part, paint it on, or use a text modifier, and the small nozzle prints it at its own layer height while the big nozzle prints the rest. Text sunk into the part is kept now instead of refused.
- **Cleaner tops around text.** The big nozzle now lays the top with one wall and a normal top fill instead of rings of walls around every letter, which closes most of the small gaps.
- **Rafts with the support body on the big nozzle (Feature Split).** The raft starts at the big nozzle's thinnest layer, so the first layer ends up a bit higher than you set, and it tells you.
- **Better time check.** The check that decides whether the big nozzle is worth it now times the prime tower and nozzle changes the same way the G-code does. Small prints that would come out slower with both nozzles now just print on the small one.

## Fixes

- Mixed-nozzle setup now works on Windows set to a region that uses a comma for decimals (most of Europe). "Keep both nozzles" could flip both nozzles back to the same size, and step 3 could show no coarse layers. Thanks to the Reddit user who reported it!
- The nozzle no longer bumps into the prime tower. Moves around tower rows that print early, and tower lines running under ramming lines, used to touch the tower walls.
- Body Split builds the prime tower when the first layer is only support.
- A PETG interface no longer ends up inside the PLA support body.
- Organic branches no longer print in midair, supports no longer grow between letters, and sunk text sits right on the part again.
- After syncing your printer, each nozzle shows the right flow type, and a material that gets moved to the other nozzle uses that nozzle's settings. This one was in v1.0.2.
- The start G-code primes the nozzle the print actually starts on.
- Setup says which nozzle prints the supports and interface, names the slot you picked, and won't leave you on a layer option that can't slice.
- Plainer error messages, and the G-code header shows the Cadence version.

## Known issues

- Body Split can't print on a raft yet, and Slim, Strong and Hybrid trees need the support body on the small nozzle.
- PLA on one nozzle and PETG on the other is refused, since the prime tower can't join them. A PETG support interface is fine.
- A few tiny gaps can still show up between letters that sit really close together.
- With fine layers under 0.1 mm, use 4 interface layers instead of the suggested 3, or the tree tips can show through a thin PETG interface.
- In Prepare, text sunk flush into a part looks glitchy. That's just the 3D view, the slice is fine.
- I've only printed on my H2D. The other dual-nozzle Bambu printers are enabled but untested.

Downloads: Mac (Apple silicon) DMG and Windows installer below. Same install steps as before, in the README.

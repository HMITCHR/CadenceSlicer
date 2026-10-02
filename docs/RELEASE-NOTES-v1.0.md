# Cadence Slicer v1.0

This is the first release. It's built from commit 1f9b7ad1bb, on top of OrcaSlicer at commit c806a09c7c.

## What it does

Cadence Slicer is a fork of OrcaSlicer that lets a multi-nozzle printer use two different nozzle sizes in one print, each at its own layer height. You pick a fine layer height and a whole number N, and the coarse nozzle prints at N times the fine height. The two sets of layers meet every N fine layers.

- Feature Split: by default the fine nozzle prints walls, top and bottom surfaces and solid infill, and the coarse nozzle prints sparse infill. Any feature can go on either nozzle, supports included.
- Body Split: each part or painted region prints entirely on one nozzle, walls and infill both. The part on the bed can be on either nozzle. Touching parts on different nozzles get interlocking beams.
- Supports work in both modes. The support body and interface print on the nozzles of the filaments you pick for them. The usual setup is the body on the coarse nozzle and the interface on the fine one, and the interface can be a different material, such as PETG under a PLA part.
- Setup is four steps. It times every legal N with a full slice and compares it with one nozzle only, and if switching nozzles doesn't save time it tells you.

With mixed-nozzle slicing off, which is the default for every project, it slices like OrcaSlicer.

The same work is going to OrcaSlicer as pull requests shortly after this release. I'll add the PR numbers here once they're open. Cadence is how you can use it until then.

## New since the last test builds

- Supports and support interfaces print in Feature Split and Body Split, including a separate interface material across the two nozzles.
- With a third filament for the interface, the prime tower grows deeper to fit instead of refusing the slice.
- No interface material on the bed or on the tower's first layer.
- Body Split slices when the part on the bed is on the coarse nozzle. That part prints at its coarse layer height from the bed.
- After the start purge line, the nozzle retracts and lifts before its first travel, so it no longer drags through the purge line.
- Supports on the coarse nozzle print at that nozzle's own line width, and the support's first layer goes down with the part's first layer.
- A different interface material, like PETG under PLA, gets a solid layer of the support's own material under it, the same as in OrcaSlicer.
- The prime tower always prints its first layer on the bed, along with the part's first layer.
- Thin sections keep their infill.
- Setup keeps your bed type and all your support settings.
- Bambu printers start on the bed type their printer profile names, Textured PEI on the H2D.

## Printers

| Printer | Status |
|---|---|
| Bambu Lab H2D | Print-tested |
| Bambu Lab H2D Pro, H2C, X2D | Enabled, not print-tested |
| Prusa XL, Snapmaker J1 and U1, and other IDEX and toolchanger printers in OrcaSlicer's profiles | Enabled, not print-tested |

Any pair of 0.2, 0.4, 0.6 and 0.8 mm nozzles, Standard or High Flow. The full list is in [Supported](SUPPORTED.md).

## Install

On macOS, open the DMG and drag CadenceSlicer into Applications. The build isn't signed, so macOS blocks the DMG the first time you open it, and then the app. Each time, go to System Settings, Privacy & Security, click Open Anyway, then open it again. Or, once the app is in Applications, run:

```
xattr -dr com.apple.quarantine /Applications/CadenceSlicer.app
```

It's Apple silicon only and needs macOS 11.3 or later.

On Windows, the first build isn't ready yet. It'll be added to this release when it is.

On Linux there's no build. It should build from source like OrcaSlicer, but I haven't tried it.

## Known issues

- On a small part the tower can take more time and filament than the part itself. Feature Split can be slower than one nozzle on small or short prints, so check the one-nozzle time on the Detail and speed step before you apply.
- Tree supports need the support body on the fine nozzle for now. With it on the coarse nozzle the slice is refused with a message. I'm working on this and it'll come in an update.
- Rafts need the support body on the fine nozzle, and they aren't supported in Body Split yet.
- With the support body on the coarse nozzle, the supports use more filament than OrcaSlicer's would. On models with lots of small contact areas, the support body on the fine nozzle can be just as fast, so compare both before a long print.
- The prime tower can get small blobs in its purge area where coarse and fine layers overlap. The part isn't affected.
- In Body Split with the support body on the coarse nozzle, some plates are refused with a prime tower message. Put the support body on the fine nozzle's filament.
- The coarse support body can get a few thin layers just under the interface. They print fine.
- With the support body on the coarse nozzle, the part can get a wider brim than OrcaSlicer would give it.
- Setup's last step doesn't say which nozzle prints the supports. Check Support/raft base and Support/raft interface before slicing.
- Setup can say Ready to apply while the picked coarse layer reads "no time: slicing would refuse this choice". Slicing then stops with a message that says what to change, and nothing wrong gets printed.
- After Apply in "Edit here", the Objects list can still show the old filament number. The G-code uses the new one.
- An exact filament slot set in More options reopens as "Set by Fine or Coarse".
- Some small display glitches: Step 2 can name the wrong slot in its hint line, the nozzle flow names can be cut short in the Printer panel, and the preview legend can be clipped in a small window. The legend's top height includes the start purge line, as in OrcaSlicer.
- Undo right after picking a nozzle diameter doesn't work while the list still has focus. Click elsewhere first, or use Edit, Undo.
- Settings that don't fit the layer grid are refused with a plain message that names the setting. See [Supported](SUPPORTED.md#known-issues).

## Reporting bugs

Please open an issue with the bug report template and attach the 3MF. With the 3MF I can usually reproduce the slice exactly. The template also asks for the printer, the nozzle pair and flow types, the materials, which mode you used, and photos if it's a print problem. If a slice is refused, include the "Details:" line from the message.

If you print on a printer I haven't, I'd like to hear how it went, good or bad.

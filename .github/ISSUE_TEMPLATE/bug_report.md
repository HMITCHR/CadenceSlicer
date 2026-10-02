---
name: Bug report
about: Something sliced or printed wrong, or the app misbehaved
title: ""
labels: bug
assignees: ""
---

Thanks for taking the time. The 3MF is the most useful thing here. With it I can usually reproduce the slice exactly. If you can't share the model, say so and I'll work with what's below.

**Build**
<!-- Help, About shows the version and build. -->
- Version and build hash:
- macOS or Windows, and version:

**Printer**
- Printer model:
- Firmware version:

**Nozzles**
<!-- For example: left 0.2 Standard, right 0.8 High Flow -->
- Left nozzle and flow type:
- Right nozzle and flow type:

**Materials**
<!-- Brand and type for each, and which nozzle each was on. For example: Bambu PLA Basic on the left, generic PETG HF on the right -->
- Fine material:
- Coarse material:
- Any others (supports, interfaces, extra Body Split parts):

**Mode**
<!-- Feature Split, Body Split, or Off. Plus the fine layer and N (or coarse layer) if you know them. -->
- Mode:
- Fine layer and coarse layer:
- Process preset:

**Files**
<!-- Drag the files into this box. GitHub needs .3mf and .gcode zipped. -->
- [ ] The project 3MF (File, Save Project As)
- [ ] The G-code header: open the .gcode in a text editor and paste everything from `; HEADER_BLOCK_START` to `; HEADER_BLOCK_END`, plus the `; CONFIG_BLOCK_START` section at the end if you can. Or just attach the whole .gcode zipped.
- [ ] If setup, the sidebar or a dialog misbehaved: the newest log from `~/Library/Application Support/CadenceSlicer/log/` (macOS) or `%APPDATA%\CadenceSlicer\log\` (Windows)

**What happened**
<!-- What you saw. If the slicer refused, paste the message exactly, including the Details line if there is one (for example "Details: SRL-A30"). -->

**What you expected**

**Steps to reproduce**
1.
2.
3.

**Photos**
<!-- For a print problem: the failed area, the prime tower, and a band change up close if it's visible. A photo of the first layer helps with anything near the bed. -->

**Crash report**
<!-- If the app crashed and your operating system offered a crash report, paste it here or attach it. -->

**Anything else**
<!-- Does the same thing happen with mixed-nozzle slicing off? Did it work in an earlier build? -->

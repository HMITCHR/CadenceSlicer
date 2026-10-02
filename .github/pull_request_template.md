## What this changes

<!-- What the change does and why. If it closes an issue, say which. -->

## How it was tested

<!--
Name the suites and filters you ran, and paste the result lines.

  libslic3r_tests   '[AppConfig]'
  fff_print_tests   '[MixedNozzleCadence]'
  slic3rutils_tests '[MixedNozzleWizard]'

Slicing changes need a before and after: the same 3MF sliced on main and on
this branch, with what differs in the G-code.
-->

## Things to check before asking for review

- [ ] Builds on macOS arm64.
- [ ] The three test suites pass, or the failures are named above with the reason.
- [ ] Mixed-nozzle mode off still produces the same G-code as before, if this
      touches slicing.
- [ ] No new network call, and no new outbound host.
- [ ] Anything new that ships is listed in `THIRD-PARTY-LICENSES.md`.
- [ ] Upstream copyright and licence headers are intact in every file touched.

## Notes for the maintainer

<!--
Anything that does not fit above: a decision you were unsure about, a follow-up
you deliberately left out, or a part you want looked at closely.
-->

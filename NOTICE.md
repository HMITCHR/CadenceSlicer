# Notice

Cadence Slicer is a modified version of OrcaSlicer.

The modifications were made by HMR3D. OrcaSlicer is not involved in this fork and
has not reviewed or endorsed it.

## Licence

Cadence Slicer is distributed under the GNU Affero General Public License,
version 3, the same licence as OrcaSlicer. The full text is in `LICENSE.txt` at
the root of this repository, unmodified. Copyright notices and licence headers in
the files this fork inherited are kept as they were.

Using this software in any way, including behind a web server, carries the same
licence requirement. See section 13 of the AGPL.

## Corresponding source

The complete corresponding source for every released build is published at:

    https://github.com/HMITCHR/CadenceSlicer

That is the intended public repository. The owner will confirm the final path
before the first public release, and this file will be updated to match if it
changes.

Each released binary is tagged in that repository. A release and its source are
published at the same time, so anyone who receives a build can rebuild it from
the tag it was built from, using the build instructions in the repository.

## Upstream base

This fork branched from OrcaSlicer upstream `main` at commit:

    c806a09c7cfaaf4c0d19aca7f6cb505487c8ecc8

The application shows the short form of that commit in its About dialog, so any
build can be traced back to the source it was derived from.

## Statement of modifications

Section 5(a) of the AGPL requires that modified files carry notices stating that
they were changed and when. This fork changes several hundred files, so a header
notice in each one would be unreadable and would go stale. This notice stands in
its place:

Files in this repository have been modified by HMR3D from the OrcaSlicer source
at commit `c806a09c7c`, in 2026 and after. The modifications are identifiable in
full from the repository history, by comparing any tag against that base commit.
The main areas changed are mixed-nozzle slicing (Feature Split and Body Split),
synchronized regional layers, prime-pad deposition, the wizard and sidebar that
drive them, the presets they need, and their tests. Branding, network endpoints
and packaging metadata have also been changed.

## Lineage

Open source slicing is a chain of derived work, and each project in it credits
the ones before.

Slic3r was created by Alessandro Ranellucci and the RepRap community.
PrusaSlicer, by Prusa Research, built on Slic3r. Bambu Studio, by Bambu Lab,
forked from PrusaSlicer. SuperSlicer extended PrusaSlicer. OrcaSlicer drew from
PrusaSlicer, Bambu Studio, SuperSlicer and CuraSlicer, and grew well beyond them.
Cadence Slicer is a fork of OrcaSlicer.

The copyrights and licences of all of those projects are kept in this source
tree. See `THIRD-PARTY-LICENSES.md` for the libraries, fonts and profiles that
ship with a build.

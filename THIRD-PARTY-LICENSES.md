# Third-party components

Cadence Slicer is licensed under the GNU Affero General Public License version 3
(see `LICENSE.txt` and `NOTICE.md`). It builds on, links against and ships a
number of components that carry their own licences. This file lists them.

The list below is derived from the build files in this repository, `deps/` and
`deps_src/`, not from memory. Where a component ships its own licence text, the
path to that text is given and that text is authoritative. Where no licence file
is vendored here, the licence named is the one the upstream project publishes for
the version pinned by the build, and the authoritative text is in that project's
own distribution.

## How to read the licence column

Several of these are copyleft. Two are worth calling out:

- **mcut** is offered under GPL-3.0 or a commercial licence. This build uses the
  GPL option. AGPL-3.0 is compatible with that. See
  `deps_src/mcut/LICENSE.txt` and `deps_src/mcut/LICENSE.GPL.txt`.
- **libnest2d** is LGPL-3.0. See `deps_src/libnest2d/LICENSE.txt`.

## Built dependencies (`deps/`)

These are fetched and built by the dependency build, then linked into the
application. Versions are the ones pinned in `deps/<name>/<name>.cmake`.

| Component | Version | Licence |
| --- | --- | --- |
| Boost | 1.84.0 | Boost Software License 1.0 |
| c-blosc | 1.17.0 (fork `tamasmeszaros/c-blosc`) | BSD-3-Clause |
| Cereal | 1.3.0 | BSD-3-Clause |
| CGAL | 5.6.3 | GPL-3.0 and LGPL-3.0 by module |
| curl | 7.75.0 | curl licence (MIT/X derivative) |
| Draco | 1.5.7 | Apache-2.0 |
| Eigen | 5.0.1 | MPL-2.0 (with some modules under LGPL) |
| Expat | vendored copy under `deps/EXPAT/expat` | MIT |
| FreeType | 2.12.1 | FreeType License (BSD style) or GPL-2.0 |
| GLEW | git 3a8eff7 | Modified BSD, Mesa 3-D, Khronos |
| GLFW | 3.4 | zlib/libpng |
| GMP | 6.2.1 | LGPL-3.0 or GPL-2.0 |
| libjpeg-turbo | 3.0.1 | BSD-3-Clause, IJG, zlib |
| libnoise | 1.0 (fork `SoftFever/Orca-deps-libnoise`) | LGPL-2.1 |
| libpng | 1.6.35 | PNG Reference Library License (libpng) |
| MPFR | 4.2.2 | LGPL-3.0 |
| NanoSVG | git 863f6aa (fork `SoftFever/nanosvg`) | zlib |
| NLopt | 2.5.0 | LGPL-2.1 and MIT by module |
| OCCT (Open CASCADE) | 7.6.0 | LGPL-2.1 with an exception |
| OpenCSG | 1.4.2 | GPL-2.0 |
| OpenCV | 4.6.0 | Apache-2.0 |
| OpenEXR | 2.5.5 | BSD-3-Clause |
| OpenSSL | 1.1.1w | OpenSSL License and SSLeay License |
| OpenVDB | git a68fd58 (fork `tamasmeszaros/openvdb`) | MPL-2.0 |
| CPython | 3.12.13 | Python Software Foundation License 2.0 |
| Qhull | 8.0.2 | Qhull licence (BSD style) |
| oneTBB | 2021.5.0 | Apache-2.0 |
| wxWidgets | 3.3.2 (fork `SoftFever/Orca-deps-wxWidgets`) | wxWindows Library Licence |
| wxInspector | 1.0.0 | MIT |
| zlib | 1.2.11 | zlib |

Windows-only: WebView2 (Microsoft SDK, proprietary redistributable). Not used by
the macOS build, which is the only build this project produces today.

## Vendored sources (`deps_src/`)

These are compiled directly from source held in this repository.

| Component | Licence | Licence text in tree |
| --- | --- | --- |
| admesh | GPL-2.0 | headers in `deps_src/admesh` |
| AGG (Anti-Grain Geometry) | AGG licence (BSD style) | `deps_src/agg/copying` |
| ankerl (unordered_dense) | MIT | headers in `deps_src/ankerl` |
| Clipper | Boost Software License 1.0 | headers in `deps_src/clipper` |
| Clipper2 | Boost Software License 1.0 | headers in `deps_src/clipper2` |
| earcut.hpp | ISC | `deps_src/earcut/LICENSE` |
| Expat | MIT | `deps_src/expat` headers |
| fast_float | Apache-2.0 or MIT or BSL-1.0 | headers in `deps_src/fast_float` |
| glu-libtess | SGI Free Software License B 2.0 | headers in `deps_src/glu-libtess` |
| hidapi | BSD-3-Clause or GPL-3.0 or original HIDAPI licence | headers in `deps_src/hidapi` |
| Dear ImGui | MIT | `deps_src/imgui/LICENSE.txt` |
| ImGuizmo | MIT | `deps_src/imguizmo/LICENSE` |
| libigl | MPL-2.0 | headers in `deps_src/libigl` |
| libnest2d | LGPL-3.0 | `deps_src/libnest2d/LICENSE.txt` |
| mcut | GPL-3.0 or commercial (GPL option used here) | `deps_src/mcut/LICENSE.txt`, `deps_src/mcut/LICENSE.GPL.txt` |
| md4c | MIT | `deps_src/md4c/LICENSE.md` |
| mdns | Public domain or MIT | headers in `deps_src/mdns` |
| miniLZO | GPL-2.0 | `deps_src/minilzo/COPYING` |
| miniz | MIT | `deps_src/miniz/LICENSE` |
| NanoSVG | zlib | headers in `deps_src/nanosvg` |
| nlohmann/json | MIT | headers in `deps_src/nlohmann` |
| pybind11 | BSD-3-Clause | `deps_src/pybind11/LICENSE` |
| Qhull | Qhull licence (BSD style) | `deps_src/qhull/COPYING.txt` |
| qoi | MIT | headers in `deps_src/qoi` |
| semver.c | MIT | headers in `deps_src/semver` |
| Shiny profiler | zlib | headers in `deps_src/Shiny` |
| stb_dxt | MIT or public domain | headers in `deps_src/stb_dxt` |
| Catch2 (tests only, not shipped) | BSL-1.0 | `tests/catch2/LICENSE.txt` |

## Fonts

Shipped in `resources/fonts/`.

| Font | Licence |
| --- | --- |
| HarmonyOS Sans SC (Regular, Bold) | HarmonyOS Sans licence, free to use and redistribute |
| NanumGothic (Regular, Bold) | SIL Open Font License 1.1, see `resources/fonts/OFL.txt` |
| Noto Sans KR (Regular, Bold) | SIL Open Font License 1.1 |
| Sarabun (Medium, SemiBold) | SIL Open Font License 1.1 |

## Icons and application artwork

The Cadence Slicer application icon and its sized variants
(`resources/images/CadenceSlicer*`) are original work commissioned for this fork.
Their provenance is recorded in `resources/images/CadenceSlicer-asset.md`.

The rest of `resources/images` and the SVG icon set are inherited from OrcaSlicer
and remain under that project's licence.

## Printer and filament profiles

Profiles live in `resources/profiles/<Vendor>/`. They are inherited from
OrcaSlicer, which in turn inherits many of them from Bambu Studio, and they come
to this fork under the repository's AGPL-3.0 licence. There is no separate
licence file for them in this repository and none upstream, so that is the only
statement that applies to them.

Two points a reader should know:

- The `BBL` profiles describe Bambu Lab printers and Bambu Lab filaments. They
  were written by Bambu Lab and are redistributed here as they reach this fork
  through OrcaSlicer. Being able to slice for a printer does not mean the printer
  vendor was involved in this software.
- This fork modifies two files that upstream inherited unchanged from Bambu Lab:
  `resources/profiles/BBL.json`, the vendor index, and
  `resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json`, whose filament
  change G-code is rewritten. Both edits are ours and are visible in the history
  against the upstream base commit named in `NOTICE.md`.

## Sample models

None are shipped. `resources/handy_models/` contains the geometry OrcaSlicer
ships for its calibration and bed helpers, which arrives under that project's
licence. If sample print models are added later, their source and licence go in
this file first.

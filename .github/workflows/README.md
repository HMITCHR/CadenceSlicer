# GitHub Actions workflows

None of these workflows publishes anything: no releases, no tags, no package
uploads, and none of them needs a secret. Their only outputs are run
artifacts, check results and the Actions cache.

| Workflow | Runs on | When it runs |
|---|---|---|
| `windows-build.yml` | windows-2025, x64 | By hand (Actions tab, Run workflow), or when a `v*` tag is pushed |
| `windows-deps.yml` | windows-2025, x64 | Only when `windows-build.yml` calls it because the dependency cache is empty |
| `macos-build-test.yml` | macOS arm64 | On every push to `main`, on pull requests, and by hand |
| `check_profiles.yml` | Ubuntu | On pull requests that touch `resources/profiles/`, and by hand (from OrcaSlicer) |
| `shellcheck.yml` | Ubuntu | When shell scripts change, once a day, and by hand (from OrcaSlicer) |

`.github/dependabot.yml` (from OrcaSlicer) opens a pull request once a month
when a GitHub Action or Docker image has a new major version.

Things to know:

- The first Windows run builds every dependency and takes a few hours. Later
  runs reuse the cache and take one to one and a half hours. Run it once by
  hand on `main` first so tag builds can find the cache.
- The installer and portable folder are attached to the run as artifacts. A
  release is made by hand from those.
- `.github/run-suite.sh` runs one test suite with a tag filter. The workflows
  use it, and it works locally too.
- OrcaSlicer's other workflows (release publishing, translation bots, issue
  triage) are not included, because they post to OrcaSlicer's own services.

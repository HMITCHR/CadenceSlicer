# Privacy and network use

With a fresh install and default settings, Cadence Slicer doesn't connect to anything, including when you launch, slice or export. Every network call it can make is either to a printer or service you set up, a service you signed in to, or a link you clicked.

This page lists all of them. It comes from reading the source for every place the app can make a request.

## What I changed from OrcaSlicer

OrcaSlicer, and Bambu Studio before it, check in with their servers when they start. I turned those off, because a fork shouldn't be calling home to someone else's servers on their behalf, and I don't run a server of my own.

- OrcaSlicer checks `check-version.orcaslicer.com` on launch. Cadence has no update endpoint, so it doesn't check. Preferences says so, and new builds are on the GitHub releases page.
- OrcaSlicer can fetch profile updates from `check-version.orcaslicer.com/profile`. That's off too. The printer and filament profiles that ship with the app are still installed and refreshed locally, which doesn't touch the network.
- The inherited code ran a background updater on every launch that contacted `api.bambulab.com` to sync plug-in and printer resources, even if you never signed in. That's off.
- Requests made by the app's own code and the built-in web view identify it as `CadenceSlicer/<version>`. Some inherited printer integrations still use the names their services expect: Klipper/Moonraker and 3DPrinterOS see "OrcaSlicer", Elegoo sees "ElegooSlicer", SimplyPrint sees "SimplyPrint Orca Plugin", and the Bambu plug-in, if you turn it on, is told the client is "BambuStudio" because Bambu's servers require that.

## Off unless you turn it on

None of these do anything on a fresh install.

| What | Where it connects | When |
|---|---|---|
| Bambu network plug-in | `api.bambulab.com`, then the download address it returns | Only if you turn on "Enable Bambu network plug-in" in Preferences and accept the download. It's off by default. |
| Everything the Bambu plug-in does | Bambu's servers | The plug-in is Bambu Lab's closed-source component, not part of Cadence. Once installed it makes its own connections, which I can't see or control. |
| Printer health messages (HMS) | `e.bambulab.com` | Only with the plug-in on and stealth mode off. |
| Bambu privacy policy check | `api.bambulab.com` | Each time you sign in to an account in the app, Orca Cloud or Bambu. This is inherited and asks Bambu for the current privacy policy version. Stealth mode skips it. |
| Orca Cloud sign-in and preset sync | `auth.orcaslicer.com`, `cloud.orcaslicer.com`, `api.orcaslicer.com` | Only if you sign in to Orca Cloud in Cadence. While you're signed in, the app refreshes the sign-in and checks your cloud plugins each time it starts. Cadence keeps its own sign-in and doesn't use OrcaSlicer's. |
| Plugins | `api.orcaslicer.com`, then the download address it returns, and the Python package index if the plugin needs extra packages | Only if you install a plugin. The plugin hub itself opens in your browser. Plugins are Python code written by other people and can make their own connections. |
| Sending to a printer | whatever address you entered for it, usually on your own network | Only when you add a printer and send to it. Covers OctoPrint, Klipper and Moonraker, Duet, Repetier, PrusaLink, FlashAir, ESP3D, AstroBox, MKS, Flashforge, Elegoo, Creality, Qidi and Snapmaker hosts. |
| Hosted print services | `simplyprint.io`, `app.obico.io` (or your own Obico server), `cloud.3dprinteros.com`, `connect.prusa3d.com` | Only if you set one up as a print host. The SimplyPrint integration identifies itself to SimplyPrint as "SimplyPrint Orca Plugin", which is how SimplyPrint recognises it. |
| Printer discovery | a broadcast on your local network | Only when you look for printers. |
| Printer status and camera | the printer, on your local network | Only with a connected printer. |
| Model downloads | the link you opened, for example a Thingiverse or MakerWorld model link | Only when you follow a download link into the app. |
| Network test | `www.bing.com` and `github.com` | Only when you open Network test from Preferences or the Help menu and press the buttons. |
| Windows media help link | `support.microsoft.com` | Windows only, when the media player component is missing. |
| WebView2 runtime | `go.microsoft.com` | Windows only. If Microsoft's WebView2 runtime is missing, the app asks at startup and downloads the installer from Microsoft only if you click Yes. |

Stealth mode in Preferences signs you out and turns off the cloud features, if you want a second switch.

## Links you click

Some buttons and help links open a page in your own browser. The app doesn't send anything itself. Your browser makes the request as it would for any link. These go to:

- the OrcaSlicer wiki (`www.orcaslicer.com/wiki/...`), for most of the inherited help links. Cadence doesn't have its own versions of those pages.
- Bambu's wiki (`wiki.bambulab.com`) and a few short video links (`e.bambulab.com`), from the calibration and filament dialogs
- `status.bambulab.com`, Bambu's privacy policy, `makerworld.com`, `thingiverse.com`, `cloud.orcaslicer.com` from the home page, and GitHub for releases and issues

## Not in the app

- No telemetry or usage analytics. There's one inherited "send system info" dialog in the code, and nothing calls it.
- No crash reporter. If it crashes, your operating system handles it, and the bug report template asks you to paste the crash report if you have one.
- No automatic updates or background downloads.
- The app creates a random install ID the first time it runs and keeps it in its settings file. It's only sent anywhere if you turn on the Bambu plug-in, which passes it to Bambu along with the app version, your OS version and your language.
- No keys or tokens for any service of mine. There isn't a service of mine.

## Files it writes

Exported 3MF files contain an `Application` field set to a Bambu Studio version. That's a file-format compatibility field so Bambu Studio and the printer accept the file, not something sent anywhere. The G-code header says it was generated by Cadence Slicer.

Your settings and logs stay on your computer, in `~/Library/Application Support/CadenceSlicer/` on macOS and `%APPDATA%\CadenceSlicer\` on Windows. Nothing there is uploaded unless you attach it to an issue yourself.

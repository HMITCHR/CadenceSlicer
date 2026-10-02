# Generates the MSIX package logo assets from the approved Cadence master PNG
# (resources\images\CadenceSlicer.png). Each PNG is rendered at its exact
# target size and preserves the master alpha channel (the manifest uses
# BackgroundColor="transparent").
#
# Run once locally on Windows (re-run only if the logo changes), then commit
# the PNGs in assets/. CI never runs this script.
#
# The resize uses the Windows System.Drawing implementation available in
# Windows PowerShell/.NET; no additional package or network access is needed.
$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$source   = Join-Path $repoRoot 'resources\images\CadenceSlicer.png'
$outDir   = Join-Path $PSScriptRoot 'assets'
New-Item -ItemType Directory -Force $outDir | Out-Null

$sizes = [ordered]@{
    'Square150x150Logo.png'                              = 150
    'Square44x44Logo.png'                                = 44
    'Square44x44Logo.targetsize-44_altform-unplated.png' = 44
    'StoreLogo.png'                                      = 50
}

Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Drawing.Common -ErrorAction SilentlyContinue

$sourceBitmap = [System.Drawing.Bitmap]::new($source)
try {
    foreach ($name in $sizes.Keys) {
        $px = $sizes[$name]
        $outPath = Join-Path $outDir $name
        $bitmap = [System.Drawing.Bitmap]::new(
            $px,
            $px,
            [System.Drawing.Imaging.PixelFormat]::Format32bppArgb
        )
        try {
            $graphics = $null
            $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
            try {
                $graphics.CompositingMode = [System.Drawing.Drawing2D.CompositingMode]::SourceCopy
                $graphics.CompositingQuality = [System.Drawing.Drawing2D.CompositingQuality]::HighQuality
                $graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
                $graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
                $graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
                $graphics.Clear([System.Drawing.Color]::Transparent)
                $graphics.DrawImage($sourceBitmap, 0, 0, $px, $px)
            }
            finally {
                if ($null -ne $graphics) { $graphics.Dispose() }
            }
            $bitmap.Save($outPath, [System.Drawing.Imaging.ImageFormat]::Png)
            Write-Host "Wrote $name ($px`x$px)"
        }
        finally {
            $bitmap.Dispose()
        }
    }
}
finally {
    $sourceBitmap.Dispose()
}

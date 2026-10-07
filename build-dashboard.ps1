param(
    [ValidateSet('current', 'win', 'mac', 'linux')]
    [string]$Platform = 'current',
    [switch]$Publish,
    [switch]$SkipInstall
)

$ErrorActionPreference = 'Stop'
$repoRoot = $PSScriptRoot
$toolRoot = Join-Path $repoRoot 'power-tool'
if (-not (Test-Path -LiteralPath (Join-Path $toolRoot 'package.json'))) {
    throw "Could not find Power Tool at $toolRoot"
}
$isWindowsHost = [System.Environment]::OSVersion.Platform -eq 'Win32NT'
$npmName = if ($isWindowsHost) { 'npm.cmd' } else { 'npm' }
$npm = Get-Command $npmName -CommandType Application -ErrorAction SilentlyContinue
if (-not $npm) {
    throw 'Install Node.js 22 LTS (including npm), then open a new terminal and retry.'
}

function Invoke-Npm {
    param([string[]]$NpmArguments)
    # Windows PowerShell must allow npm warnings on stderr to finish normally.
    $ErrorActionPreference = 'Continue'
    $PSNativeCommandUseErrorActionPreference = $false
    & $npm.Source @NpmArguments
    if ($LASTEXITCODE -ne 0) {
        throw "npm $($NpmArguments -join ' ') failed with exit code $LASTEXITCODE."
    }
}

Push-Location $toolRoot
try {
    if (-not $SkipInstall) { Invoke-Npm -NpmArguments @('ci') }
    Invoke-Npm -NpmArguments @('run', 'build')
    $outputName = if ($Publish) { 'PowerTool' } else { 'PowerTool-local' }
    $outputRoot = Join-Path $repoRoot $outputName
    $builderArguments = @('--dir', "--config.directories.output=$outputRoot")
    if ($Platform -ne 'current') { $builderArguments += "--$Platform" }
    # Local packages do not need a signing certificate or a downloaded signing tool.
    if ($isWindowsHost -and $Platform -in @('current', 'win')) {
        $builderArguments += '--config.win.signAndEditExecutable=false'
        $builderArguments += "--config.electronDist=$(Join-Path $toolRoot 'node_modules/electron/dist')"
    }
    $builder = Join-Path $toolRoot 'node_modules/electron-builder/cli.js'
    $ErrorActionPreference = 'Continue'
    $PSNativeCommandUseErrorActionPreference = $false
    & node $builder @builderArguments
    $builderExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($builderExit -ne 0) { throw "Electron packaging failed with exit code $builderExit." }
    Write-Host "Power Tool app written to $outputRoot"
    if ($isWindowsHost -and $Platform -in @('current', 'win')) {
        Write-Host "Launch: $(Join-Path $outputRoot 'win-unpacked/Power Tool.exe')"
    }
} finally {
    Pop-Location
}

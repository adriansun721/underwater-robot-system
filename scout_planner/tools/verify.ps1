[CmdletBinding()]
param(
  [ValidateSet("verify", "baseline")]
  [string]$Preset = "verify"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repositoryRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$reportPath = Join-Path $repositoryRoot "build\$Preset\ctest.xml"
$vswhere = Join-Path ${env:ProgramFiles(x86)} `
  "Microsoft Visual Studio\Installer\vswhere.exe"

if (-not (Test-Path -LiteralPath $vswhere)) {
  throw "Visual Studio Installer's vswhere.exe was not found. Install the C++ Build Tools workload."
}

$installationPath = & $vswhere -latest -products "*" `
  -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
  -property installationPath
if (-not $installationPath) {
  throw "No Visual Studio installation with the MSVC x64/x86 tools was found."
}

$developerPrompt = Join-Path $installationPath "Common7\Tools\VsDevCmd.bat"
$cmake = Join-Path $installationPath `
  "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

foreach ($requiredFile in @($developerPrompt, $cmake)) {
  if (-not (Test-Path -LiteralPath $requiredFile)) {
    throw "Required build tool was not found: $requiredFile"
  }
}

$commandLine = @(
  "call `"$developerPrompt`" -arch=amd64",
  "cd /d `"$repositoryRoot`"",
  "`"$cmake`" --workflow --preset $Preset"
) -join " && "

$stopwatch = [Diagnostics.Stopwatch]::StartNew()
& $env:ComSpec /d /s /c $commandLine
$exitCode = $LASTEXITCODE
$stopwatch.Stop()

Write-Host "[$Preset] elapsed_ms=$($stopwatch.ElapsedMilliseconds) report=$reportPath"
if ($exitCode -ne 0) {
  exit $exitCode
}

$ErrorActionPreference = 'Stop'

$interfacesRoot = Split-Path -Parent $PSScriptRoot
Push-Location (Split-Path -Parent $interfacesRoot)
try {
    python -B -m unittest discover -s interfaces/tests -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}

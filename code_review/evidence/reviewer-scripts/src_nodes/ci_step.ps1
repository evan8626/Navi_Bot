$ErrorActionPreference = 'stop'
python -c "import sys; print('suite A fails'); sys.exit(1)"
python -c "print('suite B (last) passes')"
if ((Test-Path -LiteralPath variable:\LASTEXITCODE)) { exit $LASTEXITCODE }

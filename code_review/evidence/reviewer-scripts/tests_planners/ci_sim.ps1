$ErrorActionPreference = "stop"
python -c "import sys; sys.exit(1)"
python -c "import sys; sys.exit(0)"
if ((Test-Path -LiteralPath variable:\LASTEXITCODE)) { exit $LASTEXITCODE }

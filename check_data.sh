#!/bin/sh
# Run with: sh check_data.sh [arguments for the Python test runner]

EDR_SCRIPT_DIR=$(CDPATH= cd -P "$(dirname "$0")" && pwd) || exit 1

if command -v python3 >/dev/null 2>&1 &&
    python3 -c 'import sys; sys.exit(0 if sys.version_info.major == 3 else 1)' \
        >/dev/null 2>&1; then
    exec python3 -B "$EDR_SCRIPT_DIR/testing/data_representation/run.py" \
        --report "$EDR_SCRIPT_DIR/edr-check-report.json" "$@"
fi

if command -v python >/dev/null 2>&1 &&
    python -c 'import sys; sys.exit(0 if sys.version_info.major == 3 else 1)' \
        >/dev/null 2>&1; then
    exec python -B "$EDR_SCRIPT_DIR/testing/data_representation/run.py" \
        --report "$EDR_SCRIPT_DIR/edr-check-report.json" "$@"
fi

echo 'ERROR: Python 3 was not found. Install Python 3 and add it to PATH.' >&2
exit 127

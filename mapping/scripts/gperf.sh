#!/bin/bash

###
### Run the mapper node in bag reader mode with gperftools CPU profiling.
###
### Usage:
###   ./gperf.sh [-o <profile_output.prof>] --config <config.yaml> --bag <bag_path>
###
### Example:
###   ./gperf.sh --config config/mapping_mid360.yaml --bag /data/bags/office_scan
###   ./gperf.sh -o /path/to/profile.prof --config config/mapping_mid360.yaml --bag /data/bags/office_scan
###
### Press CTRL+C to stop early — the profile data will be saved before exiting.
###

PROF_OUTPUT="/tmp/mapper_profile.prof"
MAPPER_ARGS=()

# Parse arguments: extract --output/-o for ourselves, pass everything else to mapper
while [[ $# -gt 0 ]]; do
    case "$1" in
        -o|--output)
            PROF_OUTPUT="$2"
            shift 2
            ;;
        *)
            MAPPER_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ ${#MAPPER_ARGS[@]} -eq 0 ]; then
    echo "Usage: $0 [-o|--output profile.prof] --config <config.yaml> --bag <bag_path>"
    exit 1
fi

export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libprofiler.so
export CPUPROFILE="$PROF_OUTPUT"

echo "Starting mapper with CPU profiling..."
echo "Profile output: $PROF_OUTPUT"
echo "Press CTRL+C to stop early and save profile."
echo ""

# Run mapper in the background, inheriting the script's process group so that
# terminal SIGINT reaches both the script and the mapper directly.
$(pwd)/build/mapper "${MAPPER_ARGS[@]}" &
MAPPER_PID=$!

echo "Profiling started (PID: $MAPPER_PID)"

# Cleanup: mapper already received SIGINT from the terminal; just wait for its
# graceful shutdown so the gperftools atexit handler can write the profile.
cleanup() {
    echo ""
    echo "Waiting for mapper to shut down..."
    wait "$MAPPER_PID" 2>/dev/null
    echo "Profile saved to: $PROF_OUTPUT"
    exit 0
}

trap cleanup INT TERM

# Wait for natural completion (mapper reads entire bag and exits)
wait "$MAPPER_PID" 2>/dev/null

echo "Mapper finished. Profile saved to: $PROF_OUTPUT"

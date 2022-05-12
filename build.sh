#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "Building"

/usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-renesas-ck-ra6m5 || true &

pid=$! # Process Id of the previous running command
while kill -0 $pid 2>/dev/null
do
    echo "Still building..."
    sleep 2
done

wait $pid
if [ -f /app/workspace/firmware-renesas-ck-ra6m5/Debug/firmware-renesas-ck-ra6m5.hex ]; then
    echo "Building done"
    exit 0
else
    echo "Building failed"
    exit 1
fi

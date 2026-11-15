#!/bin/sh
# Compiles the real DriveController/PIDController against stub FTC classes and runs the simulation.
# Usage: sim/run.sh [Sim|Sweep|Dead]   (default Sim)
set -e
cd "$(dirname "$0")"
OUT=$(mktemp -d)
T=../TeamCode/src/main/java/org/firstinspires/ftc/teamcode
javac -d "$OUT" $(find stubs -name '*.java') harness/*.java $T/DriveController.java $T/PIDController.java
java -cp "$OUT" org.firstinspires.ftc.teamcode.${1:-Sim}

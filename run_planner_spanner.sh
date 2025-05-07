#!/bin/bash

PROBLEMS_DIR=downward-benchmarks/ipc-11/spanner/learning-phase

OUTPUT_DIR="spanner_outputs"
mkdir -p "$OUTPUT_DIR"

for PROBLEM in ${PROBLEMS_DIR}/*.pddl; do
    BASENAME=$(basename "$PROBLEM" .pddl)
    echo "Running on $PROBLEM"
    ./fast-downward.py $PROBLEM --search "eager_greedy([spanner_heuristic()])" > "${OUTPUT_DIR}/output_${BASENAME}.txt" 2>&1
done

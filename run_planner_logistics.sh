#!/bin/bash

PROBLEMS_DIR=downward-benchmarks/logistics98

OUTPUT_DIR="logistics_outputs"
mkdir -p "$OUTPUT_DIR"

for PROBLEM in ${PROBLEMS_DIR}/*.pddl; do
    BASENAME=$(basename "$PROBLEM" .pddl)
    echo "Running on $PROBLEM"
    ./fast-downward.py $PROBLEM --search "eager_greedy([logistics_heuristic()])" > "${OUTPUT_DIR}/output_${BASENAME}.txt" 2>&1
done

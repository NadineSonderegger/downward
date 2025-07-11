#!/bin/bash

# Directory containing output txt files
INPUT_DIR="blocksworld_infinity_outputs"
OUTPUT_FILE="blocksworld_infinity_outputs/final_results.csv"

# Initialize the output file with a header
echo "File Name, Expanded States, Plan Length" > "$OUTPUT_FILE"

# Loop through all .txt files
for FILE in "$INPUT_DIR"/*.txt; do
    BASENAME=$(basename "$FILE" .txt)

    # Initialize variables
    EXPANDED_STATES=""
    PLAN_LENGTH=""

    while IFS= read -r line; do
        # Match Plan length
        if [[ "$line" =~ Plan\ length:\ ([0-9]+) ]]; then
            PLAN_LENGTH="${BASH_REMATCH[1]}"
        fi

        # Match Expanded states
        if [[ "$line" =~ Expanded\ ([0-9]+)\ state ]]; then
            EXPANDED_STATES="${BASH_REMATCH[1]}"
        fi
    done < "$FILE"

    # Write to output if both values were found
    if [[ -n "$EXPANDED_STATES" && -n "$PLAN_LENGTH" ]]; then
        echo "$BASENAME, $EXPANDED_STATES, $PLAN_LENGTH" >> "$OUTPUT_FILE"
    else
        echo "Warning: Missing data in $BASENAME"
    fi
done

echo "Results written to $OUTPUT_FILE"

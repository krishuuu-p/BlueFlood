#!/bin/bash
# Analyze logs and extract final level_calculated for each node

LOG_DIR="dcube-exp-logs/dcube_test_bed_logs"

for log in "$LOG_DIR"/log_*.txt; do
    node=$(basename "$log" .txt | sed 's/log_//')
    level=$(grep "{rx-" "$log" 2>/dev/null | tail -1 | awk -F', ' '{print $NF}')
    [ -n "$level" ] && echo "$level $node"
done | sort -n | awk '{printf "Level %2d: Node %s\n", $1, $2}'

#!/bin/bash
# Remove CBS/mqprio qdisc stack, restore default pfifo_fast
#
# Usage: sudo ./teardown-cbs.sh [interface]

set -e

IFACE=${1:-eth1}

echo "Removing qdisc stack on $IFACE"
tc qdisc del dev "$IFACE" root 2>/dev/null || true

echo "Restored defaults:"
tc qdisc show dev "$IFACE"

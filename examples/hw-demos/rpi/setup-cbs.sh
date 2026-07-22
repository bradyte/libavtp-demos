#!/bin/bash
# Configure CBS (Credit-Based Shaper) for AAF stream on i226
#
# Sets up mqprio + cbs qdisc stack so AAF packets are hardware-paced
# at the correct rate regardless of talker software burst behavior.
#
# Usage: sudo ./setup-cbs.sh [interface]
#   interface: NIC to shape (default: eth1)
#
# Run the AAF talker with matching -p flag:
#   sudo ./aaf-talker-hw -i eth1 -p 3 ...
#
# Stream profile: 48kHz stereo S32_BE, 6 frames/PDU = 8000 pkt/s
#   Wire frame: 14(eth) + 72(avtp) + 4(fcs) = 90 bytes
#   With preamble+IFG: 110 bytes = 880 bits → 7.04 Mbps

set -e

IFACE=${1:-eth1}

echo "Configuring CBS on $IFACE (priority $PRIO → TC0, queue 0)"

# mqprio: 3 traffic classes (matches Intel igc example)
#   TC0 = queue 0 (shaped, AAF traffic via priority 3)
#   TC1 = queue 1 (best-effort elevated)
#   TC2 = queues 2-3 (default best-effort)
# Map: priority 3 → TC0, priority 2 → TC1, everything else → TC2
tc qdisc replace dev "$IFACE" root handle 6666: mqprio \
    num_tc 3 \
    map 2 2 1 0 2 2 2 2 2 2 2 2 2 2 2 2 \
    queues 1@0 1@1 2@2 \
    hw 0

# CBS on TC0 (queue 0, child of mqprio class 6666:1) with hardware offload
# Parameters for 1 Gbps link, 7.04 Mbps stream:
#   idleslope  = 7040 kbps (stream bandwidth)
#   sendslope  = 7040 - 1000000 = -992960 kbps
#   hicredit   = idleslope * maxFrameSize / linkSpeed = 7040 * 1542 / 1000000 ≈ 11
#   locredit   = sendslope * frameSize / linkSpeed = -992960 * 110 / 1000000 ≈ -110
tc qdisc replace dev "$IFACE" parent 6666:1 cbs \
    idleslope 7040 \
    sendslope -992960 \
    hicredit 11 \
    locredit -110 \
    offload 1

echo "CBS configured:"
tc -s qdisc show dev "$IFACE"

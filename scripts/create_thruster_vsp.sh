#!/bin/bash

# Utilize socat to create a virtual serial port
socat pty,raw,echo=0,link=to_thruster_vsp pty,raw,echo=0,link=from_thruster_vsp &
socat_pid=$!

# Get the absolute paths of the ports.
port_to_thrusters="`pwd`/to_thruster_vsp"
port_from_thrusters="`pwd`/from_thruster_vsp"

# Override parameters to the virtual serial ports.
rosparam set /ports/thruster $port_to_thrusters
rosparam set /simulator/ports/simulated_thruster $port_from_thrusters

# loop forever
while true
do
    sleep 60
done

trap "kill $socat_pid" EXIT

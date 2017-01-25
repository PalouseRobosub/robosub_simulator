#!/bin/bash

# Utilize socat to create a virtual serial port
(socat -d -d pty,raw,echo=0 pty,raw,echo=0 > tmp 2>&1 & echo $! > socat_pid) | tee tmp
socat_pid=`cat socat_pid`
points=`head -2 tmp | sed 's/.* PTY is \(.*\)/\1/'`
rm tmp
rm socat_pid

# Utilize regular expressions to determine the names of the serial ports created
# by socat.
points=`echo "$points" | tr '\n' ' '`
point_one=`echo "$points" | sed 's/\(.*\) \(.*\) /\1/'`
point_two=`echo "$points" | sed 's/\(.*\) \(.*\) /\2/'`

# Display which serial ports were created
echo "ThrusterController writes to: $point_one and Simulated Thrusters read from: $point_two"

# Override parameters to the virtual serial ports.
rosparam set /ports/thruster $point_one
rosparam set /ports/simulated_thruster $point_two

# loop forever
while true
do
    sleep 60
done

trap "kill $socat_pid" EXIT

#!/bin/bash

# Utilize socat to create a virtual serial port
socat pty,raw,echo=0,link=to_sensor_vsp pty,raw,echo=0,link=from_sensor_vsp &
socat_pid=$!

# Get the absolute paths of the ports.
port_to_sensors="`pwd`/to_sensor_vsp"
port_from_sensors="`pwd`/from_sensor_vsp"

# Override parameters to the virtual serial ports.
rosparam set /ports/sensor $port_to_sensors
rosparam set /simulator/ports/simulated_sensor $port_from_sensors

# loop forever
while true
do
    sleep 60
done

trap "kill $socat_pid" EXIT

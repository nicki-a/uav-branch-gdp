# README #

### How do I get set up? ###

* The entire program starts from mqtt-comms-unit
* It should automatically call system_control
* Each script that needs to talk to other scripts uses mqtt_pub commands to do so

* mqtt-comms-unit processes each command
* The possible commands are all listed at the bottom of mqtt-comms-unit, with a registered handler functions


### Testing & Deployment ###
* Use the configuration flags in config.py to enable or disable specific functionalities

### Contribution guidelines ###
* Pleeaseee try to work in your own branches until you're happy with it 

### Who do I talk to? ###
* Nicki Ashworth

### Full flow explanation: ###
* The main thread begins from the file
mqtt_comms_unit.py. It calls the class
MQTTCommsUnit twice, once for the
flight_controller subsystem and once for the
vision_system subsystem.
* Each class instance subscribes to the relevant
drone/[subsystem]/command MQTT topic and starts
the MQTT client loop in separate thread. The looping status update thread
is also created (orange).
* In the main thread, expected command names are reg-
istered with each class, linked to handler functions. Fi-
nally, class SystemControl is called.
* SystemControl uses MQTTPublish to execute the flight test sequence. It first com-
mands the Flight Control subsystem to take off to a
specified relative altitude, by sending a message on the
drone/flight_controller/command MQTT topic.
* This is received by the MQTT client loop
in the MQTTCommsUnit(flight_controller) class in-
stance, which spawns a command execution thread
that instructs FlightControl to send the cor-
responding MAVLink commands. Upon completion, a
response is published on the command_response topic
and the thread terminates.
* The main thread waits for a successful com-
mand response before proceeding to the next
step: instruct the vision system to start analysing
frames. This is performed by sending a com-
mand to the drone/vision_system/command topic
which is received by the MQTT client loop in
MQTTCommsUnit(vision_system).
* The command runs in a new thread, and starts
the video analysis loop thread in VisionSystem.
* The video processing loop in VisionSystem anal-
yses frames and generates differential velocity vec-
tors. It publishes the movement command on
topic drone/flight_controller/command, including
numerical values in the data packet.
* MQTTCommsUnit(flight_controller) executes this
command as above and returns a command_response
that includes relative altitude.
* SystemControl goes on to stop the vision loop and
perform a landing using the same process described
above.

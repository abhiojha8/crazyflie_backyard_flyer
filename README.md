# CrazyFlie Backyard Flyer
In this project, I have set up a state machine using event-driven programming to autonomously fly a drone. Initially I tested on a quadcopter in Unity [simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) provided by Udacity.

The python code is similar to how the drone would be controlled from a ground station computer or an onboard flight computer. Since communication with the drone is done using Mavlink, I was able to use this code to control an PX4 quadcopter autopilot with very little modification!

## Setting up Python Environment
I have used Python 3 along with the following packages:

- [`matplotlib`](https://matplotlib.org/)
- [`jupyter`](http://jupyter.org/)
- [`udacidrone`](https://github.com/udacity/udacidrone). As `udacidrone` is updated frequently, it's recommended to update udacidrone with `pip install --upgrade udacidrone`.
- [`visdom`](https://github.com/facebookresearch/visdom/)

The Anaconda environment can be setup using the `environment.yml` 

## Task
The required task is to command the drone to fly a 10 meter box at a 3 meter altitude. This is achieved in two ways: first using manual control and then under autonomous control.

Manual control of the drone is done using the instructions found with the [simulator](https://github.com/udacity/FCND-Simulator-Releases/releases).

Autonomous control is done using an event-driven state machine. Appropriate callbacks are created, which check against transition criteria dependent on the current state. If the transition criteria are met, it will transition to the next state and pass along any required commands to the drone.

Telemetry data from the drone is logged for review after the flight. These logs are used to plot the trajectory of the drone and analyze the performance of the task. For more information check out the Flight Log section below...

## Drone API

To communicate with the simulator (and a real drone), I have used the [UdaciDrone API](https://udacity.github.io/udacidrone/) provided by Udacity.  This API handles all the communication between Python and the drone simulator.  A key element of the API is the `Drone` superclass that contains the commands to be passed to the simulator and allows to register callbacks/listeners on changes to the drone's attributes.  In this project, I have designed a subclass from the Drone class implementing a state machine to autonomously fly a box. 

### Drone Attributes

The `Drone` class contains the following attributes:

 - `self.armed`: boolean for the drone's armed state
 - `self.guided`: boolean for the drone's guided state (if the script has control or not)
 - `self.local_position`: a vector of the current position in NED coordinates
 - `self.local_velocity`: a vector of the current velocity in NED coordinates

For a detailed list of all of the attributes of the `Drone` class [check out the UdaciDrone API documentation](https://udacity.github.io/udacidrone/).


### Registering Callbacks

As the simulator passes new information about the drone to the Python `Drone` class, the various attributes will be updated.  Callbacks are functions that can be registered to be called when a specific set of attributes are updated.  There are two steps needed to be able to create and register a callback:

1. Create the callback function:

Each callback function is defined as a member function of the `BackyardFlyer` class in `backyard_flyer.py` that takes in only the `self` parameter.  Here are the various callback methods:

```python
def local_position_callback(self):
        """ this is triggered when self.local_position contains new data """
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
                        
def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
                    
def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
```

2. Register the callback:

In order to have a callback function called when the appropriate attributes are updated, each callback needs to be registered.  This registration takes place in you `BackyardFlyer`'s `__init__()` function as shown below:

```python
class BackyardFlyer(Drone):

    def __init__(self, connection):
        ...
        
	   # registering callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
```

Since callback functions are only called when certain drone attributes are changed, the first parameter to the callback registration indicates for which attribute changes we want the callback to occur.  For example, here are some message id's that are useful for code implementation (for a more detailed list, see the UdaciDrone API documentation):

 - `MsgID.LOCAL_POSITION`: updates the `self.local_position` attribute
 - `MsgID.LOCAL_VELOCITY`: updates the `self.local_velocity` attribute
 - `MsgID.STATE`: updates the `self.guided` and `self.armed` attributes


### Outgoing Commands

The UdaciDrone API's `Drone` class also contains function to be able to send commands to the drone.  Here is a list of commands that are useful for this project:

 - `connect()`: Starts receiving messages from the drone. Blocks the code until the first message is received
 - `start()`: Start receiving messages from the drone. If the connection is not threaded, this will block the code.
 - `arm()`: Arms the motors of the quad, the rotors will spin slowly. The drone cannot takeoff until armed first
 - `disarm()`: Disarms the motors of the quad. The quadcopter cannot be disarmed in the air
 - `take_control()`: Set the command mode of the quad to guided
 - `release_control()`: Set the command mode of the quad to manual
 - `cmd_position(north, east, down, heading)`: Command the drone to travel to the local position (north, east, down). Also commands the quad to maintain a specified heading
 - `takeoff(target_altitude)`: Takeoff from the current location to the specified global altitude
 - `land()`: Land in the current position
 - `stop()`: Terminate the connection with the drone and close the telemetry log

These can be called directly from other methods within the drone class:

```python
self.arm() # Sends an arm command to the drone
```

### Manual Flight

To log data while flying manually, run the `drone.py` script as shown below:

```sh
python drone.py
```

Run this script after starting the simulator. It connects to the simulator using the Drone class and runs until TCP connection is broken. The connection will timeout if it doesn't receive a heartbeat message once every 10 seconds. The GPS data is automatically logged.

To stop logging data, stop the simulator first and the script will automatically terminate after approximately 10 seconds.

Alternatively, the drone can be manually started/stopped from a python/ipython shell:

```python
from drone import Drone
drone = Drone()
drone.start(threaded=True, tlog_name="TLog-manual.txt")
```

If `threaded` is set to `False`, the code will block and the drone logging can only be stopped by terminating the simulation. If the connection is threaded, the drone can be commanded using the commands described above, and the connection can be stopped (and the log properly closed) using:

```python
drone.stop()
```

When starting the drone manually from a python/ipython shell we have the option to provide a desired filename for the telemetry log file (such as "TLog-manual.txt" as shown above).  This allows us to customize the telemetry log name as desired to help keep track of different types of log files we might have.  Note that when running the drone from `python drone.py` for manual flight, the telemetry log will default to "TLog-manual.txt".

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt" or "Logs\TLog-manual.txt" for logs created when running `python drone.py`. Each row contains a comma seperated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contains all the message properties. The types of messages relevant to this project are:

* `MsgID.STATE`: time (ms), armed (bool), guided (bool)
* `MsgID.GLOBAL_POSITION`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.GLOBAL_HOME`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.LOCAL_POSITION`: time (ms), north (meter), east (meter), down (meter)
* `MsgID.LOCAL_VELOCITY`: time (ms), north (meter), east (meter), down (meter) 


#### Reading Telemetry Logs

Logs can be read using:

```python
t_log = Drone.read_telemetry_data(filename)
```

The data is stored as a dictionary of message types. For each message type, there is a list of numpy arrays. For example, to access the longitude and latitude from a `MsgID.GLOBAL_POSITION`:

```python
# Time is always the first entry in the list
time = t_log['MsgID.GLOBAL_POSITION'][0][:]
longitude = t_log['MsgID.GLOBAL_POSITION'][1][:]
latitude = t_log['MsgID.GLOBAL_POSITION'][2][:]
```

The data between different messages will not be time synced since they are recorded at different times.


## Autonomous Control State Machine

The state machine is run continuously until either the mission is ended or the Mavlink connection is lost.

The six states predefined for the state machine:
* MANUAL: the drone is being controlled by the user
* ARMING: the drone is in guided mode and being armed
* TAKEOFF: the drone is taking off from the ground
* WAYPOINT: the drone is flying to a specified target position
* LANDING: the drone is landing on the ground
* DISARMING: the drone is disarming

While the drone is in each state, we need to check transition criteria with a registered callback. If the transition criteria are met, we set the next state and pass along any commands to the drone. For example:

```python
def state_callback(self):
	if self.state == States.DISARMING:
    	if !self.armed:
        	self.release_control()
        	self.in_mission = False
        	self.state = States.MANUAL
```

This is a callback on the state message. It only checks anything if it's in the DISARMING state. If it detects that the drone is successfully disarmed, it sets the mode back to manual and terminates the mission.       

### Running the State Machine

After filling in the appropriate callbacks, we can run the mission:

```sh
python backyard_flyer.py
```

Similar to the manual flight, the GPS data is automatically logged to the specified log file.


### Reference Frames

Two different reference frames are used. Global positions are defined [longitude, latitude, altitude (pos up)]. Local reference frames are defined [North, East, Down (pos down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame . The global reference frame is what is provided by the Drone's GPS, but degrees are difficult to work with on a small scale. Conversion to a local frame allows for easy calculation of m level distances. Two convenience function are provided to convert between the two frames. These functions are wrappers on `utm` library functions.

```python
# Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)
def local_to_global(local_position, global_home):

# Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position
def global_to_local(global_position, global_home):
```



## Simulator Results

The simulation results can be seen here

[![video](https://github.com/abhiojha8/crazyflie_backyard_flyer/blob/master/result_video_img.PNG)](https://github.com/abhiojha8/crazyflie_backyard_flyer/blob/master/crazyflie_backyard_flier.mp4)



## CrazyFlie Integration

The CrazyFlie is able to be controlled (to an extent) through the Udacidrone API. 

I made 3 sets of modifications to our backyard flyer script to be able to control our crazyflie: 

1. Updated the connection
2. Modified arming and disarming flow
3. Updated Waypoints

### 1. Connection update

The default firmware of crazyflie uses it's own communication protocol, Crazy RealTime Protocol (CRTP), instead of Mavlink, therefore I changed to using the `CrazyflieConnection` that can be found in Udacidrone starting with version 0.3.0.

1. Add an import for `CrazyflieConnection`

```python
from udacidrone.connection import CrazyflieConnection
```

2. At the bottom of the `backyard_flyer.py` script, I have replaced my connection object

```python
# replace 
conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port))

# with
conn = CrazyflieConnection('radio://0/80/2M')
```

This creates a connection to the crazyflie. The input string is the URI of the crazyflie, which is defined as a string formatted as `'radio://interface id/interface channel/interface speed`. For this setup, I have kept the default interface id and interface channel, but have increased the speed from the default value of `250K` to `2M`, which can be adjusted through the crazyflie desktop client. 

###2. Modify Arming and Disarming

Since crazyflie does not support `armed` and `guided` modes, I have modified those parts of the script. Furthermore, the concept of state for crazyflie is different from the simulator or PX4. As a result, the `state_callback()` callback is never called! Since the state callback was responsible for takeoff transition, I have modified another one of the callbacks for that purpose. 

##### Arming / Takeoff

I have augmented the local position callback with code that issues takeoff command as and when required.

```python
def local_position_callback(self):
        if self.flight_state == States.MANUAL:
            self.takeoff_transition()
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
```

 On receiving the first local position message the takeoff command is issued. 

##### Disarming / Ending Mission

With no `armed` and `guided` information, we don't know when to consider the flight complete and the mission over. Thus, I have used the landing condition as the end of the flight. The `velocity_callback()` is modified for this purpose.

```python
def velocity_callback(self):
    if self.flight_state == States.LANDING:
        if abs(self.local_position[2] < 0.01):
            self.manual_transition()
```

Note that manual transition function is used as it contains the code to consider the flight as completed and stops the connection and the script.

### 3. Update Waypoints

For Crazyflie, I modified the coordinates of the box to take into account the position of the drone at takeoff. The `calculate_box()` function is modified as follows:

```python
def calculate_box(self):
    cp = self.local_position
    cp[2] = 0
    local_waypoints = [cp + [1.0, 0.0, 0.5], cp + [1.0, 1.0, 0.5], cp + [0.0, 1.0, 0.5], cp + [0.0, 0.0, 0.5]]
    return local_waypoints
```

Since I will be testing the crazyflie in an indoor environment, I changed the altitude to 0.5 m from 3 m.

### Time to Fly!

1. Turn on the crazyflie and place it on the ground.

2. Plug in the CrazyRadio in the computer.

3. Activate the python environment and run the script

   ```sh
   python backyard_flyer_crazyflie.py
   ```



## Acknowledgements

I would like to thank Udacity for their awesome Flying Car Nanodegree. If you are unaware about it, please click [here](https://in.udacity.com/course/flying-car-nanodegree--nd787). With regards to this project, Udacity has designed the simulator, Udacidrone API and given some skeleton code to work with. Also all the topics related to event driven programming and drone integration were taught in class. 
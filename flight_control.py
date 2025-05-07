import collections, collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
from config import settings

class FlightControl:
    def __init__(self, settings):
        self.software_test = settings["software_test"]

        # Connect to the vehicle (SITL in this case)
        print("Connecting to vehicle on /dev/ttyAMA0")
        if not self.software_test:
            self.vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=False)
            self.vehicle.wait_ready(True, timeout=50) # natural timeout is 30 secs and with lowered baud rate this is insufficient
            self.vehicle.groundspeed = 3.2

            self.print_settings()

    def print_settings(self):
        print("Global Location: %s" % self.vehicle.location.global_frame)
        print("Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print("Local Location: %s" % self.vehicle.location.local_frame)  # NED
        print("Attitude: %s" % self.vehicle.attitude)
        print("Velocity: %s" % self.vehicle.velocity)
        print("GPS: %s" % self.vehicle.gps_0)
        print("Groundspeed: %s" % self.vehicle.groundspeed)
        print("Airspeed: %s" % self.vehicle.airspeed)
        print("Gimbal status: %s" % self.vehicle.gimbal)
        print("Battery: %s" % self.vehicle.battery)
        print("EKF OK?: %s" % self.vehicle.ekf_ok)
        print("Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print("Rangefinder: %s" % self.vehicle.rangefinder)
        print("Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print("Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print("Heading: %s" % self.vehicle.heading)
        print("Is Armable?: %s" % self.vehicle.is_armable)
        print("System status: %s" % self.vehicle.system_status.state)
        print("Mode: %s" % self.vehicle.mode.name)  # settable
        print("Armed: %s" % self.vehicle.armed)  # settable

    # Function to arm and take off to a target altitude
    def arm_and_takeoff(self, target_altitude):
        if self.software_test:
            print(f"Software test: arming and taking off to {target_altitude} altitude")
            return True

        print("Arming motors...")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to become armable...")
            time.sleep(1)

        # Change mode to GUIDED and arm the drone
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        # Wait until the vehicle reaches the target altitude
        while True:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        return True # TODO: need to have fail statements

    def send_local_ned_velocity(self, vx,vy,vz):
        if not self.software_test:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                    0,
                    0, 0,
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    0b0000111111000111, # BITMASK -> Consider only the velocities
                    0, 0, 0,            # POSITION
                    vx, vy, vz,         # VELOCITY
                    0, 0, 0,            # ACCELERATION
                    0, 0)

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
        else:
            print(f"Executing local coordinates: {vx, vy, vz}")

        return True

    def send_global_ned_velocity(self, vx, vy, vz):
        if not self.software_test:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0,0,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0)

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            relative_loc = self.vehic.location.global_relative_frame
        else:
            print(f"Software test: Executing global coordinates: {vx, vy, vz}")
            relative_loc = 0


        return True, relative_loc # TODO: is there a way to get a success result?

    def land(self):
        if not self.software_test:
            pass
        else:
            print(f"Software test: Landing")
        return True


if __name__ == "__main__":
    fc = FlightControl(settings)

    # Arm and take off to 1 meter
    fc.arm_and_takeoff(1)

    counter = 0
    while counter <5:
            fc.send_global_ned_velocity(5,0,0)
            time.sleep(1)
            print("Moving True North relative to front of drone")
            counter = counter + 1

    time.sleep(2)

    counter=0
    while counter<5:
            fc.send_global_ned_velocity(0,-5,0)
            time.sleep(1)
            print("Moving True West relative to front of drone")
            counter = counter + 1

    time.sleep(2)

    counter=0
    while counter<5:
            fc.send_global_ned_velocity(-5,0,0)
            time.sleep(1)
            print("Moving True South relative to front of drone")
            counter = counter + 1

    time.sleep(2)

    counter=0
    while counter<5:
            fc.send_global_ned_velocity(0,5,0)
            time.sleep(1)
            print("Moving True East relative to front of drone")
            counter = counter + 1

    time.sleep(2)

    print("Mission Complete. Ready to Land")

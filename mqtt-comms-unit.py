import time

import paho.mqtt.client as mqtt
import json
from time import sleep
from threading import Thread
import logging
from pymavlink import mavutil

from pi_access_camera import AccessCamera
from flight_control import FlightControl
from config import settings
from system_control import SystemControl


host_ip = "localhost"
port = 1883
reconnect_delay = 2
max_reconnect_count = 10


class MQTTCommsUnit:
    def __init__(self, unit_name: str):
        self._is_busy = False
        self._current_command = ""
        self._current_command_key = ""
        self._latest_command_result = ""
        self._commands = dict()
        self._base_path = f"drone/{unit_name}"

        self._client = mqtt.Client(clean_session=True)
        self._client.on_message = self._on_command
        self._client.on_disconnect = self.on_disconnect

        self._client.connect(host=host_ip, port=port)
        self._client.subscribe(topic=f"{self._base_path}/command")

        self._client.loop_start()
        status_thread = Thread(target=self._repeat_status)
        status_thread.start()

    def register_command(self, command_name):
        self._commands[command_name.__name__] = command_name
        return command_name

    def _on_command(self, client, userdata, message):
        cmd = json.loads(message.payload)

        if "command" not in cmd:
            return  # Ignore commands without a command field

        if cmd["command"] == "get_status":
            self._publish_status()
        elif cmd["command"] == "get_command_result":
            self._publish_command_result()
        elif cmd["command"] in self._commands and not self._is_busy:
            try:
                print(f"Command received: {time.time()}")
                self._start_command(cmd)
            except Exception as ex:
                print(f"failed to start command with exception {ex}")

    def _publish_status(self):
        status_resp = dict()
        if self._is_busy:
            status_resp["status"] = "running_command"
            status_resp["command"] = self._current_command
            status_resp["command_key"] = self._current_command_key
        else:
            status_resp["status"] = "idle"
        self._client.publish(f"{self._base_path}/status", json.dumps(status_resp))

    def _publish_command_result(self):
        print(f"Command response published: {time.time()}")
        self._client.publish(f"{self._base_path}/command_result", self._latest_command_result)

    def _start_command(self, cmd):
        """

        :param cmd:
        :return:
        """
        self._is_busy = True
        self._current_command = cmd["command"]
        self._current_command_key = cmd["command_key"]

        self._publish_status()

        resources = cmd.get("resources")

        thread_args = (self._commands[self._current_command],)
        if resources is not None:
            thread_args += (resources,)

        cmd_thread = Thread(target=self._command_runner, args=thread_args)
        cmd_thread.start()

    def _command_runner(self, command_func, resources=None):
        if resources:
            result = command_func(resources)
        else:
            result = command_func()
        if result:
            if type(result) is bool:
                success = result
                result_data = ""
            else:
                success, result_data = result
        else:
            success = True
            result_data = ""

        self._latest_command_result = json.dumps(
            {
                "command": self._current_command,
                "command_key": self._current_command_key,
                "success": success,
                "result_data": result_data
            })

        self._is_busy = False
        self._publish_command_result()
        self._publish_status()

    def _repeat_status(self):
        # publish status every 1 second
        while True:
            self._publish_status()
            sleep(1)

    @staticmethod
    def on_disconnect(client, userdata, rc):
        logging.info(f"MQTT disconnected with result code: {rc}")

        for i in range(max_reconnect_count):
            sleep(reconnect_delay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error(f"{err}. Reconnect failed. Retrying...")

            logging.info(f"Reconnect failed after {i} attempts. Exiting")


# each unit (FC, camera, etc) uses the above class and registers its own relevant commands
flight_controller = MQTTCommsUnit("flight_controller")
flight_control = FlightControl(settings)

@flight_controller.register_command
def takeoff(target_altitude):
    success = flight_control.arm_and_takeoff(target_altitude)
    return success

@flight_controller.register_command
def send_velocity_coords(coords):
    x, y, z = coords
    success = flight_control.send_global_ned_velocity(x, y, z)
    return success

@flight_controller.register_command
def land():
    success = flight_control.land()
    return success


algorithm_parameters = {
    "resolution_scale": 0.5,
    "rotate_video": False,
    "pixel_skip": 10, # how many pixels the ImageSearch skips when running, previously called search_skip
    "pixel_min": 2, # minimum number of pixels for the stick to be classed as identified, previously called min_points
    "search_tol": 1, # what values in the image search are classed as identified (1 = must be 255, 0.5 means >= 127.5) (THIS IS REDUNDANT WITH STICK BECOMING BINARY)
    "bin_min": 20, # what values in the binary conversion are classed as True or False (0 means all becomes True) (This effectively replaces the search_tol)
    "band_num": 10, # how many bands to use when searching for stick
    "band_tol": 1.1, # how much larger the standard deviation needs to be between pixels in either band
    "target_tol": 5, # area around a target to be considered as valid
    "filter": "No Filter", # what filter to use (i think irrelevant)
    "prior_weight": 0.9, # how much to trust the current frame compared to the previous 5
    "tracker": "KCF", # which tracker to use - options are: BOOSTING, MIL, KCF, CSRT, TLD, MEDIANFLOW, GOTURN, MOSSE
    "uav_size_tolerance": 20, # how close UAV needs to be to register as above - should change with height (how many stick pixels are in frame?)
    "uav_camera_offset": (0,0) # (x, y) where the UAV centre is compared to the camera centre
}

vision = AccessCamera(settings, algorithm_parameters)
vision_system = MQTTCommsUnit("vision_system")

@vision_system.register_command
def start_video():
    success = vision.start_video()
    return success

@vision_system.register_command
def stop_video():
    success = vision.stop_loop()
    return success


sleep(5)
SystemControl(settings)

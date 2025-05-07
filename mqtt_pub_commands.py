import paho.mqtt.client as mqtt
import json
import uuid
from time import sleep, time
from threading import Thread
import logging

host_ip = "localhost"
port = 1883
reconnect_delay = 2
max_reconnect_count = 10
timeout_s = 30


class MQTTPublish:
    def __init__(self, unit_name: str):
        self._latest_status = ""
        self._latest_status_command_key = ""
        self._latest_command_response_key = ""
        self._latest_command_response_result = ""
        self._latest_command_result_data = ""
        self._base_path = f"drone/{unit_name}"

        self._client = mqtt.Client(clean_session=True)
        self._client.on_message = self._on_message
        self._client.on_disconnect = self.on_disconnect

        self._client.connect(host=host_ip, port=port)
        self._client.subscribe(topic=f"{self._base_path}/command_result")
        self._client.subscribe(topic=f"{self._base_path}/status")

        self._client.loop_start()

    def get_command_result(self):
        print("No command response received, requesting again")
        key = uuid.uuid1().clock_seq
        cmd = json.dumps(
            {
                "command": "get_command_result",
                "command_key": key,
                "resources": ""
            })

        self._client.publish(f"{self._base_path}/command", cmd)

    def publish_command(self, command, wait_for_response=True, resources=None):
        if self._latest_status == "running_command":
            print("nope!")
            return False, "Component already running a command"

        key = uuid.uuid1().clock_seq
        cmd = json.dumps(
            {
                "command": command,
                "command_key": key,
                "resources": resources
            })

        print(f"Command published: {time()}")
        self._client.publish(f"{self._base_path}/command", cmd)

        if not wait_for_response:
            return True

        time_s = 0
        command_accepted = False
        command_response_received = False
        while time_s < timeout_s:
            # check it changes to busy, with the correct key
            if not command_accepted:
                #print(self._latest_status_command_key)
                #print(key)
                command_accepted = self._latest_status_command_key == key
                #print(f"Command accepted: {command_accepted}")

            if command_accepted:
                command_response_received = self._latest_command_response_key == key
                #print(f"Command response received: {command_response_received}")

            if command_response_received:
                cmd_success = self._latest_command_response_result
                cmd_result_data = self._latest_command_result_data
                return cmd_success, cmd_result_data

            time_s =+ 1
            sleep(1)

            if (time_s-3 % 5) == 0:
                self.get_command_result()

        if not command_accepted:
            return False, "TIMEOUT - Component didn't accept command"
        if not command_response_received:
            return False, "TIMEOUT - Component didn't finish command"

    def _on_message(self, client, userdata, message):
        msg = json.loads(message.payload)

        if "status" in msg:
            self._latest_status = msg["status"]
            if msg["status"] != "idle":
                self._latest_status_command_key = msg["command_key"]

        elif "success" in msg:
            print(f"Command response received: {time()}")
            self._latest_command_response_key = msg["command_key"]
            self._latest_command_response_result = msg["success"]
            self._latest_command_result_data = msg["result_data"]

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

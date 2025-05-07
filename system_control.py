import time

from mqtt_pub_commands import MQTTPublish
from config import settings

class SystemControl:
    def __init__(self, settings):
        self.software_test = settings["software_test"]
        self.fc = MQTTPublish("flight_controller")
        self.vision = MQTTPublish("vision_system")

        self.main()

    def publish(self, system, cmd, response=False, resources=None):
        if resources:
            result = system.publish_command(cmd, wait_for_response=response, resources=resources)
        else:
            result = system.publish_command(cmd, wait_for_response=response)

        if type(result) is bool:
            success = result
            result_data = ""
        else:
            success, result_data = result

        return success, result_data

    def main(self):
        #Takeoff
        print("Instructing flight controller to takeoff")
        success, result_data = self.publish(self.fc, "takeoff", response=True, resources=2)
        if not success:
            print(result_data)
            raise Exception(f"Error: Takeoff not completed successfully")

        # Activate vision system
        print("Instructing vision system to start video")
        success, result_data = self.publish(self.vision, "start_video", response=True)
        if not success:
            print(result_data)
            raise Exception("Error: Camera not turned on successfully")

        #Wait for stick found
        time.sleep(10)

        #Vision processing
        # Tell FC to halt and hover(will it do this anyway?)

        #turn off vision system (reduce demand on Pi)
        print("Instructing vision system to stop video")
        success, result_data = self.publish(self.vision,"stop_video", response=True)
        if not success:
            print(result_data)
            raise Exception("Error: Camera not turned off successfully")

        #land
        print("Instructing flight controller to land")
        success, result_data = self.publish(self.fc, "land", response=True)
        if not success:
            print(result_data)
            raise Exception("Error: Drone has not landed successfully")

        time.sleep(0.1)


if __name__ == "__main__":
    SystemControl(settings)

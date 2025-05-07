import time

from lxml.objectify import annotate
#from picamera2 import Picamera2
import os
import cv2
import datetime
import numpy as np
from threading import Thread
from time import sleep

from image_searching import ImageSearching
#from jivan_tests.image_annotation import Annotation
#import jivan_tests.data_handler as dhand
from mqtt_pub_commands import MQTTPublish
from config import settings

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

annotation_parameters = {
    "fontScale": 2,
    "fontFace": cv2.FONT_HERSHEY_PLAIN,
    "fontColor": (0, 255, 0),
    "fontThickness": 2,
    "red": (255,0,0),
    "green": (0,255,0),
    "blue": (0,0,255),
    "cyan": (0,255,255),
    "yellow": (255,255,0),
    "magenta": (255,0,255),
    "orange": (255,165,0),
    "brown": (139,69,19),
    "pink": (255,192,203),
    "black": (0,0,0),
    "white": (255,255,255)
}

class AccessCamera:
    def __init__(self, program_params, algorithm_params):
        self.record_data = program_params["record_data"]
        self.record_video = program_params["record_video"]
        self.annotate = program_params["annotate_frame"]
        self.camera_present = program_params["camera_attached"]
        self.software_test = program_params["software_test"]

        self.frame_num = 0
        self.STOP_FLAG = False

        self.fc = MQTTPublish("flight_controller")

        if not self.software_test:
            self.picam2 = Picamera2()
            self.picam2.start()

            init_frame = self.picam2.capture_array()
            self.img_search = ImageSearching(init_frame, algorithm_params)

            self.win_name = "Camera Output"
            cv2.namedWindow(str(self.win_name), cv2.WINDOW_NORMAL)

        time = datetime.datetime.now()
        output_filename = "Test_" + str(time.strftime('%d%m%Y_%H%M%S'))
        
        if self.record_data:
            self.csv = dhand.CSVDataHandling(output_filename)

        if self.record_video:
            self.vid = dhand.MP4DataHandling(init_frame, output_filename)

        self.video_thread = Thread(target=self.video_loop)

    def start_video(self):
        self.video_thread.start()

        count = 0
        while not self.video_thread.is_alive() or count < 5:
            count += 1

        print(f"Video loop alive: {self.video_thread.is_alive()}")
        return self.video_thread.is_alive()

    def video_loop(self):
        while not self.STOP_FLAG and self.camera_present:
            frame = self.picam2.capture_array()
            if frame is None:
                break

            bin_data, target_data, motion_data = self.img_search.process_frame(frame)
            stick_bin, rail_bin = bin_data
            img_target, rail_target, stick_target = target_data
            x_error, y_error, angle_error = motion_data

            #frame = stick_bin

            if self.annotate:
                cv2.circle(frame, (int(self.img_search.height/2), int(self.img_search.width/2)), 20, (255,255,0)) # centre

                if stick_target is not None:
                    cv2.circle(frame, (int(stick_target[0]), int(stick_target[1])), 20, (0,255,255)) # stick centre

                if rail_target is not None and not np.isnan(rail_target[0]):
                    cv2.circle(frame, (int(rail_target[0]), int(rail_target[1])), 20, (255,0,0)) # rail centre

                #img_search.annotate_centre()
                #img_search.annotate_rail()
                #img_search.annotate_stick()
                #img_search.annotate_text()

                #cv2.putText(frame, "Frame: " + str(self.frame_num), (0,20), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2)
                #if None not in (x_error, y_error, angle_error) and "NaN" not in (x_error, y_error, angle_error):
                #    cv2.putText(frame, "X Error: " + str(int(x_error)), (0,50), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2)
                #    cv2.putText(frame, "Y Error: " + str(int(y_error)), (0,80), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2)
                #    cv2.putText(frame, "Angle Error: " + str(int(angle_error)), (0,110), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2)


            if self.record_data:
                data = {"Frame": self.frame_num,
                        "X Error": x_error,
                        "Y Error": y_error,
                        "Angle Error": angle_error}

                self.csv.add_row(data)

            if self.record_video:
                self.vid.record(frame)

            self.frame_num += 1

            cv2.imshow(str(self.win_name), frame)

        while not self.STOP_FLAG and self.software_test:
            time.sleep(5)
            print("Software test: Sending coordinates (1, 2, 3)")
            self.fc.publish_command("send_velocity_coords", resources=(1,2,3))

    def stop_loop(self):
        self.STOP_FLAG = True
        self.video_thread.join()

        if self.record_data:
            self.csv.export_csv()

        if self.record_video:
            self.vid.stop_recording()

        cv2.destroyAllWindows()

        if not self.software_test:
            self.picam2.close()

        print(f"Video loop alive: {self.video_thread.is_alive()}")

        return not self.video_thread.is_alive()


if __name__ == "__main__":
    vision = AccessCamera(settings, algorithm_parameters)
    vision.start_video()

    while vision.STOP_FLAG is False:
        # empty loop - video run in different thread
        sleep(30)
        key = cv2.waitKey(1)
        if key == ord("Q") or key == ord("q") or key == 27:
            vision.stop_loop()

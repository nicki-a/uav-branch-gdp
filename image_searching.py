import numpy as np
import cv2
#from pymavlink.mavutil import location


class ImageSearching:
    def __init__(self, example_frame, params):
        self.stick_prior = []
        self.count = 0

        # TODO: TRY FIX THIS
        self.rail_thresholds = (np.array([0,50,135]), np.array([179,255,255]))
        self.stick_thresholds = (np.array([15,50,0]), np.array([179,255,255]))

        self.resolution_scale = params["resolution_scale"]
        self.rotate_video = params["rotate_video"]
        self.pixel_skip = params["pixel_skip"]
        self.pixel_min = params["pixel_min"]
        self.search_tol = params["search_tol"]
        self.bin_min = params["bin_min"]
        self.band_num = params["band_num"]
        self.band_tol = params["band_tol"]
        self.target_tol = params["target_tol"]
        self.filter = params["filter"]
        self.tracker = params["tracker"]
        self.uav_tolerance = params["uav_size_tolerance"]
        self.camera_offset = params["uav_camera_offset"]

        self.img_centre = None
        self.img_target = None # (x, y)

        self.stick_prior_init = False
        self.stick_prior = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]] # previous 5 stick motion vectors

        self.stick_tracker_init = False

        self.uav_stick = False # is the UAV above the stick

        # Input frame
        self.width = None # x
        self.height = None # y

        self.preprocessing(example_frame)

    # get height and width from frame - rotate vid if necessary (proably shouldn't rotate live feed)
    def preprocessing(self, frame):
        """run once on initialisation, to find frame information"""
        frame = cv2.resize(frame, (0, 0), fx=self.resolution_scale, fy=self.resolution_scale)

        if self.rotate_video:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # TODO .shape is height, width!!!!!!!!!!!!!!!!!!!!
        self.height, self.width, _ = frame.shape
        # print(self.height, self.width) # troubleshooting flag
        self.img_centre = (self.width//2, self.height//2)

    def process_frame(self, frame):
        """main function in this class. Processes each frame in turn"""
        # extract rgb info
        resized_frame = cv2.resize(frame, (0, 0), fx=self.resolution_scale, fy=self.resolution_scale)
        if self.rotate_video:
            resized_frame = cv2.rotate(resized_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        img_rgb = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        # process image
        stick_bin, rail_bin = self.identify_features(img_rgb)

        # acquire image target
        self.img_target = (self.img_centre[0] - self.camera_offset[0], self.img_centre[1] - self.camera_offset[1]) # this could be done in the initialise stage, not sure if it gets edited later

        # acquire rail target - by searching image
        rail_target = self.get_rail_target(rail_bin)

        # acquire stick target - by searching image (in bands)
        stick_target = self.get_stick_target(stick_bin)

        # analysis output
        motion_vector, motion_mode = self.get_motion(rail_target, stick_target)

        return [stick_bin, rail_bin], [self.img_target, rail_target, stick_target], motion_vector

    # puts the frame through the filters to get binary frames
    def identify_features(self, img_rgb):
        rail = self.extract_channels(img_rgb, self.rail_thresholds)
        stick = self.extract_channels(img_rgb, self.stick_thresholds)

        rail_bin = cv2.threshold(rail, 10, 255, cv2.THRESH_BINARY)[1] # normally rturns retval too - this is to check if the operation was successful but probably not needed
        stick_bin_unfiltered = cv2.threshold(stick, 10, 255, cv2.THRESH_BINARY)[1]

        # stick requires further filtering
        # not sure how the blurs work but they do !!!!!!!!!!!!
        stick_bin_unfiltered = cv2.subtract(stick_bin_unfiltered, rail_bin)
        stick_bin_unfiltered = cv2.GaussianBlur(stick_bin_unfiltered, (3,3),0) # dimension of kernal, standard deviations in x and y
        stick_bin_unfiltered = cv2.medianBlur(stick_bin_unfiltered, 3) # dimension square area of kernel !!!!!! TAKES A LONG TIME TO RUN

        stick_bin = cv2.threshold(stick_bin_unfiltered, self.bin_min, 255, cv2.THRESH_BINARY)[1]

        return stick_bin, rail_bin
    
    # separates the target from the background using HSV colour channels
    def extract_channels(self, img_rgb, img_filter_thresh):
        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

        # uses the thresholds to create a filter to extract a feature
        img_filter = cv2.inRange(img_hsv, img_filter_thresh[0], img_filter_thresh[1])

        # uses the filter to extract the feature from the background
        img_mask = cv2.bitwise_and(img_rgb, img_rgb, mask=img_filter)

        return img_mask

    def search_image(self, img_bin):
        x_coords = []
        y_coords = []

        for x in range(0, self.width, self.pixel_skip): ###!!!! THIS SUM TAKES A LONG TIME TO RUN
            for y in range(0, self.height, self.pixel_skip):
                (h,s,v) = img_bin[y,x]
                if h >= self.search_tol*179 and s >= self.search_tol*255 and v >= self.search_tol*255:
                    x_coords.append(x)
                    y_coords.append(y)
        coords = [x_coords, y_coords]
        return coords

#region: bands and sticks - could probably make into a different class
    def search_image_bands(self, img, direction):
        regions = []
        
        if direction == "horizontal":
            bandwidth = int(self.height//self.band_num)
            for i in range(self.band_num):
                band_low = i*bandwidth
                band_high = min((i+1)*bandwidth, self.height)
                count = sum( 
                    (h >= self.search_tol*179 and s >= self.search_tol*255 and v >= self.search_tol*255)
                    for x in range(band_low, band_high, self.pixel_skip)
                    for y in range(0, self.width, self.pixel_skip)
                    for (h,s,v) in [img[x,y]]
                ) ###!!!! THIS SUM TAKES A LONG TIME TO RUN
                regions.append((int(band_low), int(band_high), int(count)))
        elif direction == "vertical":
            bandwidth = int(self.width//self.band_num)
            for i in range(self.band_num):
                band_low = i*bandwidth
                band_high = min((i+1)*bandwidth, self.width)
                count = sum(
                    (h >= self.search_tol*179 and s >= self.search_tol*255 and v >= self.search_tol*255)
                    for y in range(band_low, band_high, self.pixel_skip)
                    for x in range(0, self.height, self.pixel_skip)
                    for (h,s,v) in [img[x,y]]
                ) ###!!!! THIS SUM TAKES A LONG TIME TO RUN
                regions.append((int(band_low), int(band_high), int(count)))
        return regions

    def band_direction(self, h_band, v_band):
        h_count = [i[2] for i in h_band]
        v_count = [i[2] for i in v_band] 
        h_std = np.std(h_count, ddof=0)
        v_std = np.std(v_count, ddof=0)
        if h_std > self.band_tol*v_std:
            direction = "horizontal"
            bands = (h_band[0], v_band[1])
        elif v_std > self.band_tol*h_std:
            direction = "vertical"
            bands = (v_band[0], v_band[1])
        else:
            direction = None
            bands = None
        return direction, bands
    
    def filter_bands(self, bands):
        mean = sum(x[2] for x in bands) / len(bands)
        indices = [i for i, band in enumerate(bands) if band[2] >= mean]

        filtered_bands = []
        current_band = [bands[indices[0]]]

        for i in range(len(indices)-1):
            if indices[i]+1 == indices[i+1]:
                current_band.append(bands[indices[i+1]])
            else:
                low = current_band[0][0]
                high = current_band[-1][1]
                count = sum([x[2] for x in current_band])
                filtered_bands.append([low,high,count])
                current_band = [bands[indices[i + 1]]]
        if current_band:
            low = current_band[0][0]
            high = current_band[-1][1]
            count = sum([x[2] for x in current_band])
            filtered_bands.append([low,high,count])
        return np.array(filtered_bands), len(filtered_bands)
    
    def research_image_bands(self, img, band, direction):
        x_coords = []
        y_coords = []
        if direction == "horizontal":
            x_range = [0,self.width]
            y_range = [band[0], band[1]]
        elif direction == "vertical":
            x_range = [band[0], band[1]]
            y_range = [0,self.height]

        for x in range(x_range[0], x_range[1], self.pixel_skip):
            for y in range(y_range[0], y_range[1], self.pixel_skip):
                (h,s,v) = img[y,x]
                if h >= self.search_tol*179 and s >= self.search_tol*255 and v >= self.search_tol*255: # tolerance is in terms of HSV channels
                    x_coords.append(x)
                    y_coords.append(y)
        coords = [x_coords, y_coords]
        return coords
    
    def select_band(self, stick_coords):
        centres = []
        for i in range(len(stick_coords)):
            centres.append((np.mean(stick_coords[i][0]), np.mean(stick_coords[i][1])))
        distances = np.linalg.norm(np.array(centres) - np.array(self.img_target), axis=1)
        closest_idx = np.argmin(distances)
        closest_band = stick_coords[closest_idx]
        return closest_band
    
    def search_image_regions(self, coords, x_range, y_range):
        x_coords = []
        y_coords = []
        for x in range(x_range[0], x_range[1]-1, self.pixel_skip):
            for y in range(y_range[0], y_range[1]-1, self.pixel_skip):
                (h,s,v) = coords[y,x]
                if h >= self.search_tol*179 and s >= self.search_tol*255 and v >= self.search_tol*255: # tolerance is in terms of HSV channels
                    x_coords.append(x)
                    y_coords.append(y)
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)
        if np.isnan(x_mean) or np.isnan(y_mean):
            coord = None
        else:
            coord = [int(x_mean), int(y_mean)]
        
        return coord, x_coords, y_coords
    
    def get_angle(self, x_coords, y_coords):
        m, c = np.polyfit(x_coords, y_coords, 1) # y = m*x + c
        angle = 90 - np.rad2deg(np.arctan(m))
        if angle > 90:
           angle = angle - 180
        angle = 0
        return angle

    def find_stick_target(self, stick_bin, coords):
        pix_centre = (int(np.mean(coords[0])), int(np.mean(coords[1])))

        coords_T = np.array(coords).T
        distances = np.linalg.norm(coords_T - pix_centre, axis=1)
        closest_idx = np.argmin(distances)
        closest_point = (coords_T[closest_idx])
        closest_point = (int(closest_point[0]), int(closest_point[1]))

        x_range = (int(closest_point[0]-self.target_tol), int(closest_point[0]+self.target_tol))
        y_range = (int(closest_point[1]-self.target_tol), int(closest_point[1]+self.target_tol))

        stick_centre, x_coords, y_coords = self.search_image_regions(stick_bin, x_range, y_range)
        angle = self.get_angle(x_coords, y_coords)
        if stick_centre is not None:
            target = (stick_centre[0], stick_centre[1], angle)
        else:
            target = None

        return target
    
    def get_rail_target(self, rail_bin):
        rail_coords = self.search_image(rail_bin)
        if rail_coords is not None:
            rail_x = np.mean(rail_coords[0])
            rail_y = np.mean(rail_coords[1])
            if np.isnan(rail_x) or np.isnan(rail_y):
                rail_target = None
            else:
                rail_g = 0 # placeholder
                rail_target = (int(rail_x), int(rail_y), rail_g)
        else:
            rail_target = None
        return rail_target
    
    def get_rail_target2(self, rail_bin):
        regions_h = self.search_image_bands(rail_bin, "horizontal")
        regions_v = self.search_image_bands(rail_bin, "vertical")
        direction, bands = self.band_direction(regions_h, regions_v)

    def get_stick_target(self, stick_bin):
        regions_h = self.search_image_bands(stick_bin, "horizontal")
        regions_v = self.search_image_bands(stick_bin, "vertical")
        direction, bands = self.band_direction(regions_h, regions_v)

        if direction is not None:
            stick_coords = []
            bands, bands_num_new = self.filter_bands(bands)
            for i in range(bands_num_new):
                stick_coords.append(self.research_image_bands(stick_bin, bands[i], direction))
            if np.isnan(stick_coords[0][0]) is False:
                stick_coords = self.select_band(stick_coords)
                stick_target = self.find_stick_target(stick_bin, stick_coords)
            else:
                stick_target = None
        else:
            bands_num_new = 1
            stick_coords = self.search_image(stick_bin)
            if stick_coords is not None:
                stick_target = self.find_stick_target(stick_bin, stick_coords)
            else:
                stick_target = None
        return stick_target
#endregion    

    def get_motion(self, rail_target=None, stick_target=None):

        if rail_target is None and stick_target is None: # search mode
            motion_mode = "search"
            # arbitrary motion -> can be made a proper algorithm later
            dx = 0
            dy = 1
            dg = 0

        elif rail_target is not None and stick_target is None: # follow rail mode
            motion_mode = "follow rail"
            dx = (rail_target[0] - self.img_target[0]) # pos = right, neg = left -> middle of rail
            dy = (self.img_target[1] - rail_target[1]) # pos = up, neg = down
            dg = rail_target[2] # parallel to direction of rail

        elif rail_target is None and stick_target is not None: # find stick mode
            motion_mode = "go stick"
            dx = (stick_target[0] - self.img_target[0]) # pos = right, neg = left
            dy = (self.img_target[1] - stick_target[1]) # pos = up, neg = down
            dg = stick_target[2] + 90 # perpendicular to direction of stick (rotate CW)

        elif rail_target is not None and stick_target is not None: # home stick mode
            motion_mode = "rail and stick"
            dx = (rail_target[0] - self.img_target[0])  # align to middle of rail
            dy = (self.img_target[1] - stick_target[1]) # towards stick
            dg = stick_target[2] + 90 # perpendicular to stick
        
        if stick_target is not None:
            x_region = (self.img_target[0] + self.target_tol, self.img_target[0] - self.target_tol)
            y_region = (self.img_target[1] + self.target_tol, self.img_target[1] - self.target_tol)
            
            if x_region[1] <= stick_target[0] <= x_region[0] and y_region[1] <= stick_target[1] <= y_region[0]:
                dg = stick_target[2]
            elif x_region[1] >= stick_target[0] >= x_region[0] and y_region[1] >= stick_target[1] >= y_region[0]:
                dg = -stick_target[2]

        motion_vector = (dx, dy, dg) # eastings, northings, headings
        return motion_vector, motion_mode

    def analyse_image(self, stick_coords, rail_coords, searched_points):
        rail_centre = (np.mean(rail_coords[0]), np.mean(rail_coords[1]))

        if searched_points > self.pixel_min:
            stick_centre = (np.mean(stick_coords[0]), np.mean(stick_coords[1]))
            stick_pt1 = (stick_coords[0][0], stick_coords[1][0])
            stick_pt2 = (stick_coords[0][searched_points-1], stick_coords[1][searched_points-1])
        # TODO: Add an else clause - should the program create an exception and stop itself?
            
        return [stick_centre, stick_pt1, stick_pt2, rail_centre]
    



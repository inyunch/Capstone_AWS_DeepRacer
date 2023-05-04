import math


class Reward:

    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) - (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            start = 0
            end = int(end)
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.68306, -4.26176, 4.0, 0.04025],
                        [-0.54273, -4.26177, 4.0, 0.03508],
                        [-0.41273, -4.26177, 4.0, 0.0325],
                        [-0.24142, -4.26177, 4.0, 0.04283],
                        [0.05988, -4.26176, 4.0, 0.07533],
                        [0.36119, -4.26176, 4.0, 0.07533],
                        [0.66249, -4.26175, 4.0, 0.07533],
                        [0.9638, -4.26175, 4.0, 0.07533],
                        [1.2651, -4.26175, 4.0, 0.07533],
                        [1.56641, -4.26176, 4.0, 0.07533],
                        [1.86772, -4.26176, 4.0, 0.07533],
                        [2.16902, -4.26177, 4.0, 0.07533],
                        [2.47033, -4.26176, 4.0, 0.07533],
                        [2.77163, -4.26174, 4.0, 0.07533],
                        [3.07294, -4.26171, 4.0, 0.07533],
                        [3.37424, -4.26175, 4.0, 0.07533],
                        [3.67555, -4.26164, 4.0, 0.07533],
                        [3.97685, -4.26132, 4.0, 0.07532],
                        [4.27814, -4.26071, 4.0, 0.07532],
                        [4.57942, -4.25999, 4.0, 0.07532],
                        [4.88068, -4.2602, 4.0, 0.07532],
                        [5.18194, -4.26181, 4.0, 0.07531],
                        [5.48318, -4.26489, 4.0, 0.07531],
                        [5.78442, -4.26911, 4.0, 0.07532],
                        [6.08565, -4.27401, 3.71179, 0.08117],
                        [6.38691, -4.27857, 3.34737, 0.09001],
                        [6.68819, -4.28053, 2.97859, 0.10115],
                        [6.98946, -4.27751, 2.63534, 0.11432],
                        [7.29057, -4.26726, 2.19658, 0.13716],
                        [7.59122, -4.24757, 1.89919, 0.15864],
                        [7.89086, -4.2161, 1.75805, 0.17138],
                        [8.18863, -4.1702, 1.71397, 0.17578],
                        [8.48302, -4.10625, 1.71397, 0.17577],
                        [8.77152, -4.01959, 1.71397, 0.17575],
                        [9.04817, -3.90136, 1.71397, 0.17553],
                        [9.30291, -3.74398, 1.71397, 0.1747],
                        [9.52498, -3.54649, 1.71397, 0.17339],
                        [9.70668, -3.31363, 1.72661, 0.17106],
                        [9.84406, -3.0527, 1.77859, 0.16579],
                        [9.93624, -2.77158, 1.85318, 0.15965],
                        [9.98412, -2.4777, 1.93706, 0.15371],
                        [9.98965, -2.17798, 1.90395, 0.15745],
                        [9.95566, -1.87891, 1.90064, 0.15836],
                        [9.88562, -1.58703, 1.90064, 0.15793],
                        [9.7827, -1.30678, 1.90064, 0.15708],
                        [9.64831, -1.04033, 1.90064, 0.15702],
                        [9.47895, -0.79381, 1.90064, 0.15736],
                        [9.27549, -0.57543, 1.90064, 0.15704],
                        [9.04296, -0.39023, 1.99, 0.14938],
                        [8.78799, -0.23815, 2.11561, 0.14032],
                        [8.5166, -0.11619, 2.00002, 0.14877],
                        [8.23377, -0.01969, 1.97995, 0.15093],
                        [7.94335, 0.05638, 1.97995, 0.15163],
                        [7.64843, 0.11725, 1.97995, 0.15209],
                        [7.36179, 0.20515, 1.97995, 0.15143],
                        [7.08947, 0.32596, 1.97995, 0.15046],
                        [6.83685, 0.48119, 1.97995, 0.14976],
                        [6.6078, 0.66902, 2.0406, 0.14516],
                        [6.40408, 0.88509, 2.15259, 0.13796],
                        [6.22578, 1.12425, 2.31829, 0.12868],
                        [6.07153, 1.38116, 2.53559, 0.11818],
                        [5.93891, 1.65096, 2.82048, 0.10659],
                        [5.82471, 1.92959, 3.19615, 0.09422],
                        [5.72537, 2.21403, 3.72472, 0.08089],
                        [5.63719, 2.50214, 4.0, 0.07533],
                        [5.55818, 2.79289, 2.92522, 0.103],
                        [5.48686, 3.08562, 2.35506, 0.12793],
                        [5.42203, 3.37984, 2.0218, 0.14902],
                        [5.36246, 3.67518, 1.73312, 0.17384],
                        [5.30732, 3.9714, 1.47934, 0.20368],
                        [5.24369, 4.26531, 1.35832, 0.22139],
                        [5.16178, 4.55424, 1.3, 0.23101],
                        [5.05222, 4.83379, 1.3, 0.23096],
                        [4.90666, 5.09676, 1.3, 0.23121],
                        [4.71661, 5.32924, 1.3, 0.23098],
                        [4.47865, 5.50641, 1.3, 0.22822],
                        [4.20675, 5.60679, 1.3, 0.22295],
                        [3.92337, 5.62036, 1.30127, 0.21802],
                        [3.65079, 5.54991, 1.3505, 0.20847],
                        [3.40633, 5.40602, 1.44052, 0.19692],
                        [3.2013, 5.20315, 1.57747, 0.18285],
                        [3.04031, 4.95761, 1.78234, 0.16473],
                        [2.92087, 4.68513, 2.12497, 0.14001],
                        [2.83391, 4.39823, 2.89169, 0.10367],
                        [2.76539, 4.10536, 4.0, 0.07519],
                        [2.69403, 3.81263, 4.0, 0.07533],
                        [2.62452, 3.51945, 4.0, 0.07533],
                        [2.556, 3.22604, 4.0, 0.07533],
                        [2.48656, 2.93286, 4.0, 0.07532],
                        [2.41445, 2.64035, 4.0, 0.07532],
                        [2.33845, 2.34885, 4.0, 0.07531],
                        [2.25795, 2.05858, 3.07535, 0.09795],
                        [2.17287, 1.76963, 2.02987, 0.14839],
                        [2.08342, 1.48198, 1.732, 0.17392],
                        [1.9898, 1.19564, 1.60909, 0.18722],
                        [1.89222, 0.91059, 1.60909, 0.18724],
                        [1.79066, 0.62691, 1.60909, 0.18725],
                        [1.6733, 0.35007, 1.60909, 0.18687],
                        [1.5228, 0.09393, 1.60909, 0.18463],
                        [1.33356, -0.12735, 1.60909, 0.18095],
                        [1.10743, -0.30379, 1.61397, 0.17771],
                        [0.8516, -0.43207, 1.73342, 0.1651],
                        [0.57502, -0.51545, 1.96257, 0.14719],
                        [0.28554, -0.56129, 2.374, 0.12346],
                        [-0.01081, -0.58, 2.0819, 0.14263],
                        [-0.31029, -0.58199, 1.63042, 0.18368],
                        [-0.61154, -0.57616, 1.46122, 0.2062],
                        [-0.91265, -0.5681, 1.42468, 0.21143],
                        [-1.21317, -0.55051, 1.42468, 0.2113],
                        [-1.51215, -0.51492, 1.42468, 0.21133],
                        [-1.80283, -0.44288, 1.42468, 0.21021],
                        [-2.06696, -0.31753, 1.42468, 0.20521],
                        [-2.28637, -0.13798, 1.42468, 0.199],
                        [-2.45192, 0.08641, 1.47169, 0.18948],
                        [-2.56184, 0.34386, 1.59835, 0.17514],
                        [-2.61978, 0.62306, 1.8127, 0.1573],
                        [-2.63346, 0.91443, 2.16868, 0.1345],
                        [-2.61401, 1.21084, 2.74293, 0.1083],
                        [-2.57308, 1.5082, 3.624, 0.08283],
                        [-2.52182, 1.80511, 3.624, 0.08314],
                        [-2.47984, 2.10322, 3.624, 0.08307],
                        [-2.44903, 2.40231, 3.624, 0.08297],
                        [-2.43032, 2.70216, 3.624, 0.0829],
                        [-2.42394, 3.00251, 3.67699, 0.0817],
                        [-2.42955, 3.30306, 3.78316, 0.07946],
                        [-2.4465, 3.60349, 3.9261, 0.07664],
                        [-2.474, 3.90336, 3.33652, 0.09025],
                        [-2.51134, 4.20229, 2.92046, 0.10315],
                        [-2.55787, 4.49994, 2.64077, 0.11408],
                        [-2.613, 4.79613, 2.44107, 0.12342],
                        [-2.67629, 5.09069, 2.28045, 0.13212],
                        [-2.74885, 5.38304, 2.13535, 0.14106],
                        [-2.83535, 5.671, 2.00486, 0.14997],
                        [-2.93954, 5.95206, 1.89534, 0.15816],
                        [-3.06489, 6.22384, 1.82333, 0.16414],
                        [-3.21436, 6.48347, 1.77712, 0.16858],
                        [-3.39032, 6.72684, 1.77123, 0.16955],
                        [-3.59414, 6.94811, 1.77123, 0.16985],
                        [-3.8258, 7.14006, 1.77123, 0.16986],
                        [-4.0832, 7.2949, 1.77123, 0.16959],
                        [-4.36153, 7.40587, 1.77123, 0.16917],
                        [-4.65392, 7.4679, 1.77123, 0.16875],
                        [-4.95212, 7.47922, 1.79549, 0.1662],
                        [-5.24808, 7.441, 1.83501, 0.16262],
                        [-5.53463, 7.35617, 1.89697, 0.15754],
                        [-5.80597, 7.22935, 1.9823, 0.15109],
                        [-6.05799, 7.06628, 2.08262, 0.14414],
                        [-6.28823, 6.87284, 2.18899, 0.13738],
                        [-6.49548, 6.65446, 2.30326, 0.13071],
                        [-6.67951, 6.41601, 2.3886, 0.1261],
                        [-6.84011, 6.16127, 2.47028, 0.12191],
                        [-6.97739, 5.89361, 2.66096, 0.11305],
                        [-7.09376, 5.61659, 3.02908, 0.09919],
                        [-7.19368, 5.33308, 3.68004, 0.08169],
                        [-7.28239, 5.04529, 4.0, 0.07529],
                        [-7.36502, 4.7552, 4.0, 0.07541],
                        [-7.44632, 4.46461, 4.0, 0.07544],
                        [-7.52914, 4.17459, 4.0, 0.0754],
                        [-7.6136, 3.88517, 4.0, 0.07537],
                        [-7.69947, 3.59629, 4.0, 0.07534],
                        [-7.78627, 3.30775, 4.0, 0.07533],
                        [-7.87349, 3.01937, 4.0, 0.07532],
                        [-7.96087, 2.73105, 4.0, 0.07532],
                        [-8.04797, 2.44263, 4.0, 0.07532],
                        [-8.13491, 2.15415, 4.0, 0.07532],
                        [-8.22157, 1.86556, 4.0, 0.07533],
                        [-8.30825, 1.57699, 4.0, 0.07533],
                        [-8.39509, 1.28847, 4.0, 0.07533],
                        [-8.48195, 0.99995, 4.0, 0.07533],
                        [-8.56881, 0.71144, 4.0, 0.07533],
                        [-8.65567, 0.42292, 4.0, 0.07533],
                        [-8.74282, 0.13449, 4.0, 0.07533],
                        [-8.83011, -0.15389, 4.0, 0.07533],
                        [-8.91724, -0.44232, 4.0, 0.07533],
                        [-9.00411, -0.73083, 4.0, 0.07533],
                        [-9.09085, -1.01938, 4.0, 0.07533],
                        [-9.1778, -1.30787, 4.0, 0.07533],
                        [-9.26489, -1.59631, 4.0, 0.07533],
                        [-9.35226, -1.88467, 4.0, 0.07533],
                        [-9.43947, -2.17307, 3.73379, 0.0807],
                        [-9.52653, -2.46152, 3.42809, 0.08789],
                        [-9.61376, -2.74991, 3.18134, 0.09471],
                        [-9.69915, -3.03881, 2.99456, 0.1006],
                        [-9.77903, -3.32916, 2.83844, 0.10609],
                        [-9.85016, -3.6216, 2.68601, 0.11205],
                        [-9.90989, -3.91644, 2.51187, 0.11976],
                        [-9.956, -4.2137, 2.20026, 0.13672],
                        [-9.9862, -4.51319, 2.05111, 0.14675],
                        [-9.99831, -4.81412, 1.98299, 0.15188],
                        [-9.99021, -5.11523, 1.97629, 0.15242],
                        [-9.95958, -5.41489, 1.97629, 0.15242],
                        [-9.90341, -5.71076, 1.97629, 0.15238],
                        [-9.81484, -5.99788, 1.97629, 0.15204],
                        [-9.69091, -6.2701, 1.97629, 0.15134],
                        [-9.53191, -6.52172, 1.97629, 0.15061],
                        [-9.34049, -6.74883, 2.00645, 0.14803],
                        [-9.12031, -6.94899, 1.94253, 0.15318],
                        [-8.8752, -7.12039, 1.7377, 0.17212],
                        [-8.60999, -7.26168, 1.63951, 0.18328],
                        [-8.3296, -7.37135, 1.63951, 0.18364],
                        [-8.03863, -7.44905, 1.63951, 0.18369],
                        [-7.74051, -7.49112, 1.63951, 0.18364],
                        [-7.43985, -7.4902, 1.63951, 0.18339],
                        [-7.14563, -7.43617, 1.63951, 0.18245],
                        [-6.87046, -7.32599, 1.64844, 0.17981],
                        [-6.62384, -7.16576, 1.74696, 0.16835],
                        [-6.40882, -6.96572, 1.95847, 0.14995],
                        [-6.22243, -6.73692, 2.3051, 0.12802],
                        [-6.05864, -6.4888, 2.46139, 0.12079],
                        [-5.91069, -6.22863, 1.99729, 0.14985],
                        [-5.77183, -5.96124, 1.83538, 0.16416],
                        [-5.62798, -5.69652, 1.82458, 0.16512],
                        [-5.4753, -5.43683, 1.82458, 0.16511],
                        [-5.30992, -5.18502, 1.82458, 0.16511],
                        [-5.12337, -4.94969, 1.82458, 0.16459],
                        [-4.9095, -4.74394, 1.82458, 0.16265],
                        [-4.66992, -4.57685, 1.82458, 0.16009],
                        [-4.40963, -4.45045, 1.91451, 0.15114],
                        [-4.1338, -4.36206, 2.1044, 0.13764],
                        [-3.84692, -4.30625, 2.42437, 0.12055],
                        [-3.55267, -4.27605, 2.96302, 0.09983],
                        [-3.25401, -4.26366, 4.0, 0.07473],
                        [-2.95324, -4.26126, 4.0, 0.0752],
                        [-2.65188, -4.2617, 4.0, 0.07534],
                        [-2.35055, -4.26185, 4.0, 0.07533],
                        [-2.04925, -4.26185, 4.0, 0.07533],
                        [-1.74796, -4.26173, 4.0, 0.07532],
                        [-1.44665, -4.2617, 4.0, 0.07533],
                        [-1.14534, -4.26172, 4.0, 0.07533],
                        [-0.84404, -4.26175, 4.0, 0.07533]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                                steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        STANDARD_TIME = 13  # seconds (time that is easily done by model)
        FASTEST_TIME = 9  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
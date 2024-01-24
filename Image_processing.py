#! /usr/bin/env python2
import cv2
import sys
import numpy as np
from PIL import Image
import math
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# Declare a global varialbe for image storage
image = None
br = CvBridge()
scaled_array =[]
scaled_arrayb = Float32MultiArray()
scaled_arrayy = Float32MultiArray()
scaled_arrayr = Float32MultiArray()
merge = Float32MultiArray() 


# weighting factors: red has highest priority and then yellow and then blue
wr, wy, wb = 1 ,1.5, 2 
# wr, wy, wb = 1 ,1, 1 # weighting factors without priority only distance

c = 0
x3tag = None
y3tag = None

# callback function to store the incoming image
def ImageCb(msg):
    global image
    image = br.imgmsg_to_cv2(msg)
    # rospy.loginfo('Image received ...')

# drawing region of interest
def region_of_interest(image):
    pts = np.array([[135,14.5],[152,394],[577,378],[579,5.2]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(image, [pts], True, (0,165,255),3) # If true the last point is connected to the previous point else if false not connected
    cv2.circle(image, (357, 210), 5, (0,160,255), -1) # marking centre point 

# distance calculation for testing from a reference point to objects
def cal_dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

# distance calculation from robot to the objects
def dist_robot_obj(scaled_array, x3tag, y3tag):
    distances = []
    for point in scaled_array:
        if point is not None and x3tag is not None and y3tag is not None:
            point_x, point_y = point
            dist = math.sqrt((point_x - x3tag)**2 + (point_y - y3tag)**2)
            distances.append(dist)
    return distances

# publishing color to make the sorting conditions more conservative
def publish_color(color):
    pub_color.publish(color)
    print("color is: ", color)

# displaying distance b/w robot and object and the coordinates of robot & objects
def distance_display(weighted_distance, x, y, x3tag, y3tag, img_copy, centroid):
     for value in weighted_distance:
        value = "{:.2f}".format(value)
        red_text = str(value)
        x_str = "x = {:.2f}".format(x)
        y_str = "y = {:.2f}".format(y)
        point_str = "{}, {}".format(x_str, y_str)
        x_str_rob = "xr = {:.2f}".format(x3tag)
        y_str_rob = "yr = {:.2f}".format(y3tag)
        point_str_rob = "{}, {}".format(x_str_rob, y_str_rob)
        
        cv2.putText(img_copy, red_text, (int((x+1)*centroid[0]),int((1-y)*centroid[1])+30), cv2.FONT_HERSHEY_PLAIN, 1.3, (255,255,255),2)
        cv2.putText(img_copy, point_str, (int((x+1)*centroid[0]),int((1-y)*centroid[1])+50), cv2.FONT_HERSHEY_PLAIN, 1.3, (255,255,255),2)
        cv2.putText(img_copy, point_str_rob, (int((x+1)*centroid[0]),int((1-y)*centroid[1])+70), cv2.FONT_HERSHEY_PLAIN, 1.3, (255,255,255),2)

# publishing nearest coordinate of the object to the robot with all posssible conditions
def publish_nearest_points(scaled_array_r,scaled_array_y, scaled_array_b, unfilter_weighted_distance_r, unfilter_weighted_distance_y, unfilter_weighted_distance_b, img_copy, centroid, float_array ):
    filter_red_points = []
    weighted_distance_r = []
    for distance, point in zip(unfilter_weighted_distance_r, scaled_array_r):
        x, y = point
        if y >= -0.72:
            filter_red_points.append(point)
            weighted_distance_r.append(distance)  
    # print("Filter_red_points: ", filter_red_points)
    # print("Filter_red_points: ", weighted_distance_r)


    filter_blue_points = []
    weighted_distance_b = []
    for distance, point in zip(unfilter_weighted_distance_b, scaled_array_b):
        x, y = point
        if x >= -0.6:
            filter_blue_points.append(point)
            weighted_distance_b.append(distance)  
    # print("Filter_blue_points: ", filter_blue_points)
    # print("Filter_blue_points: ", weighted_distance_b)



    filter_yellow_points = []
    weighted_distance_y = []
    for distance, point in zip(unfilter_weighted_distance_y, scaled_array_y):
        x, y = point
        if x < 0.8:
            filter_yellow_points.append(point)
            weighted_distance_y.append(distance)  
    # print("Filter_yellow_points: ", filter_yellow_points)
    # print("Filter_yellow_points: ", weighted_distance_y)

    if len(weighted_distance_r) > 0:
        shortest_index_r = weighted_distance_r.index(min(weighted_distance_r))
        closest_point_r = filter_red_points[shortest_index_r]

    if len(weighted_distance_y) > 0:
        shortest_index_y = weighted_distance_y.index(min(weighted_distance_y))
        closest_point_y = filter_yellow_points[shortest_index_y]

    if len(weighted_distance_b) > 0:
        shortest_index_b = weighted_distance_b.index(min(weighted_distance_b))
        closest_point_b = filter_blue_points[shortest_index_b]


    if len(weighted_distance_r) > 0:
        # print("Inside red loop")
    

        if (len(weighted_distance_b) > 0 and len(weighted_distance_y) > 0):

            for i in range(len(weighted_distance_r)):
                for j in range(len(weighted_distance_y)):
                    for k in range(len(weighted_distance_b)):
                        # if ((weighted_distance_r[i] < weighted_distance_y[j] and weighted_distance_r[i] < weighted_distance_b[k])):
                            
                            if (weighted_distance_r[i] < weighted_distance_y[j] and weighted_distance_r[i] < weighted_distance_b[k]):
                                # print("weighted_distance_r < weighted_distance_y and weighted_distance_r < weighted_distance_b")
                                # print("scaled centre values: ", closest_point_r[0], closest_point_r[1])
                                publish_color("red")
                                x_pub.publish(closest_point_r[0])
                                y_pub.publish(closest_point_r[1])
                                distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)
                                # cv2.putText(img_copy, str(int(weighted_distance_r)), (closest_point_r[0], closest_point_r[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
                                # else:
                                #     x_pub.publish(0)
                                #     y_pub.publish(0)
                                ##publish
                            elif (weighted_distance_r[i] < weighted_distance_y[j] and weighted_distance_r[i] > weighted_distance_b[k]):
                                # print("weighted_distance_r > weighted_distance_b")
                                # print("scaled centre values: ", closest_point_b[0], closest_point_b[1])
                                publish_color("blue")
                                x_pub.publish(closest_point_b[0])
                                y_pub.publish(closest_point_b[1])
                                distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)
            
                            elif (weighted_distance_r[i] > weighted_distance_y[j] and weighted_distance_r[i] < weighted_distance_b[k]):
                                # print("weighted_distance_r > weighted_distance_y")
                                # print("scaled centre values: ", closest_point_y[0], closest_point_y[1])
                                publish_color("yellow")
                                x_pub.publish(closest_point_y[0])
                                y_pub.publish(closest_point_y[1])
                                distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                                distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)

                            elif (weighted_distance_r[i] > weighted_distance_y[j] and weighted_distance_r[i] > weighted_distance_b[k]):
                                if (weighted_distance_b[i] < weighted_distance_y[j]):
                                    # print("weighted_distance_b < weighted_distance_y")
                                    # print("scaled centre values: ", closest_point_b[0], closest_point_b[1])
                                    publish_color("blue")
                                    x_pub.publish(closest_point_b[0])
                                    y_pub.publish(closest_point_b[1])
                                    distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
                                    distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                                    distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)
                                
                                elif (weighted_distance_b[i] > weighted_distance_y[j]):
                                    # print("weighted_distance_b > weighted_distance_y")
                                    # print("scaled centre values: ", closest_point_y[0], closest_point_y[1])
                                    publish_color("yellow")
                                    x_pub.publish(closest_point_y[0])
                                    y_pub.publish(closest_point_y[1])
                                    distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
                                    distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                                    distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)


        elif (len(weighted_distance_y) > 0):
            for i in range(len(weighted_distance_r)):
                for j in range(len(weighted_distance_y)):

                    if (weighted_distance_r[i] > weighted_distance_y[j]):
                        # print("weighted_distance_r > weighted_distance_y")
                        # print("scaled centre values: ", closest_point_y[0], closest_point_y[1])
                        publish_color("yellow")
                        x_pub.publish(closest_point_y[0])
                        y_pub.publish(closest_point_y[1])
                        distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)
                        distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)


                    elif (weighted_distance_r[i] < weighted_distance_y[j]):
                        # print("weighted_distance_r < weighted_distance_y")
                        # print("scaled centre values: ", closest_point_r[0], closest_point_r[1])
                        publish_color("red")
                        x_pub.publish(closest_point_r[0])
                        y_pub.publish(closest_point_r[1])
                        distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)
                        distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)

        elif (len(weighted_distance_b) > 0):
            for i in range(len(weighted_distance_r)):
                for j in range(len(weighted_distance_b)):

                    if (weighted_distance_r[i] > weighted_distance_b[j]):
                        # print("weighted_distance_r > weighted_distance_b")
                        # print("scaled centre values: ", closest_point_b[0], closest_point_b[1])
                        publish_color("blue")
                        x_pub.publish(closest_point_b[0])
                        y_pub.publish(closest_point_b[1])
                        distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)                    
                        distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)

                    elif (weighted_distance_r[i] < weighted_distance_b[j]):
                        # print("weighted_distance_r < weighted_distance_b")
                        # print("scaled centre values: ", closest_point_r[0], closest_point_r[1])
                        publish_color("red")
                        x_pub.publish(closest_point_r[0])
                        y_pub.publish(closest_point_r[1])
                        pub_distance.publish(weighted_distance_r[0])
                        distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)                    
                        distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
        else:
            # print("only red object is there!!!")
            # print("scaled centre values: ", closest_point_r[0], closest_point_r[1])
            publish_color("red")
            x_pub.publish(closest_point_r[0])
            y_pub.publish(closest_point_r[1])
            pub_distance.publish(weighted_distance_r[0])
            distance_display(weighted_distance_r, closest_point_r[0], closest_point_r[1], x3tag, y3tag, img_copy, centroid)
            scaling_factors_robot(x3tag, y3tag, closest_point_r[0], closest_point_r[1])



    elif (len(weighted_distance_y) > 0 or len(weighted_distance_b) > 0):
        # print("Inside blue loop")
            
        if (len(weighted_distance_y) > 0 and len(weighted_distance_b) > 0):
            for i in range(len(weighted_distance_b)):
                for j in range(len(weighted_distance_y)):

                    if (weighted_distance_b[i] < weighted_distance_y[j]):
                        # print("weighted_distance_b < weighted_distance_y")
                        # print("scaled centre values: ", closest_point_b[0], closest_point_b[1])
                        publish_color("blue")
                        x_pub.publish(closest_point_b[0])
                        y_pub.publish(closest_point_b[1])
                        distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                        distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)

                    elif (weighted_distance_b[i] > weighted_distance_y[j]):
                        # print("weighted_distance_b > weighted_distance_y")
                        # print("scaled centre values: ", closest_point_y[0], closest_point_y[1])
                        publish_color("yellow")
                        x_pub.publish(closest_point_y[0])
                        y_pub.publish(closest_point_y[1])
                        distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)
                        distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)


        elif (len(weighted_distance_b) > 0):
            for i in range(len(weighted_distance_b)):
                # print("only blue object")
                # print("scaled centre values: ", closest_point_b[0], closest_point_b[1])
                publish_color("blue")
                x_pub.publish(closest_point_b[0])
                y_pub.publish(closest_point_b[1])
                pub_distance.publish(weighted_distance_b[0])
                distance_display(weighted_distance_b, closest_point_b[0], closest_point_b[1], x3tag, y3tag, img_copy, centroid)


        elif (len(weighted_distance_y) > 0):
            # print("only yellow object")
            # print("scaled centre values: ", closest_point_y[0], closest_point_y[1])
            publish_color("yellow")
            x_pub.publish(closest_point_y[0])
            y_pub.publish(closest_point_y[1])
            distance_display(weighted_distance_y, closest_point_y[0], closest_point_y[1], x3tag, y3tag, img_copy, centroid)


    else:
        x_pub.publish(0)
        y_pub.publish(0)
        publish_color("noobject")
        cv2.putText(img_copy, "No Object", (int(centroid[0])-20,int(centroid[1])+10), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255),2)
        if (float_array == []):
            print("kein Objekt scaled centre values: ")
            cv2.putText(img_copy, "No Object", (int(centroid[0])-20,int(centroid[1])+10), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255),2)
        pub_distance.publish(0.0)
        

# subscribe to the april tag "x" coordinate of robot
def tag_callback_x(data):
    # rospy.loginfo("Received value: %f", data.data)
    global x3tag
    x3tag = data.data
    # print("xtag: ", x3tag)

# subscribe to the april tag "y" coordinate of robot
def tag_callback_y(data):
    global y3tag
    y3tag = data.data
    # print("ytag: ", y3tag)

# Error between the robot desired position and the object location (For testing)
def scaling_factors_robot(x3tag, y3tag, point_x, point_y):
    x_translation = x3tag - point_x
    y_translation = y3tag - point_y
    # print("xtag: ", x3tag, "ytag: ", y3tag)
    print("Translation error: ", x_translation, y_translation)
    scaling_factor = 10
    scaled_x = x_translation / scaling_factor
    scaled_y = y_translation / scaling_factor

    x_offset = -0.07
    y_offset = -0.07

    final_x = point_x - scaled_x + x_offset
    final_y = point_y - scaled_y + y_offset
    # print("final scaled x and y of object: ", final_x, final_y)

# perform image processing
def Img_process(image):
    img_copy = np.copy(image)
    hsvFrame = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    region_of_interest(img_copy)

    # ref_pt = [(135,20)] # Reference point for calculating distance (for testing)
    # img_copy = cv2.circle(img_copy, ref_pt[0], 5, (0,165,255), -1)

    blue_lower = np.array([44, 86, 100])
    blue_upper = np.array([115, 213, 255])
    mask_b = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    kernel = np.ones((5,5), "uint8")
    mask_bb = cv2.dilate(mask_b, kernel)

    centroid = np.array([322.5, 239.0])
    
    _, contours_r, _ = cv2.findContours(mask_bb, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)

    object_centers_b = [] #For blue objects
    scaled_array_b = []
    weighted_distance_b = []

#######################################For Blue color tracking ######################################
#####################################################################################################
    if len(contours_r)!=0:
        for pic, contour in enumerate(contours_r):
            area = cv2.contourArea(contour)
            # print("Blue area = ", area)
            if(area >= 75):
                x1, y1, x2, y2 = cv2.boundingRect(contour) #coordinates of our object
                centre_b = [(int(x1 + (x2/2)), int(y1 + (y2)/2))]
                M = cv2.moments(contour)
                # print("size of M: ", len(M))
                if M["m00"] !=0:
                    center_x = float(M["m10"] / M["m00"])
                    center_y = float(M["m01"] / M["m00"])
                    object_centers_b.append((center_x, center_y))
            
                scale_centre_b = [(centre_b[0][0], centre_b[0][1])]
                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1+x2, y1+y2), (255,0,0), 2)
                cv2.putText(img_copy, "B", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2)
                for point in centre_b:
                    center = [(point[0], point[1])]
                    img_copy = cv2.circle(img_copy, center[0], radius=2, color=(0,0,255), thickness=3)
        
            if(area > 1 and area < 75):
                scale_centre_b = [(0.0 , 0.0)]

        
        for point in object_centers_b:
                scaled_x = (point[0] - centroid[0])/centroid[0]
                scaled_y = (centroid[1] - point[1])/centroid[1]
                scaled_array_b.append((scaled_x, scaled_y))

        distances_b = dist_robot_obj(scaled_array_b, x3tag, y3tag)
        weighted_distance_b = [wb * d for d in distances_b]
        flatten_array = [value for sublist in scaled_array_b for value in sublist]
        scaled_arrayb.data = flatten_array
        # rospy.loginfo("Blue array published: {}".format(scaled_arrayb))

#######################################For Yellow color tracking ######################################
#######################################################################################################

    yellow_lower = np.array([20, 30, 195])
    yellow_upper = np.array([30, 255, 255])
    mask_y = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    kernel = np.ones((5,5), "uint8")
    mask_yy = cv2.dilate(mask_y, kernel)

    _, contours_y, _ = cv2.findContours(mask_yy, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)

    object_centers_y = [] #For yellow objects
    scaled_array_y = []
    weighted_distance_y = []

    if len(contours_y)!=0:
        for pic, contour in enumerate(contours_y):
            area = cv2.contourArea(contour)
            # print("yellow area = ", area)
            if(area >= 175):
                x1, y1, x2, y2 = cv2.boundingRect(contour) #coordinates of our object
                centre_y = [(int(x1 + (x2/2)), int(y1 + (y2)/2))]
                M = cv2.moments(contour)
                if M["m00"] !=0:
                    center_x = float(M["m10"] / M["m00"])
                    center_y = float(M["m01"] / M["m00"])
                    object_centers_y.append((center_x, center_y))
            
                scale_centre_y = [(centre_y[0][0], centre_y[0][1])]
                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1+x2, y1+y2), (0,255,255), 2)
                cv2.putText(img_copy, "Y", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255),2)
                for point in centre_y:
                    center = [(point[0], point[1])]
                    img_copy = cv2.circle(img_copy, center[0], radius=2, color=(255,0,0), thickness=3)

            if(area > 1 and area < 175):
                scale_centre_y = [(0.0 , 0.0)]

        
        for point in object_centers_y:
                scaled_x = (point[0] - centroid[0])/centroid[0]
                scaled_y = (centroid[1] - point[1])/centroid[1]
                scaled_array_y.append((scaled_x, scaled_y))

        distances_y = dist_robot_obj(scaled_array_y, x3tag, y3tag)
        weighted_distance_y = [wy * d for d in distances_y]
        flatten_array = [value for sublist in object_centers_y for value in sublist]
        scaled_arrayy.data = flatten_array
        # rospy.loginfo("Yellow array published: {}".format(scaled_arrayy))

#######################################For Red color tracking ######################################
#######################################################################################################

    red_lower = np.array([135, 69, 0])
    red_upper = np.array([179, 255, 255])
    mask_r = cv2.inRange(hsvFrame, red_lower, red_upper)

    kernel = np.ones((5,5), "uint8")
    mask_rr = cv2.dilate(mask_r, kernel)

    _, contours_r, _ = cv2.findContours(mask_rr, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)

    object_centers_r = [] #For red objects
    scaled_array_r = []
    weighted_distance_r = []

    if len(contours_r)!=0:
        for pic, contour in enumerate(contours_r):
            area = cv2.contourArea(contour)
            # print("red area = ", area)
            if(area >= 160):
                x1, y1, x2, y2 = cv2.boundingRect(contour) #coordinates of our object
                centre_r = [(int(x1 + (x2/2)), int(y1 + (y2)/2))]
                M = cv2.moments(contour)
                # print("size of M: ", len(M))
                if M["m00"] !=0:
                    center_x = float(M["m10"] / M["m00"])
                    center_y = float(M["m01"] / M["m00"])
                    object_centers_r.append((center_x, center_y))
            
                scale_centre_r = [(centre_r[0][0], centre_r[0][1])]
                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1+x2, y1+y2), (0,0,255), 2)
                cv2.putText(img_copy, "R", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
                for point in centre_r:
                    center = [(point[0], point[1])]
                    img_copy = cv2.circle(img_copy, center[0], radius=2, color=(0,255,255), thickness=3)

            if(area > 1 and area < 160):
                scale_centre_r = [(0.0 , 0.0)]

        
        for point in object_centers_r:
                scaled_x = (point[0] - centroid[0])/centroid[0]
                scaled_y = (centroid[1] - point[1])/centroid[1]
                scaled_array_r.append((scaled_x, scaled_y))

        distances_r = dist_robot_obj(scaled_array_r, x3tag, y3tag)
        weighted_distance_r = [wr * d for d in distances_r]
        flatten_array = [value for sublist in object_centers_r for value in sublist]
        scaled_arrayy.data = flatten_array
        # rospy.loginfo("Red array published: {}".format(scaled_arrayr))

    scaled_array = [item for sublist in zip(scaled_array_b, scaled_array_y, scaled_array_r) for item in sublist]
    float_array =[val for point in scaled_array for val in point]
    merge.data = float_array
    scaled_array_pub.publish(merge)
    # print(float_array)
    publish_nearest_points(scaled_array_r,scaled_array_y, scaled_array_b, weighted_distance_r, weighted_distance_y, weighted_distance_b, img_copy, centroid, float_array)

#######################################################################################################
################################Visualization of Image processing######################################

    cv2.imshow("Stream: ", img_copy)
    cv2.waitKey(1)
    


# the main function
if __name__ == '__main__':
    # 1. Iniitalize a ROS node
    rospy.init_node("image_subscriber_node", anonymous=True)
    # 2. Create a subscriber that subscribes the camera image topic /camera_rect/image_rect
    rospy.Subscriber("/camera_rect/image_rect", Image, ImageCb)
    rospy.Subscriber("/tag_position", Float64, tag_callback_x, queue_size=10)
    rospy.Subscriber("/tag_position_y", Float64, tag_callback_y, queue_size=10)

    pub = rospy.Publisher('imagetimer', Image, queue_size=10)
    x_pub = rospy.Publisher('x_topic', Float32, queue_size=10)
    y_pub = rospy.Publisher('y_topic', Float32, queue_size=10)
    scaled_array_pub = rospy.Publisher('scaled_array_topic', Float32MultiArray, queue_size=10)
    pub_color = rospy.Publisher('color_topic', String, queue_size=10)
    pub_distance = rospy.Publisher('distance_topic', Float32, queue_size=10)

    
    
    # 3. Run a loop to access the stored image and apply processing 
    loop_rate = rospy.Rate(50) # 30, 1Hz
    while not rospy.is_shutdown():
        if image is not None:
            # insert image processing code here
            Img_process(image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("break")
                break
            

            pub.publish(br.cv2_to_imgmsg(image))
        loop_rate.sleep()


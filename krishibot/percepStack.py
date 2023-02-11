#! /usr/bin/env python3



####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    global pub_rgb
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    
     #, add global variable if any

    ############################### Add your code here #######################################

    ##########################################################################################
    pose = image_processing(image)
    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    global pub_depth
    depth_val = []
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    # print(image)
    img_float32 = np.float32(image)
    pose = image_processing(img_float32)
    imageFrame1 = cv2.cvtColor(img_float32, cv2.COLOR_RGB2BGR)
    transformed_depth_image = cv2.cvtColor(imageFrame1, cv2.COLOR_BGR2GRAY)
    transformed_depth_image = np.array(transformed_depth_image, dtype='uint8')/255

    for val in pose:
        val1 = transformed_depth_image[val]
        depth_val.append(round(val1, 2))

    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
    ############################### Add your code here #######################################

    ##########################################################################################
    pub_depth.publish(str(depth_val))


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose = []
    imageFrame = image
    # show_image(imageFrame)
	# Convert the imageFrame in
	# BGR(RGB color space) to
	# HSV(hue-saturation-value)
	# color space
    imageFrame1 = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR)
    hsvFrame = cv2.cvtColor(imageFrame1, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
    red_lower = np.array([140, 140, 75], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)


    redb_lower = np.array([0, 140, 70], np.uint8)
    redb_upper = np.array([10, 255, 255], np.uint8)
    redb_mask = cv2.inRange(hsvFrame, redb_lower, redb_upper)

        # Set range for yellow color and
        # define mask
    yellow_lower = np.array([11, 146, 78], np.uint8)
    yellow_upper = np.array([20, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

        
        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
        
        # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)

    redb_mask = cv2.dilate(redb_mask, kernal)
    res_redb = cv2.bitwise_and(imageFrame, imageFrame, mask = redb_mask)
                                
        
        # For yellow color
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame, mask = yellow_mask)

    font = cv2.FONT_HERSHEY_SIMPLEX							 
        
            

        # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            epsilon = 0.001*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)
            cv2.drawContours(imageFrame, [approx], 0, ((0, 255, 0)), 2)

            M = cv2.moments(contour)
            cx1 = int(round(float(M['m10']/M['m00']), 0))
            cy1 = int(round(float(M['m01']/M['m00']), 0))

            cv2.putText(imageFrame,'.', (cx1,cy1), font,3, (255, 255, 255),2)
            cv2.putText(imageFrame,'center', (cx1-50,cy1-20), font,1, (255, 255, 255),1)

            transformed_depth_image = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
            transformed_depth_image = np.array(transformed_depth_image, dtype='uint8')/255
            centroid1 = [cy1,cx1]
            
            pose.append(centroid1)
            print (centroid1)
            depth_val1 = transformed_depth_image[cy1,cx1]
            # print (transformed_depth_image.shape)
            # print(imageFrame.shape)
            print(round(depth_val1, 2))
            # print(depth_val1)
            break
    
            #cv2.putText(imageFrame,str(x) + ',' + str(y), (x,y),  1, (255, 255, 0), 2)
            #  print ( '[',cx1,',',cy1,']' )
            #cv2.putText(imageFrame, "Red Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
    contours, hierarchy = cv2.findContours(redb_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            epsilon = 0.001*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)
            cv2.drawContours(imageFrame, [approx], 0, ((0, 255, 0)), 2)

            M = cv2.moments(contour)
            cx2 = int(M['m10']/M['m00'])
            cy2 = int(M['m01']/M['m00'])

            cv2.putText(imageFrame,'.', (cx2,cy2), font,3, (255, 255, 255),2)
            cv2.putText(imageFrame,'center', (cx2-50,cy2-20), font,1, (255, 255, 255),1)

            transformed_depth_image = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
            transformed_depth_image = np.array(transformed_depth_image, dtype='uint8')/255
            centroid1 = [cy2,cx2]
            
            pose.append(centroid1)
            print (centroid1)
            depth_val1 = transformed_depth_image[cy2,cx2]
            # print (transformed_depth_image.shape)
            # print(imageFrame.shape)
            # print(round(depth_val1, 2))
            print(depth_val1)
            break

        # Creating contour to track yellow color
    contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            
            epsilon = 0.001*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)
            cv2.drawContours(imageFrame, [approx], 0, ((0, 255, 0)), 2)
            
            M = cv2.moments(contour)
            
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #print ( '[',cx1,',',cy1,']',',','[',cx,',',cy,']' )
            cv2.putText(imageFrame,'.', (cx,cy), font,3, (255, 255, 255),2)
            cv2.putText(imageFrame,'center', (cx-50,cy-20), font,1, (255, 255, 255),1)
            
            transformed_depth_image = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
            transformed_depth_image = np.array(transformed_depth_image, dtype='uint8')/255
            centroid = [cy,cx]
            pose.append(centroid)
            print (centroid)
            depth_val = transformed_depth_image[cy,cx]
            # print (transformed_depth_image.shape)
            # print(imageFrame.shape)
            # print(round(depth_val, 2))
            print(depth_val)
            break
    #		cv2.putText(imageFrame, "yellow Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
    
                
        # Program Termination
    show_image(imageFrame)
    # if cv2.waitKey(0) & 0xFF == ord('q'):
    #     cv2.release()
    #     cv2.destroyAllWindows()
        
    
    ############### Write Your code to find centroid of the bell peppers #####################

    ##########################################################################################
    return pose


def show_image(img):
      a = cv2.imshow("Image Window", img)
      cv2.waitKey(3)
      a.release()
      cv2.destroyAllWindows()

def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####

    rospy.init_node("percepStack", anonymous=True)
    
    sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    # cv2.namedWindow("Image Window", 1)
    # sub_image_depth_1 = rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)
    # cv2.namedWindow("Image Window", 1)
    # sub_image_color_2 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth_2 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
    # sub_image_color_3 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    # sub_image_depth_3 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)


    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
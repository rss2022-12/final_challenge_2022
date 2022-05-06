import cv2
import numpy as np
import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from visual_servoing.msg import ConeLocationPixel


#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################



def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    src=cv2.imread(img)
    img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    cv2.imshow("image", src)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def plot_color_scatter_RGB(img):
    src=cv2.imread(img)
    img = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
    r,g,b = cv2.split(img)
    fig=plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    axis.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Red")
    axis.set_ylabel("Green")
    axis.set_zlabel("Blue")
    plt.show()
    
def plot_color_scatter_HSV(img):
    src=cv2.imread(img)
    img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(img)
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    pixel_colors=colors.hsv_to_rgb(pixel_colors)
    axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Hue")
    axis.set_ylabel("Saturation")
    axis.set_zlabel("Value")
    plt.show()

def show_range(img):
    
    light_orange = (1, 190, 200)
    dark_orange = (18, 255, 255)
    src=cv2.imread(img)
    img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, light_orange, dark_orange)
    result = cv2.bitwise_and(img, img, mask=mask)
    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

        
def contouring(img):
     
    light_orange = (1, 180, 190)
    dark_orange = (23, 255, 255)
    src=cv2.imread(img)
    img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, light_orange, dark_orange)
    result = cv2.bitwise_and(img, img, mask=mask)
    RGBimg=cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(RGBimg, cv2.COLOR_RGB2GRAY)
    #cv2.imshow("image", gray)
       
    ret, threshold = cv2.threshold(gray,40, 255, 0)
    #cv2.imshow("thresh", threshold)
   
    kernel = np.ones((5,5),np.uint8)
    dilate = cv2.dilate(gray,kernel,iterations = 2)
    contours, hierarchy =  cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    
    c = max(contours, key = cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)
    # cv2.imshow("contour", img)
    #x,y,w,h = cv2.boundingRect(contours[6])
    print([x,y,w,h])   
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
    cv2.imshow("bounding",img)
    
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    


def cd_color_segmentation_callback(img):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
            img: np.3darray; the input image with a cone to be detected. BGR.
            template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
            bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                            (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    ########## YOUR CODE STARTS HERE ##########
      
     
    light_orange = (1, 180, 190)
    dark_orange = (23, 255, 255)
    #src=cv2.imread(img)
    img = cv2.cvtColor(img.data, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, light_orange, dark_orange)
    result = cv2.bitwise_and(img, img, mask=mask)
    RGBimg=cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(RGBimg, cv2.COLOR_RGB2GRAY)
    #cv2.imshow("image", gray)
       
    ret, threshold = cv2.threshold(gray,40, 255, 0)
    #cv2.imshow("thresh", threshold)
   
    kernel = np.ones((5,5),np.uint8)
    dilate = cv2.dilate(gray,kernel,iterations = 2)
    contours, hierarchy =  cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    
    c = max(contours, key = cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)

    # bounding_box = ((x, y), (x+w, y+h))
    pixel_location=(x+w/2.0,y+h/6)
    pub=rospy.Publisher(rospy.get_param("pixel_topic", "/relative_cone_px"),ConeLocationPixel, queue_size=1)

    pub.publish(pixel_location)
    ########### YOUR CODE ENDS HERE ###########
    
    # Return bounding box

# image_print("./test_images_cone/test9.jpg")
# plot_color_scatter_HSV("./test_images_cone/test4.jpg")
# show_range("./test_images_cone/test2.jpg")
# contouring("./test_images_cone/test1.jpg")
# print(cd_color_segmentation("./test_images_cone/test6.jpg"))
def listener():
        
    rospy.init_node("cone_detector",anonymous=True)
    rospy.Subscriber(rospy.get_param("image_topic","/zed/zed_node/rgb/image_rect_color"),Image, cd_color_segmentation_callback)
    rospy.spin()

 
if __name__ == '__main__':
    listener()
    


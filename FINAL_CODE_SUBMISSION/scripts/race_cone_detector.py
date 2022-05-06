#!/usr/bin/env python
import cv2
import csv
import sys
from cv_bridge import CvBridge, CvBridgeError
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
from std_msgs.msg import Int32MultiArray


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


# def image_print(img):
#     """
#     Helper function to print out images, for debugging. Pass them in as a list.
#     Press any key to continue.
#     """
#     src=cv2.imread(img)
#     img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
#     cv2.imshow("image", src)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# def plot_color_scatter_RGB(img):
#     src=cv2.imread(img)
#     img = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
#     r,g,b = cv2.split(img)
#     fig=plt.figure()
#     axis = fig.add_subplot(1, 1, 1, projection="3d")
#     pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
#     norm = colors.Normalize(vmin=-1.,vmax=1.)
#     norm.autoscale(pixel_colors)
#     pixel_colors = norm(pixel_colors).tolist()
#     axis.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker=".")
#     axis.set_xlabel("Red")
#     axis.set_ylabel("Green")
#     axis.set_zlabel("Blue")
#     plt.show()
    
# def plot_color_scatter_HSV(img):
#     src=cv2.imread(img)
#     img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
#     h, s, v = cv2.split(img)
#     fig = plt.figure()
#     axis = fig.add_subplot(1, 1, 1, projection="3d")
#     pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
#     norm = colors.Normalize(vmin=-1.,vmax=1.)
#     norm.autoscale(pixel_colors)
#     pixel_colors = norm(pixel_colors).tolist()
#     pixel_colors=colors.hsv_to_rgb(pixel_colors)
#     axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#     axis.set_xlabel("Hue")
#     axis.set_ylabel("Saturation")
#     axis.set_zlabel("Value")
#     plt.show()

# def show_range(img):
    
#     light_white = (0,0,0)
#     dark_white = (180,50,200)
#     src=cv2.imread(img)
#     img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(img, light_white, dark_white)
#     result = cv2.bitwise_and(img, img, mask=mask)
#     plt.subplot(1, 2, 1)
#     plt.imshow(mask, cmap="gray")
#     plt.subplot(1, 2, 2)
#     plt.imshow(result)
#     plt.show()

        
# def contouring(img):
#     light_white = (0,0,210)
#     dark_white = (100,120,255)
#     src=cv2.imread(img)
#     img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
#     print(type(img))
#     mask = cv2.inRange(img, light_white, dark_white)
#     result = cv2.bitwise_and(img, img, mask=mask)
#     RGBimg=cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
#     gray = cv2.cvtColor(RGBimg, cv2.COLOR_RGB2GRAY)
#     # cv2.imshow("image", gray)
       
#     ret, threshold = cv2.threshold(gray,40, 255, 0)
#     #cv2.imshow("thresh", threshold)
#     plt.imshow(threshold)
   
#     kernel = np.ones((5,5),np.uint8)
#     dilate = cv2.dilate(gray,kernel,iterations = 2)
#     contours, hierarchy =  cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:] #https://stackoverflow.com/questions/25504964/opencv-python-valueerror-too-many-values-to-unpack
#     cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    
#     c = max(contours, key = cv2.contourArea)
#     x,y,w,h = cv2.boundingRect(c)
#     # cv2.imshow("contour", img)
#     #x,y,w,h = cv2.boundingRect(contours[6])
#     print([x,y,w,h])   
#     cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
#     cv2.imshow("bounding",img)
#     # im_pil = Image.fromarray(img)
#     # im_pil.show()
    
    
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    


# def cd_color_segmentation(img,template):
#     """
#     Implement the cone detection using color segmentation algorithm
#     Input:
#             img: np.3darray; the input image with a cone to be detected. BGR.
#             template_file_path; Not required, but can optionally be used to automate setting hue filter values.
#     Return:
#             bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
#                             (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
#     """
#     ########## YOUR CODE STARTS HERE ##########
      
     
#     # light_orange = (0, 200, 200)
#     # dark_orange = (15, 255, 255)
    
#     light_white = (0,0,150)
#     dark_white = (100,120,255)
#     #src=cv2.imread(img)
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     # print(y)
#     mask = cv2.inRange(img, light_white, dark_white)
#     result = cv2.bitwise_and(img, img, mask=mask)
#     RGBimg=cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
#     gray = cv2.cvtColor(RGBimg, cv2.COLOR_RGB2GRAY)
#     #cv2.imshow("image", gray)
       
#     ret, threshold = cv2.threshold(gray,40, 255, 0)
#     #cv2.imshow("thresh", threshold)
   
#     kernel = np.ones((5,5),np.uint8)
#     dilate = cv2.dilate(gray,kernel,iterations = 2)
#     contours, hierarchy =  cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
#     cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    
#     c = max(contours, key = cv2.contourArea)
#     x,y,w,h = cv2.boundingRect(c)
#     # print([x,y,w,h])
#     bounding_box = ((x, y), (x+w, y+h))

#     ########### YOUR CODE ENDS HERE ###########

#     # Return bounding boxg
#     return bounding_box

def houghtransform(img):
    '''
    edges: Output of the edge detector.
lines: A vector to store the coordinates of the start and end of the line.
rho: The resolution parameter \rho in pixels.
theta: The resolution of the parameter \theta in radians.
threshold: The minimum number of intersecting points to detect a line.
'''
    img_height=img.height
    bridge=CvBridge()  
    img=bridge.imgmsg_to_cv2(img, img.encoding) # "bgr8")
    # np.savetxt('myfile1.csv', img,fmt='%s',join)
    # np.set_printoptions(threshold=sys.maxsize)
    #print(img)
    #afile= open("output.txt","w")
    #for line in (img):
#	    np.savetxt(afile,line)
   #   afile.close()
        

   
    

    img_height=np.shape(img)[0]
    img_width=np.shape(img)[1]
    rot_mat=cv2.getRotationMatrix2D((round(img_width/2),round(img_height/2)),180,1.0)
    img=cv2.warpAffine(img,rot_mat,(img_width,img_height))
    
    light_white = (0,0,210)
    dark_white = (100,120,255)
   
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # print(type(img))
    mask = cv2.inRange(img, light_white, dark_white)
    result = cv2.bitwise_and(img, img, mask=mask)
    RGBimg=cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(RGBimg, cv2.COLOR_RGB2GRAY)
    # cv2.imshow("image", gray)
       
    ret, threshold = cv2.threshold(gray,40, 255, 0)
    # kernel = np.ones((5,5),np.uint8)
    # dilate=cv2.erode(gray,kernel,iterations=2)
    gray[0:2*img_height//4,:]=0
    gray[4*img_height//5:img_height,:]=0
    edges = cv2.Canny(image=gray, threshold1=200, threshold2=300)
    # cv2.imshow("bounding",edges)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    img_height=np.shape(edges)[0]
   # edges=edges[img_height//2:img_height,:]

    #RGBimg=RGBimg[img_height//2:img_height,:]
 
    lines=cv2.HoughLinesP(edges,rho = 5,theta = 1*np.pi/180,threshold = 200,minLineLength = 100,maxLineGap = 50)
    lines=np.array(lines)
    
    try:
        N = lines.shape[0]
    except IndexError:
	print("no lines")
        return
    
    
    origin=round(np.shape(edges)[1]/2.0)
    #print(origin)
    
    min_negx=[origin*2.0,origin*2.0,origin*2.0,origin*2.0,origin*100.0,origin*2.0,0]
    min_posx=[origin*2.0,origin*2.0,origin*2.0,origin*2.0,origin*100.0,origin*2.0,0]
    for i in range(N):
        
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]    
        x2 = lines[i][0][2]
        y2 = lines[i][0][3] 
        
       # print(x1,y1,x2,y2)
   
        if x2-x1==0:
            continue
        
        slope=float((y2-y1))/float((x2-x1))
        b=y1-slope*x1
        #print(slope)
        if slope==0:
            # print(slope)
            continue
        # cv2.line(RGBimg,(x1,y1),(x2,y2),(0,255,0),2)
        x0=-b/slope
        #print(x0)
       
        if abs(slope)>0.10:
             
            if x0<origin:
                #print('hi')
                if abs(origin-x0)<min_negx[4]:
                    min_negx=(x1,y1,x2,y2,x0,slope,b)
                    
            if x0>origin:
                # print('hi')
                if abs(origin-x0)<min_posx[4]:
                    min_posx=(x1,y1,x2,y2,x0,slope,b)
                    
                    
        
    # x_avg=np.mean([min_negx[0],min_negx[2],min_posx[0],min_posx[2]])
    # y_avg=np.mean([min_negx[1],min_negx[3],min_posx[1],min_posx[3]])
    
    a1=min_negx[5]
    b1=-1
    c1=min_negx[6]
    a2=min_posx[5]
    b2=-1
    c2=min_posx[6]
    try:
        x_avg=float((b1*c2-b2*c1))/float((a1*b2-a2*b1))
        y_avg=float(a2*c1-a1*c2)/float(a1*b2-a2*b1)
        if y_avg>(3*img_height/5):
             print("low intersection")
	     return
             y_avg=img_height//2+50
             x_avg=img_width//2 - 3
        if x_avg>10:
            pub=rospy.Publisher(rospy.get_param("visual_servoing/pixel_topic", "/relative_cone_px"),ConeLocationPixel, queue_size=1)
            pixel_location=ConeLocationPixel()
            pixel_location.u=x_avg
            pixel_location.v=222 # y_avg
            pub.publish(pixel_location)
        else:
	    print("one line seen")
    except ZeroDivisionError:
        return
  
    
       
    # cv2.circle(gray,(int(round(x_avg)),int(round(y_avg))),10,(0,255,0),2)
    # cv2.circle(gray,(int(round(np.shape(edges)[1]/2.0)),10),10,(255,0,0),2)
    # cv2.line(gray,(int(round(min_negx[0])),int(round(min_negx[1]))),(int(round(min_negx[2])),int(round(min_negx[3]))),(0,255,0),2)
    # cv2.line(gray,(int(round(min_posx[0])),int(round(min_posx[1]))),(int(round(min_posx[2])),int(round(min_posx[3]))),(0,255,0),2)
 
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def listener():
        
    rospy.init_node("cone_detector",anonymous=True)
    rospy.Subscriber(rospy.get_param("visual_servoing/image_topic","/zed/zed_node/rgb/image_rect_color"),Image, houghtransform)
    rospy.spin()

 
if __name__ == '__main__':
    listener()
    

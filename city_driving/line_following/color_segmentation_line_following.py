import cv2
import numpy as np

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
	cv2.imshow("Bounding Box", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, blah=None):
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

	# y=268
 	# h=43
  
	height=len(img)
	width=len(img[0])
	cv2.rectangle(img, (0, 0), (width, height // 2), (0, 0, 0), -1)
	#print(np.mean(img))
	cv2.rectangle(img, (0, int(height * 0.8)), (width, height), (0, 0, 0), -1)
        
	hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# lower bound and upper bound for orange color
	lower_bound = np.array([0,140,170])	 #  try lowering V
	upper_bound = np.array([65,256,256])

	# find the colors within the boundaries
	mask = cv2.inRange(hsv, lower_bound, upper_bound)

	# Find contours from the mask
	contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = contours[0] if len(contours) == 2 else contours[1]
	x, y, w, h = None, None, None, None
	#make a bounding box around the cone	
	for entry in contours:
		x,y,w,h = cv2.boundingRect(entry)
		cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 5)

	try:
		bounding_box=((x, y), (x + w, y + h + 5))
	except:
		print("no bounding box!")
		bounding_box=((0,0),(0,0))
 
	########### YOUR CODE ENDS HERE ###########
 
	return bounding_box
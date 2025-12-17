"""
camera_fxns

all camera helper functions used in main.py
Note: gig-E camera used
"""
import cv2
import numpy as np
import time
import json
import mvsdk 

def calibrate_camera():
    """Calibrate the camera--only needs to be used once per program

    Returns:
        new_mtx, new_dist: camera params necessary for undistorting new images
        
        usage:  
        image = cv2.imread("photo.jpg")
        undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)
    """
    # Load camera parameters from JSON file
    with open('camera-params.json', 'r') as json_file:
        camera_params = json.load(json_file)
    new_mtx = np.array(camera_params['camera_matrix'])
    new_dist = np.array(camera_params['distortion_coefficients'])
    return new_mtx, new_dist
# LOAD CALIB FIRST
camera_matrix, dist_coeffs = calibrate_camera()

def take_photo():
    """takes and returns photo

    Returns:
        img: image taken
    """
    ret_frame=None # what we will be returning
    # Enumerate cameras
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    # Open the camera
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    # Set camera mode to continuous acquisition
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    # Manual exposure, exposure time = 80 ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera,50 * 1000)
    # Start the SDK’s internal image capture thread
    mvsdk.CameraPlay(hCamera)
    # Allocate it according to the camera’s maximum resolution
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    # Allocate the RGB buffer
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)
    # Capture one frame from the camera
    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
        ret_frame=frame.copy()        
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
    # Close the camera
    mvsdk.CameraUnInit(hCamera)
    # Free the frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)

    camera_matrix, dist_coeffs = calibrate_camera()
    undistorted = cv2.undistort(ret_frame, camera_matrix, dist_coeffs)
    rotated = cv2.rotate(undistorted, cv2.ROTATE_180)
    flipped = cv2.flip(rotated, 1)
    return flipped

def crop_img(img, x=0, y=374, w=1190, h=208):
    """
    crop_img

    crops image to just be conveyor belt
    
    :param img: original image
    :param x: start x
    :param y: start y
    :param w: width
    :param h: height
    """
    y_start, y_end = y, y+h
    x_start, x_end = x, x+w
    cropped_img = img[y_start:y_end, x_start:x_end]
    # show_img(cropped_img)
    return cropped_img


def preprocess(img):
    """
    preprocess

    crop, blur, hsv mask for both good (white) and bad (orange) items
    
    :param img: image to prep
    :returns ret_img=white mask, cropped=plain cropped for display, ret_bad=orange mask
    """
    cropped = crop_img(img)
    blur = cv2.GaussianBlur(cropped, (7,7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 100])     # low saturation, high value
    upper_white = np.array([179, 50, 255])
    ret_img = cv2.inRange(hsv, lower_white, upper_white)

    lower_orange = np.array([10, 120, 100])
    upper_orange = np.array([30, 255, 255])
    ret_bad = cv2.inRange(hsv, lower_orange, upper_orange)
    # ret = cv2.medianBlur(ret_bad,5)
    return ret_img, cropped, ret_bad


def find_items(img,orig_img,good_item):
    """
    find_items

    locates all items in the image, displaying circles on the orig_img (b/c img is binary)
    red outline=unreachable
    green outline=reachable
    yellow outline=not reachable yet, but will be on next cycle
    red center=bad (orange) item
    blue center=good (white) item

    :param img: binary preprocessed img
    :param orig_img: cropped image for displaying
    :param good_item: if masked img is for good or bad items
    :returns: list of image coordinates (centers)
    """
    coords=[]
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    for contour in contours:
        # print(cv2.contourArea(contour))
        area = cv2.contourArea(contour)
        if (area > 500) and(area<800) :  # For the outside of the die...
            # print(cv2.contourArea(contour))
            # cv2.drawContours(orig_img, contour, -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(contour)
            x_loc = x+(0.5*w)
            y_loc = y+(0.5*h)
            coords.append([x_loc,y_loc])
            # Draw circle centers based on reachability
            color=None
            center_color=None
            if(x_loc<=323):
                # not yet reachable but will be next
                color=(0,255,255) # yellow
            elif((x_loc>780) or (y_loc<=30) or (y_loc>=185)):
                # past reachability
                color=(0,0,255) # red
            else:
                # reachable
                color=(0,255,0) # green
            cv2.circle(orig_img, (int(x_loc),int(y_loc)), 15, color, 2) 
            if good_item:
                center_color=(255,0,0) # good items get blue center
            else:
                center_color=(0,0,255) # bad items get red center
            cv2.circle(orig_img, (int(x_loc),int(y_loc)), 1, center_color, 2)
    return coords

def start_img_window(window='default'):
    """
    starts the image window at the beginning of the program
        so the show_img() is not blocking
    
    :param window: name of window
    """
    cv2.startWindowThread()
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)  

def show_img(img, window='default', resize=True):
    """
    shows a blown-up version of the img
    
    :param img: image to show
    :param window: name of window
    :param resize: if it should be resized or not
    """
    if resize:
        cv2.resizeWindow(window, (1190*2), (208*2)) # scale up
    cv2.imshow(window, img) # Display the frame
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def calculate_homography():
    """
    calculates homography matrix from these calinration points

    returns the H matrix for converting to robot coords later
    """
    img_pts = np.array([
        [287,168],
        [775,166],
        [758,35],
        [331,49]
    ])
    robot_pts = np.array([
        [0,0], # ACCORDING TO FRAME 2....
        [577.422,7.586],
        [556.064,160.082],
        [52.424,140.5]
    ])
    H, _ = cv2.findHomography(img_pts, robot_pts)
    return H
    

def convert_pix_to_world(pix_x,pix_y,H, good=True):
    """
    convert_pix_to_world

    converts pixel value to robot value
    
    :param pix_x: pixel x val
    :param pix_y: pixel y val
    :param H: Homography matrix
    :param good: if good or bad item b/c offsets are a little different (orange mask was a little off bc of lighting)
    """
    if good:
        x_offset = -15
        y_offset = 8
    else: # offset for orange caps
        x_offset = -17
        y_offset = 7
    point = np.array([pix_x,pix_y,1.0])
    world_pt = H @ point
    world_pt /= world_pt[2] # normalize
    return world_pt[0]+x_offset, world_pt[1]+y_offset

def wait_for_items(img_coords, img_coords_bad):
    """
    wait_for_items

    iterates through image coordinates and returns True if any item is beyond the value threshold
    
    :param img_coords: good item coords
    :param img_coords_bad: bad item coords
    """
    value = 640
    # iterate thru coords, return True if in range
    for coord in img_coords:
        if (coord[0]>=value):# and (coord[0]<(value+10)): # ignore starting unreachables
            return True
    for coord in img_coords_bad:
        if (coord[0]>=value):# and (coord[0]<(value+10)):
            return True
    return False
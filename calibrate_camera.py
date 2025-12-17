import cv2 as cv
import numpy as np
import os
import json
import camera_fxns
"""
Write text on image to keep count of how many calibration images we have
"""
def write(img, text,
          org=(50, 50),
          font=cv.FONT_HERSHEY_SIMPLEX,
          fontScale=1, color=(255, 0, 0),
          thickness=2,
          line_type=cv.LINE_AA):

    newImg = cv.putText(img,
                        text,
                        org,
                        font,
                        fontScale,
                        color,
                        thickness,
                        line_type)

    return newImg


"""
Prepare for camera calibration (src: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
"""
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# webcam = cv.VideoCapture(0)
num_photos = 0
total_photos=10

# Get the current PATH environment variable
path = os.environ.get("PATH", "")

key = ord('r')

"""
Main calibration image capture loop with live webcam feed
"""
while key != ord('s') and num_photos<total_photos:
    # ret, still = webcam.read()
    still = camera_fxns.take_photo()
   
    # if not ret:
    #     print("Failed to capture image.")
    #     break
    
    # still = cv.cvtColor(still, cv.COLOR_BGR2GRAY)
    text = f"Taken {num_photos}/10"
    img=write(still,text)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(still, (7, 6), None)
    
    # If found, add object points, image points (after refining them)
    if ret:
        num_photos+=1
        objpoints.append(objp)
        gray=cv.cvtColor(still, cv.COLOR_BGR2GRAY)

        corners2 = cv.cornerSubPix(cv.cvtColor(still, cv.COLOR_BGR2GRAY), corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Define the filename and path
        save_path = os.path.join(path, f'image_{num_photos}.jpg')
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        # Save calibration photo without corners
        cv.imwrite(save_path, still)
        
        # # Draw and display the corners
        # cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        
        # Save the frame to the specified directory
        print(f"Image {num_photos} saved as {save_path}")
    
    # print(still)
    cv.imshow("Webcam", img)
    key = cv.waitKey(10)


"""
Camera calibration and Re-projection error
"""
# Calibrate Camera
print("Calibrating camera...")
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints,imgpoints, gray.shape[::-1], None, None)
if ret:
    # Save calibration parameters to a JSON file
    camera_params = {
        "camera_matrix": mtx.tolist(),
        "distortion_coefficients": dist.tolist(),
        "rotation_vectors": [rvec.tolist() for rvec in rvecs],
        "translation_vectors": [tvec.tolist() for tvec in tvecs]
    }
    with open('camera-params.json', 'w') as json_file:
        json.dump(camera_params, json_file, indent=3)

    print("Camera parameters saved to camera-params.json")

# Calculate re-projection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "Total Re-projection Error: {}".format(mean_error/len(objpoints)) )


"""
Undistort image
"""
# Load camera parameters from JSON file
with open('camera-params.json', 'r') as json_file:
    camera_params = json.load(json_file)

new_mtx = np.array(camera_params['camera_matrix'])
new_dist = np.array(camera_params['distortion_coefficients'])

# img = cv.imread('images/plant_distorted.jpg')
# ret, img = webcam.read()
img = camera_fxns.take_photo()
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(new_mtx, new_dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('undistorted_frame.jpg', dst)

# Define the filename and path
save_path = os.path.join(path, f'undistorted.jpg')

# Save calibration photo without corners
cv.imwrite(save_path, dst)


cv.imshow("Distorted", img)
cv.imshow("Undistorted", img)
key = cv.waitKey(0)



# webcam.release()
cv.destroyAllWindows()
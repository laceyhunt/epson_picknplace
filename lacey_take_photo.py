import cv2
import numpy as np
import mvsdk

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
    return undistorted
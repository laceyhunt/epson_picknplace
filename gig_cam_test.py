import camera_fxns
camera_fxns.start_img_window()
while True:
    img = camera_fxns.take_photo()
    crop = camera_fxns.crop_img(img)
    # camera_fxns.cv2.imshow('test', crop) # Display the frame
    camera_fxns.show_img(crop)
    # camera_fxns.cv2.waitKey(0)
    if camera_fxns.cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera_fxns.cv2.destroyAllWindows()
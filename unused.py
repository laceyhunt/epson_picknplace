import camera_fxns
camera_fxns.start_img_window()
camera_fxns.start_img_window('test')
camera_fxns.start_img_window('orig')
orig_img=camera_fxns.take_photo()
img, cropped, img_bad = camera_fxns.preprocess(orig_img)

camera_fxns.show_img(cropped,'orig')

img_coords = camera_fxns.find_items(img_bad, cropped,False)
num_items = len(img_coords)
camera_fxns.show_img(cropped)
camera_fxns.show_img(img_bad,'test')
print(f"{num_items} items found.")
# camera_fxns.show_img(cropped)
camera_fxns.cv2.waitKey(0)
camera_fxns.cv2.destroyAllWindows()
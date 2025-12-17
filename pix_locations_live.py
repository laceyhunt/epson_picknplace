import camera_fxns
from camera_fxns import cv2

# camera = cv2.VideoCapture(0) 
# ret, orig = camera.read()
# img = camera_fxns.crop_img(orig)
o_img = camera_fxns.take_photo()
img = camera_fxns.crop_img(o_img)
def show_pixel_values(img):
    # Mouse callback function
    def mouse_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pixel = img[y, x]   # (row = y, col = x)
            print(f"X: {x}, Y: {y}, Pixel: {pixel}")

    cv2.namedWindow("def", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("def", 1190, 208)
    cv2.setMouseCallback("def", mouse_event)

    while True:
        cv2.imshow("def", img)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            break

    cv2.destroyAllWindows()

# Example
# img = cv2.imread("your_image.jpg")
show_pixel_values(img)

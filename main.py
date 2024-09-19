import rospy
from dobot.srv import SetPTPCmd, SetPTPCmdRequest
from dobot.srv import SetEndEffectorSuctionCup, SetEndEffectorSuctionCupRequest
import cv2 as cv
import numpy as np
import tkinter as tk
from tkinter import Label, Entry, IntVar, Checkbutton, Button
import threading
import time

# Global variables
points = []
tracking = False
initial_gray = None
frame = None
homography_matrix = None
current_centroid = None
last_update_time = time.time()
scale_factor_y = 0.41085  # Scale factor camera for x-axis to robot
scale_factor_x = 0.4802  # Scale factor camera for y-axis to robot
auto_mode_active = False
reference_images = {}  # Dictionary to store reference images
category_labels = ['pale', 'surfaced', 'unknown']  # Labels for classification
auto_pickup = False  # Flag to enable/disable auto pickup
roi_corners = None  # Dynamic ROI corners based on selected homography or rectangle
middle_camera_x = 320
middle_camera_y = 240
middle_x_robot = 0  # Define the initial value for the robot's middle X-coordinate
middle_y_robot = 110  # Define the initial value for the robot's middle Y-coordinate
frame_frozen = False  # Flag to freeze the frame when executing pickup
orb_active = True  # Flag to control ORB detection

# Predefined ROI rectangle coordinates
roi_rectangle = np.array([
    [15, 20], 
    [479, 20], 
    [479, 465], 
    [15, 465]
], dtype=np.int32).reshape((-1, 1, 2))

# Camera matrix and distortion coefficients (using your provided values)
camera_matrix = np.array([
    [959.7808651839981, 0, 298.2425204757662],
    [0, 961.3210947068818, 271.3019300782519],
    [0, 0, 1]
])
dist_coeffs = np.array([0.1456080062940991, -0.3525662795219286, 0.008754005511518403, -0.001863066062528824, 0])

# Object points in the robot's coordinate system (assuming the object is on a flat surface)
object_points = np.array([
    [-0.5, -0.5, 0],  # Top-left corner
    [0.5, -0.5, 0],  # Top-right corner
    [0.5, 0.5, 0],  # Bottom-right corner
    [-0.5, 0.5, 0]  # Bottom-left corner
], dtype=np.float32)

# Load reference images for ORB feature matching
ref_img1 = cv.imread('/home/nada/Pictures/ref1.jpg', cv.IMREAD_GRAYSCALE)
ref_img2 = cv.imread('/home/nada/Pictures/ref2.jpg', cv.IMREAD_GRAYSCALE)

if ref_img1 is None or ref_img2 is None:
    print("Error: Could not load one or both reference images.")
    exit()

# Initialize ORB detector
orb = cv.ORB_create()

# Find the keypoints and descriptors with ORB in the reference images
kp1, des1 = orb.detectAndCompute(ref_img1, None)
kp2, des2 = orb.detectAndCompute(ref_img2, None)

# ROS initialization
rospy.init_node('dobot_control_node', anonymous=True)
set_ptp_cmd_service = rospy.ServiceProxy('/DobotServer/SetPTPCmd', SetPTPCmd)
set_suction_cup_service = rospy.ServiceProxy('/DobotServer/SetEndEffectorSuctionCup', SetEndEffectorSuctionCup)

# Basket coordinates
basket_1_coords = (150, 150, 100)  # Pale surface
basket_2_coords = (240, 0, 100)  # Textured surface
basket_3_coords = (0, -180, 100)  # Unknown

# Safe coordinates
safe_coords = (200, 0, 100)

def limit_param(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def SetPTPCmd(x, y, z, r=0, ptpMode=0):
    try:
        if x != 0 or y != 0:
            req = SetPTPCmdRequest()
            req.ptpMode = ptpMode
            req.x = x
            req.y = y
            req.z = z
            req.r = r
            set_ptp_cmd_service(req)
    except rospy.ServiceException as e:
        pass

def SetSuctionCup(enableCtrl, suck, isQueued=0):
    try:
        req = SetEndEffectorSuctionCupRequest()
        req.enableCtrl = enableCtrl
        req.suck = suck
        req.isQueued = isQueued
        set_suction_cup_service(req)
    except rospy.ServiceException as e:
        pass

def compute_homography():
    global homography_matrix
    if len(points) == 4:
        homography_matrix, _ = cv.findHomography(points, object_points[:, :2])
    else:
        homography_matrix = None

def calculate_centroid():
    if len(points) == 4:
        centroid_x = int(sum(point[0][0] for point in points) / 4)
        centroid_y = int(sum(point[0][1] for point in points) / 4)
        return centroid_x, centroid_y
    return (0, 0)

def calculate_delta(middle_x, middle_y, centroid_x, centroid_y):
    delta_x_pixels = centroid_x - middle_x
    delta_y_pixels = centroid_y - middle_y
    return delta_x_pixels, delta_y_pixels

def apply_homography_to_centroid(centroid_x, centroid_y):
    delta_x_pixels, delta_y_pixels = calculate_delta(middle_camera_x, middle_camera_y, centroid_x, centroid_y)
    delta_x_pixels = -delta_x_pixels
    delta_x_robot = delta_x_pixels * scale_factor_x
    delta_y_robot = delta_y_pixels * scale_factor_y
    x_robot = middle_x_robot - delta_y_robot
    y_robot = middle_y_robot + delta_x_robot
    x_robot = limit_param(x_robot, -50, 85)  # X-axis: min -50, max 85
    y_robot = limit_param(y_robot, 60, 249)  # Y-axis: min 60, max 249
    print(f"Centroid X, Y: ({centroid_x}, {centroid_y})")
    print(f"Delta X, Y (pixels): ({delta_x_pixels}, {delta_y_pixels})")
    print(f"Transformed Robot Coordinates: X={x_robot}, Y={y_robot}")
    return int(x_robot), int(y_robot)

def detect_rectangle_and_set_homography():
    global points, homography_matrix, roi_corners
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (4, 4), 0)
    edged = cv.Canny(blurred, 50, 150)
    contours, _ = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv.contourArea(contour) > 1000:  # Only consider contours with an area greater than 1000 pixels
            approx = cv.approxPolyDP(contour, 0.02 * cv.arcLength(contour, True), True)
            if len(approx) == 4:
                points = approx.reshape(-1, 2).astype(np.float32)
                compute_homography()
                roi_corners = points  # Set ROI corners to the detected rectangle
                update_labels()
                break

    if roi_corners is not None:
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv.fillPoly(mask, [roi_corners], 255)
        target_img_gray = cv.bitwise_and(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), mask)
        classify_surface(target_img_gray)

def classify_surface(target_img_gray):
    global classification_result, orb_active
    if not orb_active:
        print("ORB deactivated.")
        return

    # Detect ORB keypoints and descriptors in the target image
    kp_target, des_target = orb.detectAndCompute(target_img_gray, None)

    if des_target is not None and len(des_target) >= 2:
        bf = cv.BFMatcher(cv.NORM_HAMMING)

        # Find the top 2 matches for each descriptor
        matches1 = bf.knnMatch(des1, des_target, k=2)
        matches2 = bf.knnMatch(des2, des_target, k=2)

        # Apply the ratio test as per Lowe's paper
        good_matches1 = []
        good_matches2 = []
        ratio_thresh = 0.9

        for m, n in matches1:
            if m.distance < ratio_thresh * n.distance:
                good_matches1.append(m)

        for m, n in matches2:
            if m.distance < ratio_thresh * n.distance:
                good_matches2.append(m)

        num_matches1 = len(good_matches1)
        num_matches2 = len(good_matches2)

        if num_matches1 > num_matches2 and num_matches1 > 20:
            classification_result.set(f"Type 1 (Reference Image 1) - {num_matches1} matches")
            print(f"Classified as Type 1 with {num_matches1} good matches.")
        elif num_matches2 > 20:
            classification_result.set(f"Type 2 (Reference Image 2) - {num_matches2} matches")
            print(f"Classified as Type 2 with {num_matches2} good matches.")
        else:
            classification_result.set("Type 0 (Unknown) - Less than 20 matches")
            print("Classified as unknown.")
    else:
        classification_result.set("Type 0 (Unknown) - No detected features")
        print("Classified as unknown due to no detected features.")


def update_labels():
    global homography_matrix, current_centroid
    if len(points) == 4 and homography_matrix is not None:
        current_centroid = calculate_centroid()
        x_robot, y_robot = apply_homography_to_centroid(current_centroid[0], current_centroid[1])
        if x_robot is not None and y_robot is not None:
            label_coords.config(text=f"Robot Coordinates: X={x_robot}, Y={y_robot}")
            label_homography.config(text=f"Homography Matrix:\n{homography_matrix}")
            label_centroid.config(text=f"Centroid: X={current_centroid[0]}, Y={current_centroid[1]}")
            label_classification.config(text=f"Classification: {classification_result.get()}")
    else:
        reset_labels_to_na()

def reset_labels_to_na():
    label_coords.config(text="Robot Coordinates: N/A")
    label_homography.config(text="Homography Matrix: N/A")
    label_centroid.config(text="Centroid: N/A")
    label_classification.config(text="Classification: N/A")

def draw_polygon(frame, points):
    global roi_corners
    if points is not None and len(points) == 4:
        centroid_x, centroid_y = calculate_centroid()
        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
        cv.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        cv.circle(frame, (centroid_x, centroid_y), 5, (255, 0, 0), -1)
        roi_corners = np.array(points, dtype=np.int32)

def draw_rectangle_with_click(event, x, y, flags, param):
    global points, frame, tracking, initial_gray, frame_frozen, orb_active
    if event == cv.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append([x, y])
            cv.circle(frame, (x, y), 3, (0, 255, 0), -1)
        if len(points) == 4:
            tracking = True
            initial_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
            compute_homography()
            frame_frozen = False  # Ensure frame is unfrozen when new homography is set
            update_labels()
            orb_active = True  # Enable ORB detection

def update_points(initial_gray, gray, old_points):
    new_points, status, _ = cv.calcOpticalFlowPyrLK(initial_gray, gray, old_points, None)
    if new_points is not None and status is not None:
        good_new = new_points[status.flatten() == 1]
        good_old = old_points[status.flatten() == 1]
        return good_new.reshape(-1, 1, 2), good_old.reshape(-1, 1, 2)
    return old_points, old_points

def save_selected_region_as_ref():
    global frame, points, homography_matrix
    if homography_matrix is not None:
        width, height = 640, 480
        dst_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
        warped_image = cv.warpPerspective(frame, homography_matrix, (width, height))
        cv.imwrite('ref.png', warped_image)
        print("Selected region saved as ref.png")

def execute_pickup_and_place():
    if current_centroid is not None:
        x_robot, y_robot = apply_homography_to_centroid(current_centroid[0], current_centroid[1])
        if x_robot is not None and y_robot is not None:
            SetPTPCmd(0, 150, 50)
            rospy.sleep(2)
            SetPTPCmd(x_robot, y_robot, 10)
            rospy.sleep(2)
            SetPTPCmd(x_robot, y_robot, -22)
            rospy.sleep(2)
            SetSuctionCup(True, True)
            rospy.sleep(1)
            #SetPTPCmd(0, 110, 100)
            #rospy.sleep(2)
            SetPTPCmd(x_robot, y_robot, 20)
            rospy.sleep(1)
            SetPTPCmd(x_robot, y_robot, 50)
            rospy.sleep(1)
            SetPTPCmd(0, 160, 100)  # Safe coordinate to move the boxes without triggering the alarm
            rospy.sleep(1)
            SetPTPCmd(130, 160, 100)  # Safe coordinate to move the boxes without triggering the alarm
            rospy.sleep(1)

            '''if auto_identify.get():
                classify_surface()
                category = classification_result.get()
                if "Type 1" in category:
                    print(f"Classified as Type 1 with {matches1} matches. Moving to Basket 1.")
                    SetPTPCmd(*basket_1_coords)  # Move to basket 1 (Pale)
                elif "Type 2" in category:
                    print(f"Classified as Type 2 with {matches2} matches. Moving to Basket 2.")
                    SetPTPCmd(*basket_2_coords)  # Move to basket 2 (Textured)
                else:
                    print("Classified as Unknown. Moving to Basket 3.")
                    SetPTPCmd(*basket_3_coords)  # Move to basket 3 (Unknown)'''

            SetPTPCmd(safe_coords[0], safe_coords[1], 50)
            rospy.sleep(4)

            SetSuctionCup(True, False)
            rospy.sleep(2)

            SetPTPCmd(safe_coords[0], safe_coords[1], 100)
            rospy.sleep(2)

def update_center_point():
    global middle_x_robot, middle_y_robot
    try:
        middle_x_robot = float(entry_robot_middle_x.get())
        middle_y_robot = float(entry_robot_middle_y.get())
    except ValueError:
        pass  # Removed print statement to reduce terminal output

def opencv_loop():
    global points, frame, tracking, initial_gray, root, last_update_time, current_centroid, roi_corners, auto_mode_active, frame_frozen, orb_active

    cap = cv.VideoCapture(2)  # vidcapture
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_BRIGHTNESS, 75)
    #cap.set(cv.CAP_PROP_CONTRAST, 50)  # Setting contrast to 50%
    cv.namedWindow("Frame")
    cv.setMouseCallback("Frame", draw_rectangle_with_click)

    while True:
        if not frame_frozen:  # Only update the frame if it's not frozen
            ret, original_frame = cap.read()
            if not ret:
                break

            frame = original_frame.copy()

            height, width = frame.shape[:2]
            middle_frame_x = width // 2
            middle_frame_y = height // 2
            cv.circle(frame, (middle_frame_x, middle_frame_y), 5, (0, 0, 255), -1)

            # Draw the predefined ROI rectangle on the frame
            cv.polylines(frame, [roi_rectangle], isClosed=True, color=(255, 0, 0), thickness=2)

            if tracking and initial_gray is not None:
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                points, _ = update_points(initial_gray, gray, points)
                if len(points) == 4:
                    draw_polygon(frame, points)
                    compute_homography()
                    update_labels()

                initial_gray = gray.copy()

                if roi_corners is not None:
                    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                    cv.fillPoly(mask, [roi_corners], 255)
                    target_img_gray = cv.bitwise_and(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), mask)

                    classify_surface(target_img_gray)
                    update_labels()

        cv.imshow("Frame", frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            points = []
            tracking = False
            initial_gray = None
            cv.destroyWindow("Mask")  # Close the feature matching window
            orb_active = False  # Disable ORB detection when reset
        elif key == ord('s'):
            save_selected_region_as_ref()  # Save the selected region when "s" is pressed
        elif key == ord('d'):
            detect_rectangle_and_set_homography()  # Detect rectangle and set homography when "d" is pressed

    cap.release()
    cv.destroyAllWindows()
    root.quit()

def tkinter_loop():
    global root, entry_robot_middle_x, entry_robot_middle_y
    global label_coords, label_homography, label_centroid, label_delta, label_classification, auto_identify, classification_result, auto_mode_active

    root = tk.Tk()
    root.title("Homography and Coordinates Display")

    classification_result = tk.StringVar()
    auto_identify = tk.IntVar()

    label_coords = tk.Label(root, text="Robot Coordinates: N/A", font=("Open Sans", 12))
    label_coords.grid(row=0, column=0, padx=10, pady=5)

    label_homography = tk.Label(root, text="Homography Matrix: N/A", font=("Open Sans", 12))
    label_homography.grid(row=1, column=0, padx=10, pady=5)

    label_centroid = tk.Label(root, text="Centroid: N/A", font=("Open Sans", 12))
    label_centroid.grid(row=2, column=0, padx=10, pady=5)

    label_delta = tk.Label(root, text="Delta: N/A", font=("Open Sans", 12))
    label_delta.grid(row=3, column=0, padx=10, pady=5)

    label_classification = tk.Label(root, text="Classification: N/A", font=("Open Sans", 12))
    label_classification.grid(row=4, column=0, padx=10, pady=5)

    auto_identify_checkbox = tk.Checkbutton(root, text="Enable Automatic Identification", variable=auto_identify)
    auto_identify_checkbox.grid(row=5, column=0, padx=10, pady=5)

    button_execute = tk.Button(root, text="Execute Pickup and Place", command=execute_pickup_and_place)
    button_execute.grid(row=6, column=0, padx=10, pady=10)

    root.mainloop()

if __name__ == "__main__":
    opencv_thread = threading.Thread(target=opencv_loop)
    opencv_thread.start()
    tkinter_loop()
    opencv_thread.join()

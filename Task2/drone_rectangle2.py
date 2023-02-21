# import the necessary packages
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from realsense_depth import *
from utlities import *
from control_class import *

## IMAGE RESOLUTION
WIDTH = 1920
HEIGHT = 1080

## Vide Recording
# fourcc = 0x7634706d
# out = cv2.VideoWriter('out.mp4',fourcc,20,(WIDTH,HEIGHT))


#Estimating Drone Position using cv2.aruco.estimatePoseSingleMarkers

#Estimating Drone Position using cv2.aruco.estimatePoseSingleMarkers
def drone_pose(matrix_coefficients, distortion_coefficients,corners):
    ## MARKER SIZE OF DRONE

    ## Size of Marker on Drone
    MARKER_SIZE = 0.04
    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, matrix_coefficients,distortion_coefficients)
    return tvec,rvec

prevx,prevy = None,None
'''
PID TUNING BASED ON ERROR VALUES'''
def PID_tune(error):
    for i in range(3):
        if(error[i]<0.1):
            kp[i] = kp_max[i]*0.1
            kd[i] =kd_max[i]*2
        elif(error[i]>0.1 and error[i]<0.2):
            kd[i] = kd_max[i]*1.3
            kp[i] = kp_max[i]*0.6
        elif(error[i]>0.2 and error[i]<0.3):
            kd[i] = kd_max[i]*1.1
            kp[i] = kp_max[i]*0.8
        elif(error[i]>0.3):
            kd[i] = kd_max[i]
            kp[i] = kp_max[i]   



'''
Get Makers : Takes Image 
Reduces Search Space to 300 X 300 based on previous position of drone
Returns corners, id and rejected'''
def get_markers(img):
    global prevx, prevy
    if prevx is None:
        subimg = img
    else:
        xmin = max(prevx - 150, 0)
        ymin = max(prevy - 150, 0)
        xmax = min(prevy + 150, 1920)
        ymax = min(prevy + 150, 1080)
        # Generating Sub Image
        subimg = img[ymin:ymax,xmin:xmax]
    ## Aruco Detection on reduced Search Space
    corners, ids, rejected = cv2.aruco.detectMarkers(subimg, arucoDict, parameters=arucoParams)
    if prevx is not None:
        if corners:
            corners += [prevx, prevy]
            prevx, prevy = (corners[0] + corners[2]) // 2
        else:
            prevx, prevy = None, None
    return corners, ids, rejected

def drone_navigation(desired_pos, epsilon):
    prev_time = 0
    error =0
    derr =0
    error_sum = 0
    prev_time = time.time()
    f_old = prev_time
    num_not_detected = 0
    try:
        while True:
            ## Reading Colour frame from depth Camera
            ret, color_frame = dc.get_frame()
            gray = cv2.cvtColor(color_frame,cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = get_markers(gray)
            # Uncomment to work with full image rather than cropped
            # (corners, ids, rejected) = cv2.aruco.detectMarkers(gray,arucoDict, parameters=arucoParams)

            if len(corners) == 0 or 0 not in ids:
                # Drone not detected
                print('[INFO] : Drone not detected ...')
                # If drone is not detected for more than 20 frames => drone out of camera's field of view. Hence Exiting
                if num_not_detected > 20:
                    print('[INFO] : Drone out of range...')
                    print('[INFO] : Landing ...')
                    command.land()
                    print('[INFO] : Quitting ...')
                    break;

                num_not_detected +=1
                # In case the drone is not detected, hover at same postion and skip rest of the loop and re read the frame
                command.set_attitude(roll=mean_vals[0],pitch=mean_vals[1],throttle=mean_vals[2],yaw=mean_vals[3])

                # Resizing the image for viewing
                scaled = cv2.resize(color_frame, (1280, 720), interpolation = cv2.INTER_CUBIC)
                cv2.imshow("Frame", scaled)
                # Uncomment below line to record video
                #out.write(color_frame)

                ## Printing FPS
                f_new = time.time()
                fps = 1/(f_new-f_old)
                f_old = f_new
                print('[INFO] : fps = ',fps)
                continue
            else:
                # drone detected
                num_not_detected = 0

            # finding correspinding index
            ids = ids.tolist()
            i = ids.index([0])

            ## Checking for Large Makers and ignoring
            if ( abs(corners[i][0][1][0] - corners[i][0][0][0]) > 100 or abs(corners[i][0][1][1] - corners[i][0][0][1]) > 100 ):
                print(' [ERROR] : Incorrect Marker detected ...')
                continue

            ## Displaying Aruco Marker
            color_frame,xc,yc = aruco_display(color_frame,corners[i])
            ## Drone Pose
            tvec,rvec = drone_pose(k,d,corners[i])
            pos = tvec[0][0]
            x = pos[0]
            y = pos[1]
            z = pos[2]

            # Calculation of angle of the drone using corner points
            point1 = (corners[i][0][1] + corners[i][0][2])//2
            point2 = (corners[i][0][0] + corners[i][0][3])//2
            if point1[0] == point2[0]:
                angle = np.pi/2
            else: 
                angle =  np.arctan((point1[1] - point2[1])/(point1[0] - point2[0]))



            ## Obtaining Depth Info
            depth = dc.get_depth(xc,yc)
            ## Uncomment to use Pose Estimation based depth
            #depth = z

            ## Updating the current position
            curr_pos = np.array([x,y,depth,angle])

            ## Printing position at the top right
            position = 'x :'+str(round(x*100))+'  y:'+str(round(y*100))+'  z :'+str(round(depth*100)) +'  angle :'+str(round(angle*180/np.pi))
            cv2.putText(color_frame, str(position),(100,100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

            # Checking if desired position is reached
            err = desired_pos - curr_pos
            err[2] /= 10
            # err[2] /= 4
            # if(np.linalg.norm((desired_pos - curr_pos)[:2]) < epsilon):
            if(np.linalg.norm(err) < epsilon):
                print('[INFO] : Reached Position')
                for i in range(10):
                    
                    command.set_attitude(roll=mean_vals[0],pitch=mean_vals[1],throttle=mean_vals[2],yaw=mean_vals[3])
                    time.sleep(0.1)
                return True
                
            print('desired pos : ',desired_pos)
            print('curr_pos :', curr_pos )

            curr_time = time.time()
            time_ = curr_time-prev_time

            # Computing the Error and Derr
            error = desired_pos  -curr_pos
            derr = (error -prev_err)/(time_)

            
            ##  Chaning PID based on error
            PID_tune(error)
            
            # Compting P I D
            P = kp*error
            I = ki*error_sum
            D =  kd*derr

            ## Uncomment to PID Values
            #print(P,I,D)

            result = P+I+D

            # Uncomment to result effective correction3
            #print(result)
            
            ## Computing  x and y correction based on result and angle of the drone
            # temp0 = result[0]
            # temp1 = result[1]
            # result[0] =  (int)(np.cos(angle)*temp0 + np.sin(angle)*temp1)
            # result[1] =  (int)(np.cos(angle)*temp1 - np.sin(angle)*temp0)

            # Updating result accordingly
            result = mean_vals - (np.rint(result)).astype(int)
            
            ## Uncomment to view RPTY
            # print("pitch : ",result[0])
            # print("roll : ",result[1])
            # print("throttle : ",result[2])
            # print("yaw : ",result[3])
            

            #Sending Command to Drone
            command.set_attitude(throttle = result[2], yaw = result[3],pitch = result[0],roll = result[1])
            print(result)

            # Computing Error Sum
            error_sum += error*(time_)
            prev_time = curr_time

            ## Resizing the image for viewing
            scaled = cv2.resize(color_frame, (1280, 720), interpolation = cv2.INTER_CUBIC)
            cv2.imshow("Frame", scaled)

            # Uncomment to record video
            #out.write(color_frame)

            
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            ## Updating the current time for FPS Calculation
            f_new = time.time()
            fps = 1/(f_new-f_old)
            f_old = f_new
            print('[INFO] : fps = ',fps)

    except KeyboardInterrupt:
        print('[INFO] : KeyboardInterrupt')
        return False


# defining an empty custom dictionary 
arucoDict = cv2.aruco.custom_dictionary(0, 4, 1)
# adding empty bytesList array to fill with 2 markers  
arucoDict.bytesList = np.empty(shape = (2, 2, 4), dtype = np.uint8)

# adding new markers
mybits = np.array([[0,1,0,0],[1,1,0,0],[1,0,1,0],[1,1,0,1]], dtype = np.uint8)
arucoDict.bytesList[0] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,1,1,1],[1,0,0,1],[1,0,0,1],[0,0,0,1],], dtype = np.uint8)
arucoDict.bytesList[1] = cv2.aruco.Dictionary_getByteListFromBits(mybits)

# Aruco Parameters
arucoParams = aruco.DetectorParameters_create()

# Loading Camera and Distortion Matrix
calibration_matrix_path = "calibration_matrix.npy"
distortion_coefficients_path = "distortion_coefficients.npy"
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

## Adjusting values for 1080p
if WIDTH == 1920 and HEIGHT == 1080:
    k = k*1.5
    k[2][2] = 1
# initialize the video stream and allow the camera sensor to warm up


first_time = True
num_not_detected = 0
tolerance = 0.1
desired_depth = 1.6

# Initialisation
error = np.zeros(4)
derr = np.zeros(4)
error_sum = np.zeros(4)
prev_err = np.zeros(4)

## PITCH,ROLL,THROTTLE,YAW
### PID PARAMETERS
kp_max = np.array([10,10,40,0])
# ki_max = np.array([0, 0, 0, 0])
# kd_max = np.array([0, 0, 0, 0])
ki_max = np.array([2,2,1,0])
kd_max = np.array([5,5,20,0])

kp = np.copy(ki_max)
ki = np.copy(ki_max)
kd = np.copy(kd_max)

# ---------------- BUILDING THE WAYPOINTS ----------------

corners = np.array([
    [ 0.75, -0.375],
    [-0.75, -0.375],
    [-0.75,  0.375],
    [ 0.75,  0.375]
])
intermediary_points = [8, 4, 8, 4]

# corners = np.array([
    # [ 0.5, -0.5],
    # [-0.5, -0.5],
    # [-0.5,  0.5],
    # [ 0.5,  0.5]
# ])
# intermediary_points = [4, 4, 4, 4]

waypoints = []
waypoints.append([*corners[0], desired_depth, 0])

for i in range(4):
    corner = corners[i]
    next_corner = corners[(i + 1) % 4]
    for j in range(intermediary_points[i]):
        # eg: if intermediary_points[i] = 4
        # then new_waypoint divides the segment into 1:3, 2:2, 3:1, 4:0
        new_waypoint = corner + (next_corner - corner) * (j + 1) / intermediary_points[i]
        waypoints.append([*new_waypoint, desired_depth, 0])

waypoints = np.array(waypoints)

# ----------------- BUILT THE WAYPOINTS ------------------


## Mean Value - May have to change according to Battery Level
mean_vals = np.array([1500,1500,1470,1500])

print("[INFO] starting video stream...")
## The Resolution is for the Colour Stream from the D435.
## The Depth Stream is fixed at 720p
dc = DepthCamera(width=WIDTH, height=HEIGHT)

# Connecting to the drone
print("[INFO] : Connecting to the drone")
command = Pluto("192.168.4.1")

print("[INFO] : Taking Off")
for i in range(10):
    command.disarm()
    time.sleep(0.1)

command.takeoff()

# for i in range(10):
    # # command.set_attitude(throttle=1500,yaw = 1500,roll = 1485,pitch=1500)
    # command.boxarm()
    # time.sleep(0.1)
print('[INFO] : Takeoff Completed')

epsilon = 0.2

success = True
for waypoint in waypoints:
    kp, ki, kd = kp_max, ki_max, kd_max
    print("going to ", waypoint[:2])
    if not drone_navigation(waypoint, epsilon):
        success = False
        try:
            command.land()
        except KeyboardInterrupt:
            command.disarm()
        break
    print("reached  ", waypoint[:2])

dc.release()
cv2.destroyAllWindows()
command.land()

if success:
    print('[INFO] : Mission Completed')
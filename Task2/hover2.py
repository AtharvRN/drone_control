import time
import cv2
import cv2.aruco as aruco
import numpy as np
from realsense_depth import *
# import the necessary packages
from threading import Thread
from utlities import *
from control_class import *

## IMAGE RESOLUTION
WIDTH = 1280
HEIGHT = 720

out = cv2.VideoWriter('output.mp4',cv2.VideoWriter_fourcc(*"MP4V"),15,(1280,720))
t =[]
fps = []
def drone_pose(frame ,matrix_coefficients, distortion_coefficients,id_no):
    ## MARKER SIZE OF DRONE
    MARKER_SIZE = 0.056
    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[id_no], MARKER_SIZE, matrix_coefficients,distortion_coefficients)
    return tvec,rvec
flag = False
def Adjust_PID(error,derr,P,D): # this function adjust the values of P and D of roll, pitch and throttle according to the error and derivative
    
    '''Calculating conditional P and D values of roll, pitch and throttle'''

    print('Adjusting PID')
    for i in range(3): # we are only considering roll, pitch and throttle
        if abs(error[i]) < 0.02:
            D[i] *= 0.01
            
        elif abs(error[i]) < 0.5 and abs(derr[i]) > 3:
            P[i] *= 0.1
        elif abs(error[i]) > 0.55:
            D[i] *= 0.0
            if abs(derr[i]) > 3:
                P[i] *= 0.0
    return P,D
# defining an empty custom dictionary 
arucoDict = cv2.aruco.custom_dictionary(0, 4, 1)
# adding empty bytesList array to fill with 5 markers 
arucoDict.bytesList = np.empty(shape = (5, 2, 4), dtype = np.uint8)

# adding new markers
mybits = np.array([[0,1,0,0],[1,1,0,0],[1,0,1,0],[1,1,0,1]], dtype = np.uint8)
arucoDict.bytesList[0] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,1,1,1],[1,0,0,1],[1,0,0,1],[0,0,0,1],], dtype = np.uint8)
arucoDict.bytesList[1] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,0,1],[0,0,0,1],[1,0,1,0],[0,1,1,1]], dtype = np.uint8)
arucoDict.bytesList[2] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,0,0],[1,1,1,0],[1,0,1,1],[0,1,1,1],], dtype = np.uint8)
arucoDict.bytesList[3] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,1,0],[1,0,1,0],[0,0,0,0],[1,1,1,1]], dtype = np.uint8)
arucoDict.bytesList[4] = cv2.aruco.Dictionary_getByteListFromBits(mybits)

arucoParams = aruco.DetectorParameters_create()

calibration_matrix_path = "calibration_matrix.npy"
distortion_coefficients_path = "distortion_coefficients.npy"
    
k = np.load(calibration_matrix_path)*1.5
k[2][2] =1
d = np.load(distortion_coefficients_path)

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

time.sleep(2.0)

# Initialize the video stream
reached = False
first_time = True

num_not_detected = 0
eps = 0.1
desired_depth = 2.2
# PID PARAMETERS
command = Command("192.168.4.1")
## MEAN - HOVERING VALUES
# mean_roll =  1511
# mean_pitch = 1482
# mean_throttle = 1475

dc = DepthCamera(width=WIDTH, height=HEIGHT)
error = np.zeros(4)
derr = np.zeros(4)
error_sum = np.zeros(4)
prev_err = np.zeros(4)
## PITCH,ROLL,THROTTLE,YAW
kp = np.array([18,18,20,0])
ki = np.array([3.5,3.5,3,0])
kd = np.array([3,3,1,0])
mean_vals = np.array([1500,1500,1470,1500])

mean_roll =  1500
mean_pitch = 1500
mean_throttle = 1500
mean_yaw = 1500
angle = 0
for i in range(10):
    command.disarm()
    time.sleep(0.1)
# command.calib()
prev_depth = 0
time.sleep(0.3)
for i in range(10):
    command.disarm()
    time.sleep(0.1)
# command.takeoff()
time.sleep(0.2)
prev_time = time.time()
f_old = prev_time
start = f_old
print('[INFO] : Takeoff Completed')


prevx, prevy = None, None

def get_markers(img):
    global prevx, prevy
    if prevx is None:
        subimg = img
    else:
        xmin = max(prevx - 150, 0)
        ymin = max(prevy - 150, 0)
        xmax = min(prevy + 150, 1920)
        ymax = min(prevy + 150, 1080)
        subimg = img[ymin:ymax,xmin:xmax]
    corners, ids, rejected = cv2.aruco.detectMarkers(subimg, arucoDict, parameters=arucoParams)
    if prevx is not None:
        if corners:
            corners += [prevx, prevy]
            prevx, prevy = (corners[0] + corners[2]) // 2
        else:
            prevx, prevy = None, None
    return corners, ids, rejected

try:
    while True:

        ret, color_frame = dc.get_frame()

        gray = cv2.cvtColor(color_frame,cv2.COLOR_BGR2GRAY)
        #(corners, ids, rejected) = cv2.aruco.detectMarkers(gray,arucoDict, parameters=arucoParams)
        (corners, ids, rejected) = get_markers(gray)
        # command.disarm()

        if  0 in ids : 
                #print(ids)
                ids = ids.tolist()
                i = ids.index([0])

                if ( abs(corners[i][0][1][0] - corners[i][0][0][0]) > 100 or abs(corners[i][0][1][1] - corners[i][0][0][1]) > 100 ) :
                    print(' [ERROR] : Large Marker detected ...')
                
                color_frame,xc,yc = aruco_display(color_frame,corners[i])
                tvec,rvec = drone_pose(color_frame,k,d,i)
                x = tvec[0][0][0]
                y = tvec[0][0][1]
                z = tvec[0][0][2]


                point1 = corners[i][0][1] + corners[i][0][2]
                point2 = corners[i][0][0] + corners[i][0][3]
                if point1[0] == point2[0]:
                    angle = np.pi/2
                else: 
                    angle =  np.arctan((point1[1] - point2[1])/(point1[0] - point2[0]))
                num_not_detected = 0

        else :
            print('[INFO] : Drone not detected ...')
            if num_not_detected > 20:
                print('[INFO] : Drone out of range...')
                print('[INFO] : Landing ...')
                command.land()
                print('[INFO] : Quitting ...')
                break

            num_not_detected +=1
            #print(color_frame)
            scaled = cv2.resize(color_frame, (1280, 720), interpolation = cv2.INTER_CUBIC)
            cv2.imshow("Frame", scaled)
            curr_time = time.time()
            fps_ = 1/(curr_time-prev_time)
            prev_time=curr_time
            print('fps = ',fps_)
            fps.append(fps_)
            out.write(scaled)
            command.set_attitude(mean_throttle,mean_yaw,mean_pitch,mean_roll)
            
            key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            continue
        
        ## HOVERING 
        if first_time:
            desired_pos = np.array([x,y,desired_depth,0])
            first_time = False

        else:
            depth = dc.get_depth(xc,yc)
            #depth = z
            position = 'x :'+str(round(x*100))+'  y:'+str(round(y*100))+'  z :'+str(round(depth*100)) +'  angle :'+str(round(angle*180/np.pi))
            cv2.putText(color_frame, str(position),(100,100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                
            curr_pos = np.array([x,y,depth,angle])
        
            if(np.linalg.norm(desired_pos - curr_pos) < 0.2):
                print('[INFO] : Reached Position')
                #kd = [0.5,0.5,0.5,0]
                
            print('desired pos : ',desired_pos)
            print('curr_pos :', curr_pos )

            curr_time = time.time()
            t.append(curr_time-start)
            time_ = curr_time-prev_time
            #time_ = 0.045
            error = curr_pos-desired_pos
            derr = (error -prev_err)/(time_)

            P = kp*error
            I = ki*error_sum
            D =  kd*derr

            print(P,I,D)

           # P,D = Adjust_PID(error,derr,P,D)
            result = P+I+D
            print(result)

            # pitch = mean_vals[0] + result[0]
            # roll = mean_vals[1] + result[1]
            # throttle = mean_vals[2] + result[2]
            # yaw = mean_vals[3] + result[3]
            result = mean_vals+ (np.rint(result)).astype(int)
            


            print("pitch : ",result[0])
            print("roll : ",result[1])
            print("throttle : ",result[2])
            print("yaw : ",result[3])
            command.set_attitude(throttle = result[2], yaw = result[3],pitch = result[0],roll = result[1])
            error_sum += error*(time_)
            prev_time = curr_time
            scaled = cv2.resize(color_frame, (1280, 720), interpolation = cv2.INTER_CUBIC)
            cv2.imshow("Frame", scaled)
            out.write(scaled)
            #cv2.imshow("Frame", color_frame)

            
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        ## Updating the current time for FPS Calculation
        f_new = time.time()
        fps_ = 1/(f_new-f_old)
        fps.append(fps_)
        f_old = f_new
        print('[INFO] : fps = ',fps_)
        
        # Restricting FPS
        # time.sleep(0.01)

except KeyboardInterrupt:
    print('[INFO] : Keyboard Interrupt')
    command.boxarm()
    command.land()
# except:
#     print('[INFO] : Exception')
#     command.boxarm()
    

    command.land()

    
dc.release()
cv2.destroyAllWindows()

print(np.mean(fps))
print(len(t)/len(fps))


# Final Submission Team 21 #
## Problem Statement - Pluto Drone Swarm Challenge - Drona Aviation
#### Task 1 - Python wrapper
- Create a Python wrapper to control the drone movements. (Eg. Pitch forward, Roll left, take off - Land, etc.)
- Fly the drone using a Python wrapper (from a Linux Machine/Windows PC)
#### Task 2 - Pluto control
- Generate the ArUco tag and place it on the drone.
- Using ArUco tag, get a pose estimation of the drone.
- Add PID to the script for controlling the drone
- Hover the drone in one position.
- Move the drone in rectangular motion (1 x 2 meter)

#### Task 3 - Pluto Swarming
- Generate one more ArUco tag and place it on the second drone.
- Initially, Drone2 will be at position0, and drone1 will be at position1. Write
commands to move Drone1 from position1 to position2. When Drone1 reaches
position2, drone2 should follow drone1 and reach position1 automatically.
- Same way, create a rectangle motion. (1 x 2 meter)
### Steps to run locally
1. Clone the repository
  ```
  git clone https://github.com/AtharvRN/drone_control.git 
  ``` 
  OR
  ```
  git clone git@github.com:AtharvRN/drone_control.git
  ```
    
2. After cloning the repository, execute the following code in order to install the required libraries :

  ```shell
  pip install -r requirements.txt
  ```
3. In order to run Task 1 (using an Xbox controller), execute the following code:

  ```shell
  cd Task1/
  ```
  ```
  python comm_ontrol.py
  ```

3. (OR) In order to run Task 1 (using a keyboard) execute the following code:

```shell
cd Task1/
```
```
python ControlWithKeyboard.py
```

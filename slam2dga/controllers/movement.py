#!/usr/bin/env python3
import random, math
import time
import logging
from aux import Vector2D
from slamEKF_bb import ekf_slam, zeros_matrix, identity_matrix, get_matrix_inv, set_LM_Pos_into_state, get_LM_Pos_from_state

class GPS(object):
    """ GPS-based positioning sensor
    """

    def __init__(self, robot):
        """ Constructor
        """
        # Robot class
        self.robot = robot

    def getOrientation(self, degrees = False):

        if degrees:
            return math.degrees(self.robot.position.get_orientation())
        else:
            return self.robot.position.get_orientation()

    def getPosition(self):
        return Vector2D(self.robot.position.get_position()) 



class Odometry(object):
    """ Odometry-based positioning sensor
    """

    def __init__(self, robot, bias = 0):
        """ Constructor
        :type range: int
        :param enode: Random-Walk speed (tip: 500)
        """
        # Robot class
        self.robot = robot

        # Fixed Parameters
        self.L = 0.053          # Distance between wheels
        self.R = 0.0205         # Wheel radius

        # Internal variables 
        self.pos = Vector2D(self.robot.position.get_position())  
        self.ori = self.robot.position.get_orientation()

    def step(self):

        # Read odometry sensor
        dleft, dright = self.robot.differential_steering.get_distances()
        dleft, dright = dleft * 0.01, dright * 0.01

        # If going straight
        if abs(dleft-dright) < 1e-6:
            dt = 0
            dx = 0.5*(dleft+dright) * math.cos(self.ori)
            dy = 0.5*(dleft+dright) * math.sin(self.ori)

        # If rotating or curving
        else:
            dr  = (dleft+dright)/(2*(dright-dleft)) * self.L

            dt = (dright - dleft) / self.L
            dx =  dr * math.sin(dt+self.ori) - dr * math.sin(self.ori)
            dy = -dr * math.cos(dt+self.ori) + dr * math.cos(self.ori)

        # Update position and orientation estimates
        self.ori  = (self.ori + dt) % math.pi
        self.pos += Vector2D(dx,dy)

    def setOrientation(self, orientation = 0):
        self.ori = orientation

    def setPosition(self, position = [0,0]):
        self.pos = position

    def getOrientation(self):
        return self.ori

    def getPosition(self):
        return self.pos

class OdoCompass(object):
    """ Odometry-based positioning sensor with compass
    """

    def __init__(self, robot, bias = 0, variance = 0):
        """ Constructor
        :type range: int
        :param enode: Random-Walk speed (tip: 500)
        """
        # Robot class
        self.robot = robot

        # Fixed Parameters
        self.L = 0.053          # Distance between wheels
        self.R = 0.0205         # Wheel radius

        # Internal variables
        self.pos = Vector2D(self.robot.position.get_position())  
        self.ori = self.robot.position.get_orientation()

    def step(self):       

        # Read odometry sensor 
        dl, dr = self.robot.differential_steering.get_distances()
        dl, dr = dl * 0.01, dr * 0.01

        # Read compass sensor
        dt = self.robot.position.get_orientation() - self.ori

        # Add noise to readings
        dl += random.gauss(0, 0.01)
        dr += random.gauss(0, 0.01)
        dt += random.gauss(0, math.radians(0))

        # Calculate delta in position
        dx = (dl+dr)/2 * math.cos(self.ori + dt/2) 
        dy = (dl+dr)/2 * math.sin(self.ori + dt/2) 

        # Update position and orientation estimates
        self.ori += dt 
        self.pos += Vector2D(dx,dy)

    def setOrientation(self):
        self.ori = self.robot.position.get_orientation()

    def setPosition(self):
        self.pos = Vector2D(self.robot.position.get_position())

    def getOrientation(self):
        return self.ori

    def getPosition(self):
        return self.pos

class Navigate(object):
    """ Set up a Navigation loop on a background thread
    The __navigating() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, robot, MAX_SPEED, withKF = True, fric=0):
        """ Constructor
        :type range: int
        :param enode: Random-Walk speed (tip: 500)
        """
        self.robot = robot
        self._id = str(int(robot.variables.get_id()[2:])+1)

        # Internal variables
        self.__stop = False
        self.__walk = True
        self.__distance_to_target = 0
        self.__window = []
        self.__distance_traveled = 0
        self.__old_vec  = 0
        self.__old_time = 0
        self.__accumulator_I = 0
        self.__accumulator_stuck = 0

        # Fixed Parameters
        self.MAX_SPEED = MAX_SPEED        # Maximum robot speed
        self.L = 0.053                    # Distance between wheels
        self.R = 0.0205                   # Wheel radius

        # Navigation parameters
        self.Kp = 50                                    # Angular velocity proportional gain
        self.window_size = 10                           # Avoid vector filtering           
        self.thresh = math.radians(70)                  # Wait before moving
        self.thresh_front_collision = math.radians(15)  # Aggressive avoidance

        # Obstacle avoidance parameters
        self.thresh_ir    = 0
        self.weights      = 25*[-10, -10, 0, 0, 0, 0, 10, 10]
        self.lm_internal_ids = []

        # Vectorial obstacle avoidance parameters
        self.Ki = 0.1
        # Init kalman filter
        if withKF:
            print("Navigation module is init the SLAM EKF")
            curpos = robot.position.get_position()[0:2]
            cur_ori = robot.position.get_orientation()
            self.slam_xEst = zeros_matrix(3,1)
            self.slam_pEst = identity_matrix(3)
            self.slam_xEst[0][0] = curpos[0]
            self.slam_xEst[1][0] = curpos[1]
            self.slam_xEst[2][0] = cur_ori
            self.slam_frozen_lm_id = [] #lm global locations that have became a part of consensus and frozen
            self.slam_frozen_lm_poses = [] #poses of the landmarks that have been frozen
    def set_kfstate(self, curpos, curori):
        self.slam_xEst[0][0] = curpos[0]
        self.slam_xEst[1][0] = curpos[1]
        self.slam_xEst[2][0] = curori

    def get_kfstate(self):
        return [self.slam_xEst[0][0],
                self.slam_xEst[1][0],
                self.slam_xEst[2][0]]

    def get_kf_state_error(self):
        return math.dist(self.robot.position.get_position()[0:2], [self.slam_xEst[0][0],self.slam_xEst[1][0] ])

    def set_lm_frozen(self, lm_id_g, lm_pose_g):
        if lm_id_g !=-1 and lm_id_g not in self.slam_frozen_lm_id and lm_id_g in self.lm_internal_ids:
            self.slam_frozen_lm_id.append(lm_id_g)
            self.slam_frozen_lm_poses.append(lm_pose_g)
            if lm_id_g in self.lm_internal_ids:
                self.slam_xEst = set_LM_Pos_into_state(self.slam_xEst , self.lm_internal_ids.index(lm_id_g), lm_pose_g)


    def fwd_cycle_kf(self, ud, lm_poses,pose_measure=[]):
        #repeat frozen lms:
        for idx, lm_id_g in enumerate(self.slam_frozen_lm_id):
            lm_pose_g = self.slam_frozen_lm_poses[idx]
            self.slam_xEst = set_LM_Pos_into_state(self.slam_xEst , self.lm_internal_ids.index(lm_id_g), lm_pose_g)
        self.slam_xEst, self.slam_pEst, self.lm_internal_ids, pred_pos = ekf_slam(self.slam_xEst,
                                                  self.slam_pEst,
                                                  ud,
                                                  lm_poses,
                                                  pose_measure,
                                                  self.lm_internal_ids,
                                                  self.slam_frozen_lm_id)
        #print("EA poses: ", self.slam_xEst, self.robot.position.get_position()[0:2])
        return pred_pos

    def update_state(self, target = [0,0]):
        self.position = Vector2D(self.robot.position.get_position()[0:2])
        self.orientation = self.robot.position.get_orientation()
        self.target = Vector2D(target)

    def navigate(self, target = [0,0]):

        # Update the current position, orientation and target of the robot
        self.update_state(target = target)
        
        # Calculate the local frame vector to the desired target
        vec_target = (self.target-self.position).rotate(-self.orientation)
        

        T = vec_target.normalize()

        # The desired vector (we only care about direction)
        D = T

        self.update_rays(Vector2D(0,0),Vector2D(0,0),D)

        dotProduct = 0
        # The target angle is behind the robot, we just rotate, no forward motion
        if D.angle > self.thresh or D.angle < -self.thresh:
            dotProduct = 0

        # Else, we project the forward motion vector to the desired angle
        else:
            dotProduct = Vector2D(1,0).dot(D.normalize())

        # The angular velocity component is the desired angle scaled linearly
        angularVelocity = self.Kp * D.angle

        # The final wheel speeds are computed combining the forward and angular velocities
        right = dotProduct * self.MAX_SPEED/2 - angularVelocity * self.L
        left = dotProduct * self.MAX_SPEED/2 + angularVelocity * self.L

        # Set wheel speeds
        self.robot.epuck_wheels.set_speed(right, left)

        # Store the distance to arrive at target
        self.__distance_to_target = abs(vec_target)

    def navigate_with_obstacle_avoidance(self, target = [0,0], use_kf_pos=True):

        # Update the current position, orientation and target of the robot
        self.update_state(target = target)
        if use_kf_pos:
            vec_target = (self.target - Vector2D([self.slam_xEst[0][0], self.slam_xEst[1][0]])).rotate(-self.orientation)
        else:
        # Calculate the local frame vector to the desired target
            vec_target = (self.target-self.position).rotate(-self.orientation)

        # Calculate the local frame vector to the avoid objects
        acc = Vector2D()
        prox_readings = self.robot.epuck_proximity.get_readings()

        for reading in prox_readings:
            #print("my proxi readings: ", reading.value, reading.angle.value())
            acc += Vector2D(reading.value, reading.angle.value(), polar = True)

        # Generate a time window to have smoother variations on the avoid vector
        self.__window.append(acc)
        if len(self.__window) > self.window_size:
            self.__window.pop(0)
        
        vec_avoid = (1/self.window_size) * sum(self.__window, start = Vector2D()) 

        if abs(vec_avoid) < 0.01:
            self.__window = []

        # Normalize and weight to obtain desired control behavior

        T = 0.2 * vec_target.normalize()
        A = -0.5 * vec_avoid#.normalize()

        # Saturate the avoid angle
        if abs(A.angle) > math.radians(90):
            A = Vector2D(A.length, math.copysign(math.radians(90), A.angle), polar = True)


        # The desired vector (we only care about heading)
        D =  (T + A)

        self.update_rays(T,A,D)

        # The target angle is behind the robot, we just rotate, no forward motion
        if D.angle > self.thresh or D.angle < -self.thresh:
            dotProduct = 0

        # Else, we project the forward motion vector to the desired angle
        else:
            dotProduct = Vector2D(1,0).dot(D.normalize())

        # The angular velocity component is the desired angle scaled linearly
        angularVelocity = self.Kp * D.angle

        # The final wheel speeds are computed combining the forward and angular velocities
        right = dotProduct * self.MAX_SPEED/2 - angularVelocity * self.L
        left = dotProduct * self.MAX_SPEED/2 + angularVelocity * self.L

        # Set wheel speeds
        self.robot.epuck_wheels.set_speed(right, left)

        # Store the distance to arrive at target
        self.__distance_to_target = abs(vec_target)

    def update_rays(self, T, A, D):
        # Change to global coordinates for the plotting

        self.robot.variables.set_attribute("rays", self._id 
                                            + ', ' + repr(self.position) 
                                            + ', ' + repr(T.rotate(self.orientation)) 
                                            + ', ' + repr(A.rotate(self.orientation))
                                            + ', ' + repr(D.rotate(self.orientation)) 
                                            +'\n')

    def avoid(self, left = 0, right = 0, move = False):
        obstacle = False
        avoid_left = avoid_right = 0

        # Obstacle avoidance
        readings = self.robot.epuck_proximity.get_readings()
        self.ir = [reading.value for reading in readings]
                
        # Find Wheel Speed for Obstacle Avoidance
        for i, reading in enumerate(self.ir):
            if reading > self.thresh_ir:
                obstacle = True
                avoid_left  += self.weights[i] * reading
                avoid_right -= self.weights[i] * reading

        if obstacle:
            left  = self.MAX_SPEED/2 + avoid_left
            right = self.MAX_SPEED/2 + avoid_right

        if move:
            self.robot.epuck_wheels.set_speed(right, left)

        return left, right

    def avoid_static(self, left = 0, right = 0, move = False):

        # Update the current position, orientation and target of the robot
        self.update_state()
        
        # Calculate the local frame vector to the avoid objects
        acc = Vector2D()
        prox_readings = self.robot.epuck_proximity.get_readings()

        for reading in prox_readings:
            acc += Vector2D(reading.value, reading.angle.value(), polar = True)

        # Project the forward motion vector to the desired angle
        dotProduct = Vector2D(1,0).dot(acc.normalize())
        
        # The angular velocity component is the desired angle scaled linearly
        angVelocity = self.Kp * acc.angle

        # The final wheel speeds are computed combining the forward and angular velocities
        right = dotProduct * self.MAX_SPEED/2 - angVelocity * self.L
        left = dotProduct * self.MAX_SPEED/2 + angVelocity * self.L

        if move:
            self.robot.epuck_wheels.set_speed(right, left)

        return left, right


    def stop(self):
        robot.epuck_wheels.set_speed(0,0)

    def set_wheels(self, right, left):
        robot.epuck_wheels.set_speed(right,left)

    def get_distance_to(self, to, use_kf_distance=True):

        # Update the current position, orientation and target of the robot
        self.update_state(target = to)

        # Return the distance to
        if use_kf_distance:
            mypos = Vector2D([self.slam_xEst[0][0], self.slam_xEst[1][0]])
        else:
            mypos = self.position
        return abs(self.target-mypos)


    def saturate(self, left, right, style = 1):
        # Saturate Speeds greater than MAX_SPEED

        if style == 1:

            if left > self.MAX_SPEED:
                left = self.MAX_SPEED
            elif left < -self.MAX_SPEED:
                left = -self.MAX_SPEED

            if right > self.MAX_SPEED:
                right = self.MAX_SPEED
            elif right < -self.MAX_SPEED:
                right = -self.MAX_SPEED

        else:

            if abs(left) > self.MAX_SPEED or abs(right) > self.MAX_SPEED:
                left = left/max(abs(left),abs(right))*self.MAX_SPEED
                right = right/max(abs(left),abs(right))*self.MAX_SPEED

        return left, right


class RandomWalk(object):
    """ Set up a Random-Walk loop on a background thread
    The __walking() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, robot, MAX_SPEED):
        """ Constructor
        :type range: int
        :param enode: Random-Walk speed (tip: 500)
        """
        self.__stop = 1
        self.__walk = True
        self._id = str(int(robot.variables.get_id()[2:]) + 1)
        self.__distance_to_target = None
        self.__distance_traveled = 0
        self.__left_wheel_speed = 0
        self.__right_wheel_speed = 0

        # General Parameters
        self.robot = robot
        self.MAX_SPEED = MAX_SPEED

        # Random walk parameters
        self.remaining_walk_time = 3
        self.my_lambda = 10  # Parameter for straight movement
        self.turn = 8
        self.possible_directions = ["straight", "cw", "ccw"]
        self.actual_direction = "straight"

        # Navigation parameters
        self.L = 0.053  # Distance between wheels
        self.R = 0.0205  # Wheel radius
        self.Kp = 5  # Proportional gain
        self.thresh = math.radians(70)  # Wait before moving

        # Obstacle avoidance parameters
        self.thresh_ir = 0
        self.weights = 50 * [-10, -10, 0, 0, 0, 0, 10, 10]

        # Vectorial obstacle avoidance parameters
        self.vec_avoid = []
        self.__old_time = time.time()
        self.__old_vec = 0
        self.__accumulator_I = 0

    def get_current_speed(self):
        return self.__left_wheel_speed, self.__right_wheel_speed

    def step(self):
        """ This method runs in the background until program is closed """

        if self.__walk:
            # Levy random-Walk
            left, right = self.random()

            # Avoid Obstacles
            left, right = self.avoid_argos3_example(left, right)

            # Saturate wheel actuators
            left, right = self.saturate(left, right)

            # Set wheel speeds
            #right -= right * random.random() * self.friction
            #left -= left * random.random() * self.friction
            self.__right_wheel_speed = right
            self.__left_wheel_speed = left
            self.robot.epuck_wheels.set_speed(right, left)

            # No rays for plotting
            self.robot.variables.set_attribute("rays", "")

    def random(self):

        # Decide direction to take
        if (self.remaining_walk_time == 0):
            if self.actual_direction == "straight":
                self.actual_direction = random.choice(self.possible_directions)
                self.remaining_walk_time = random.randint(0, self.turn)
            else:
                self.actual_direction = "straight"
                self.remaining_walk_time = math.ceil(random.expovariate(1/(self.my_lambda * 4)))
        else:
            self.remaining_walk_time -= 1

        # Return wheel speeds
        speed = self.MAX_SPEED/2

        if (self.actual_direction == "straight"):
            return speed, speed

        elif (self.actual_direction == "cw"):
            return speed, -speed

        elif (self.actual_direction == "ccw"):
            return -speed, speed


    def avoid_vec_lua(self, left, right):
    # Obstacle avoidance; translated from Lua
    # Source: swarm intelligence examples

        accumul = Vector2D(0,0)

        readings = self.robot.epuck_proximity.get_readings()
        
        for reading in readings:

            # We calculate the x and y components given length and angle
            vec = Vector2D(reading.value, reading.angle.value(), polar = True)

            # We sum the vectors into a variable called accumul
            accumul += vec

        # We get length and angle of the final sum vector
        length, angle = accumul.to_polar()

        if length > 0.01:
            # print(length)
            # If the angle is greater than 0 the resulting obstacle is on the left. 
            # Otherwise it is on the right
            # We turn with a speed that depends on the angle. 
            # The closer the obstacle to the x axis of the robot, the quicker the turn
            speed = max(0.5, math.cos(angle)) * self.MAX_SPEED

            if angle > 0:
                right = speed
                left = 0
            else:
                right = 0
                left = speed

        return left, right


    def avoid(self, left = 0, right = 0, move = False):

        obstacle = avoid_left = avoid_right = 0

        # Obstacle avoidance
        readings = self.robot.epuck_proximity.get_readings()
        self.ir = [reading.value for reading in readings]
                
        # Find Wheel Speed for Obstacle Avoidance
        for i, reading in enumerate(self.ir):
            if reading > self.thresh_ir:
                obstacle = True
                avoid_left  += self.weights[i] * reading
                avoid_right -= self.weights[i] * reading

        if obstacle:
            left  = self.MAX_SPEED/2 + avoid_left
            right = self.MAX_SPEED/2 + avoid_right

        if move:
            self.robot.epuck_wheels.set_speed(right, left)

        return left, right


    def avoid_argos3_example(self, left, right):
        # Obstacle avoidance; translated from C++
        # Source: https://github.com/ilpincy/argos3-examples/blob/master/controllers/epuck_obstacleavoidance/epuck_obstacleavoidance.cpp
        
        readings = self.robot.epuck_proximity.get_readings()
        self.ir = [reading.value for reading in readings]

        fMaxReadVal = self.ir[0]
        unMaxReadIdx = 0

        if(fMaxReadVal < self.ir[1]): 
            fMaxReadVal = self.ir[1]
            unMaxReadIdx = 1

        if(fMaxReadVal < self.ir[7]): 
            fMaxReadVal = self.ir[7]
            unMaxReadIdx = 7

        if(fMaxReadVal < self.ir[6]): 
            fMaxReadVal = self.ir[6]
            unMaxReadIdx = 6

        # Do we have an obstacle in front?
        if(fMaxReadVal > 0):
            # Yes, we do: avoid it 
            if(unMaxReadIdx == 0 or unMaxReadIdx == 1): 
                # The obstacle is on the right, turn left 
                left, right = -self.MAX_SPEED/2, self.MAX_SPEED/2
            else: 
                # The obstacle is on the left, turn right 
                left, right = self.MAX_SPEED/2, -self.MAX_SPEED/2       

        return left, right
        
    def avoid_vec_lua_erb(self, left, right):
        # Obstacle avoidance; translated from LUA
        # Source: swarm intelligence ULB course

        accumul = Vector2D(0,0)

        pos = tuple(self.robot.position.get_position()[0:2])
        ori = self.robot.position.get_orientation()

        readings = self.robot.epuck_range_and_bearing.get_readings()
        
        for reading in readings:

            # if reading[2] < math.radians(15):
            #     continue

            # else:
            # We calculate the x and y components given length and angle
            vec = Vector2D(reading[1]/100, reading[2]+ori, polar=True)

            # We sum the vectors into a variable called accumul
            accumul += vec

        # We get length and angle of the final sum vector
        length, angle = accumul.to_polar()
        # print(length, angle)

        if 0.05 < length < 0.1:
            # If the angle is greater than 0 the resulting obstacle is on the left. 
            # Otherwise it is on the right
            # We turn with a speed that depends on the angle. 
            # The closer the obstacle to the x axis of the robot, the quicker the turn
            speed = max(0.5, math.cos(angle)) * self.MAX_SPEED
            if angle > 0:
                right = speed
                left = 0
            else:
                right = 0
                left = speed

        return left, right


    def saturate(self, left, right, style = 1):
        # Saturate Speeds greater than MAX_SPEED

        if style == 1:

            if left > self.MAX_SPEED:
                left = self.MAX_SPEED
            elif left < -self.MAX_SPEED:
                left = -self.MAX_SPEED

            if right > self.MAX_SPEED:
                right = self.MAX_SPEED
            elif right < -self.MAX_SPEED:
                right = -self.MAX_SPEED

        else:

            if abs(left) > self.MAX_SPEED or abs(right) > self.MAX_SPEED:
                left = left/max(abs(left),abs(right))*self.MAX_SPEED
                right = right/max(abs(left),abs(right))*self.MAX_SPEED

        return left, right

    def setWalk(self, state):
        """ This method is called set the random-walk to on without disabling I2C"""
        self.__walk = state

    def setLEDs(self, state):
        """ This method is called set the outer LEDs to an 8-bit state """
        if self.__LEDState != state:
            self.__isLEDset = False
            self.__LEDState = state
        
    def getIr(self):
        """ This method returns the IR readings """
        return self.ir

    def start(self):
        pass

    def stop(self):
        pass




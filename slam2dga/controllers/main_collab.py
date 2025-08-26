#!/usr/bin/env python3
# This is the main control loop running in each argos robot

# /* Import Packages */
#######################################################################
import random, math
import sys, os
import pickle


experimentFolder = os.environ['EXPERIMENTFOLDER']
sys.path += [os.environ['MAINFOLDER'], \
             os.environ['EXPERIMENTFOLDER'] + '/controllers', \
             os.environ['EXPERIMENTFOLDER']
             ]

from controllers.movement import RandomWalk, Navigate, Odometry, OdoCompass
from controllers.groundsensor import ResourceVirtualSensor, Resource
from controllers.erandb import ERANDB
from controllers.rgbleds import RGBLEDs
from controllers.aux import *
from controllers.aux import Timer
from controllers.statemachine import *

from controllers.control_params import params as cp
from loop_functions.loop_params import params as lp

from toychain.src.Node import Node
from toychain.src.Block import Block, State
from toychain.src.utils import gen_enode

from toychain.src.consensus.ProofOfAuth import ProofOfAuthority
from toychain.src.Transaction import Transaction
from slamEKF_bb import ekf_slam, zeros_matrix, identity_matrix, get_matrix_inv
from collections import defaultdict
from aux import EpsilonGreedyAgent
import toychain.src.GPClustering_sym as gpc
import sympy as sp

# /* Global Variables */
#######################################################################
global robot

global startFlag
startFlag = False

global txList, tripList, submodules
txList, tripList, submodules = [], [], []

global clocks, counters, logs, txs
clocks, counters, logs, txs = dict(), dict(), dict(), dict()

# global slam experiments related variables
global seen_lm_ids, seem_lm_counts, confirmed_wp, my_speed, my_rot_speed, previous_pos, previous_ori, point_to_verify, obs_noise_factor, pos_noise_factor
seen_lm_ids = []
seem_lm_counts = []
confirmed_wp = []
my_speed = 0
my_rot_speed = 0
previous_pos = [0, 0]
previous_ori = 0
point_to_verify = [0, 0]
obs_noise_factor = 0
pos_noise_factor = 0

# /* Logging Levels for Console and File */
#######################################################################
import logging

loglevel = 10
logtofile = False

# /* Experiment Global Variables */
#######################################################################

clocks['peering'] = Timer(5)
clocks['sensing'] = Timer(2)
clocks['block'] = Timer(150)
clocks['save_points'] = Timer(15)
clocks['log_pos_error'] = Timer(30)
rwSpeed = cp['scout_speed']
navSpeed = cp['recruit_speed']

# initial velocity and speed test
counters['velocity_test'] = 0
counters['rotation_test'] = 0


cache = Resource({"x": lp['cache']['x'], "y": lp['cache']['y'], "radius": lp['cache']['r']})

global geth_peer_count

GENESIS = Block(0, 0000, [], [gen_enode(i + 1) for i in range(int(lp['environ']['NUMROBOTS']))], 0, 0, 0, nonce=1,
                state=State())

    ####################################################################################################################################################################################


#### INIT STEP #####################################################################################################################################################################
####################################################################################################################################################################################

def init():
    global clocks, counters, logs, submodules, me, rw, nav, odo, w3, fsm, erb, rgb, obs_noise_factor, pos_noise_factor, state_reward_history, eg_agent, my_action_count, num_lm_confirmed, num_lm_discovered, num_lm_discovered_list
    robotID = str(int(robot.variables.get_id()[2:]) + 1)
    robotIP = '127.0.0.1'
    robot.variables.set_attribute("id", str(robotID))
    robot.variables.set_attribute("circle_color", "gray50")
    robot.variables.set_attribute("foraging", "")
    robot.variables.set_attribute("state", "")
    robot.variables.set_attribute("block", "0")
    robot.variables.set_attribute("hash", str(hash("genesis")))
    robot.variables.set_attribute("state_hash", str(hash("genesis")))

    # /* Initialize Console Logging*/
    #######################################################################
    log_folder = experimentFolder + '/logs/' + robotID + '/'

    # Monitor logs (recorded to file)
    name = 'monitor.log'
    os.makedirs(os.path.dirname(log_folder + name), exist_ok=True)
    logging.basicConfig(filename=log_folder + name, filemode='w+',
                        format='[{} %(levelname)s %(name)s] %(message)s'.format(robotID))
    logging.getLogger('sc').setLevel(20)
    logging.getLogger('w3').setLevel(70)
    logging.getLogger('poa').setLevel(70)
    robot.log = logging.getLogger()
    robot.log.setLevel(10)

    # /* Initialize submodules */
    #######################################################################
    # # /* Init web3.py */
    robot.log.info('Initialising Python Geth Console...')
    w3 = Node(robotID, robotIP, 1233 + int(robotID), ProofOfAuthority(GENESIS))

    # /* Init an instance of peer for this Pi-Puck */
    me = Peer(robotID, robotIP, w3.enode, w3.key)

    # /* Init E-RANDB __listening process and transmit function
    robot.log.info('Initialising RandB board...')
    erb = ERANDB(robot, cp['erbDist'], cp['erbtFreq'])

    # /* Init Random-Walk, __walking process */
    robot.log.info('Initialising random-walk...')
    rw = RandomWalk(robot, cp['scout_speed'])

    # /* Init Navigation, __navigate process */
    robot.log.info('Initialising navigation...')
    nav = Navigate(robot, cp['recruit_speed'])

    # /* Init odometry sensor */
    robot.log.info('Initialising odometry...')
    odo = OdoCompass(robot)


    # /* Init LEDs */
    rgb = RGBLEDs(robot)

    # /* Init Finite-State-Machine */
    fsm = FiniteStateMachine(robot, start=States.IDLE)

    # List of submodules --> iterate .start() to start all
    submodules = [erb]
    num_lm_confirmed=0
    num_lm_discovered = 0
    num_lm_discovered_list = []

    # epsilon greedy agent for state selection
    eg_agent = EpsilonGreedyAgent(epsilon=0.2, num_bandits=2)
    if lp['generic']['load_eaparam'] == 1:
        #path of last experiment log
        with open(f"{experimentFolder}/results/data/EXAMPLE/001/{me.id}/action_estimate.csv", 'r') as file:
            last_line = file.readlines()[-1]
        parsed_line = last_line.strip().split(' ')
        eg_agent.setState([float(parsed_line[2]), float(parsed_line[3])])
        #print(parsed_line,eg_agent.getState())

    if int(robotID) > lp['generic']['num_1'] and int(robotID) <= lp['generic']['num_robots'] - lp['generic'][
        'num_m']:  # robot ID starts from 1
        obs_noise_factor = 1
        pos_noise_factor = 1
    elif int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
        print(robotID, ' is malicious')
        obs_noise_factor = 0.5
        pos_noise_factor = 0.1
    else:
        obs_noise_factor = 0.5
        pos_noise_factor = 0.1
    print(robotID, " set noise factors to: ", obs_noise_factor, pos_noise_factor)

    # /* Initialize logmodules*/
    #######################################################################
    # Experiment data logs (recorded to file)
    # name   = 'resource.csv'
    # header = ['COUNT']
    # logs['resources'] = Logger(log_folder+name, header, rate = 5, ID = me.id)

    name = 'balance.csv'
    header = ['BAL', 'FBAL', 'TIME']
    logs['balance'] = Logger(log_folder + name, header, ID=me.id)

    name = 'action_estimate.csv'
    header = ['ac0', 'ac1', 'TIME']
    logs['action_estimates'] = Logger(log_folder + name, header, ID=me.id)

    name = 'lm_confirm.csv'
    header = ['lmx', 'lmy',  'ERROR', 'TIME']
    logs['lm_confirm'] = Logger(log_folder + name, header, ID=me.id)

    name = 'lm_discovery.csv'
    header = ['lmx', 'lmy', 'ID', 'TIME']
    logs['lm_discovery'] = Logger(log_folder + name, header, ID=me.id)

    name = 'self_pos_error.csv'
    header = ['spx', 'spy', 'gtx', 'gty', 'TIME', 'ERROR', 'TIME']
    logs['self_pos_error'] = Logger(log_folder + name, header, ID=me.id)

    txs['report'] = None

    state_reward_history = []
    my_action_count = [0, 0]


#########################################################################################################################
#### CONTROL STEP #######################################################################################################
#########################################################################################################################
global pos
pos = [0, 0]
global last
last = 0
global last_leave_decision_epoch
last_leave_decision_epoch = 0
global last_join_decision_epoch
last_join_decision_epoch = 0


def controlstep():
    global last, pos, clocks, counters, startFlag, startTime, seen_lm_ids, seem_lm_counts, confirmed_wp, my_speed, my_rot_speed, previous_pos, previous_ori, point_to_verify, point_to_discover, obs_noise_factor, pos_noise_factor, eg_agent, state_reward_history, my_action_count, num_lm_confirmed, num_lm_discovered, num_lm_discovered_list
    robotID = str(int(robot.variables.get_id()[2:]) + 1)
    if not startFlag:
        ##########################
        #### FIRST STEP ##########
        ##########################

        startFlag = True
        startTime = 0

        robot.log.info('--//-- Starting Experiment --//--')

        for module in submodules:
            try:
                module.start()
            except:
                robot.log.critical('Error Starting Module: %s', module)
                sys.exit()

        for log in logs.values():
            log.start()

        for clock in clocks.values():
            clock.reset()

        # Startup transactions

        txdata = {'function': 'register', 'inputs': []}
        tx = Transaction(sender=me.id, data=txdata)
        w3.send_transaction(tx)

        txdata = {'function': 'register', 'inputs': []}
        tx = Transaction(sender=me.id, data=txdata)
        w3.send_transaction(tx)

        w3.start_tcp()
        w3.start_mining()
    else:

        ###########################
        ######## ROUTINES #########
        ###########################

        def peering():

            # Get the current peers from erb
            erb_enodes = {w3.gen_enode(peer.id) for peer in erb.peers}

            # Add peers on the toychain
            for enode in erb_enodes - set(w3.peers):
                try:
                    w3.add_peer(enode)
                except Exception as e:
                    raise e

            # Remove peers from the toychain
            for enode in set(w3.peers) - erb_enodes:
                try:
                    w3.remove_peer(enode)
                except Exception as e:
                    raise e

            # Turn on LEDs according to geth peer count
            rgb.setLED(rgb.all, rgb.presets.get(len(w3.peers), 3 * ['red']))

        def sensing(withnoise=True):
            encoded_string=robot.variables.get_attribute("seen")
            relative_pose = []
            relative_pose_for_nav = []
            abs_lm_pose = []
            noisy_dxdy=[]
            seen_lm_in_range = False
            closest_lm_distance = 100
            if encoded_string != "":
                decoded_list = list(map(int, encoded_string.split(',')))
                for lm_idx, lm_loc in enumerate(lp['landmark']['positions']):
                    for this_lm_id in decoded_list:
                        if lm_idx==this_lm_id:
                            robot_loc = robot.position.get_position()
                            robot_ori = robot.position.get_orientation()
                            dx = lm_loc[0] - robot_loc[0]
                            dy = lm_loc[1] - robot_loc[1]

                            pd = math.sqrt(dx ** 2 + dy ** 2)
                            if pd<closest_lm_distance:
                                closest_lm_distance=pd
                            if pd<= lp['generic']['lmobs_range']: #landmarks within the high-precision perception range
                                if withnoise:
                                    dx += (random.random()-0.5) * lp['generic']['lmobs_noise']*obs_noise_factor
                                    dy += (random.random() - 0.5) * lp['generic']['lmobs_noise']*obs_noise_factor
                                d = math.sqrt(dx ** 2 + dy ** 2)
                                angle = pi_2_pi(math.atan2(dy, dx) - robot_ori)
                                zi = [d, angle, lm_idx]
                                relative_pose.append(zi)
                                noisy_dxdy.append([dx, dy, lm_idx])
                                abs_lm_pose.append([lm_loc[0], lm_loc[1], lm_idx])
                            elif pd <= lp['generic']['lmsensing_range']:
                                dx += (random.random() - 0.5) * lp['generic']['lmobs_noise']*0.1
                                dy += (random.random() - 0.5) * lp['generic']['lmobs_noise']*0.1
                                d = math.sqrt(dx ** 2 + dy ** 2)
                                angle = pi_2_pi(math.atan2(dy, dx) - robot_ori)
                                zia = [d, angle, lm_idx]
                                relative_pose_for_nav.append(zia)

                                dx += (random.random()-0.5) * lp['generic']['lmobs_noise']*obs_noise_factor
                                dy += (random.random() - 0.5) * lp['generic']['lmobs_noise']*obs_noise_factor
                                d = math.sqrt(dx ** 2 + dy ** 2)
                                angle = pi_2_pi(math.atan2(dy, dx) - robot_ori)
                                zib = [d, angle, lm_idx]
                                relative_pose.append(zib)
                                noisy_dxdy.append([dx, dy, lm_idx])
                                abs_lm_pose.append([lm_loc[0], lm_loc[1], lm_idx])
                            if pd <= lp['generic']['lmsensing_range']:
                                seen_lm_in_range = True

            curpos = robot.position.get_position()[0:2]
            curori = robot.position.get_orientation()

            if withnoise:
                curpos[0] += (random.random()-0.5) * lp['generic']['sp_noise']*pos_noise_factor
                curpos[1] += (random.random() - 0.5) * lp['generic']['sp_noise']*pos_noise_factor
            self_pose = [[curpos[0]], [curpos[1]], [curori]]
            return relative_pose, self_pose, abs_lm_pose, noisy_dxdy, seen_lm_in_range, relative_pose_for_nav, closest_lm_distance

        def sensing_dxdy():
            encoded_string = robot.variables.get_attribute("seen")
            relative_pose = []
            if encoded_string != "":
                decoded_list = list(map(int, encoded_string.split(',')))
                for lm_idx, lm_loc in enumerate(lp['landmark']['positions']):
                    for this_lm_id in decoded_list:
                        if lm_idx == this_lm_id:
                            robot_loc = robot.position.get_position()
                            robot_ori = robot.position.get_orientation()
                            dx = robot_loc[0] - lm_loc[0]
                            dy = robot_loc[1] - lm_loc[1]
                            d = math.sqrt(dx ** 2 + dy ** 2)
                            angle = pi_2_pi(math.atan2(dy, dx) - robot_ori)
                            zi = [d, angle, lm_idx]
                            relative_pose.append(zi)
            return relative_pose

        def get_full_balance():
            blockchain = w3.chain
            wallet_balance = blockchain[-1].state.tokens[int(me.id) - 1]
            all_balance = wallet_balance
            for point in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports + blockchain[-1].state.task_tree.root.get_child("TaskNode2").reports:
                if int(point[1]) == int(me.id) and point[2] == 0:
                    all_balance += point[5]
            return wallet_balance, all_balance

        def find_cluster_center(unconfirmed_points):
            cluster_info = defaultdict(lambda: {"center": [0, 0], "count": 0, "me_verified": False})
            # Iterate through each point to build clusters.
            for this_point in unconfirmed_points:
                cluster_id = this_point[3]
                verified_by_myself = False
                if this_point[1] == me.id:
                    # print(this_point, me.id)
                    verified_by_myself = True
                # Skip confirmed point id.
                if cluster_id != -1:
                    cluster_info[cluster_id]["center"][0] += this_point[0][0]
                    cluster_info[cluster_id]["center"][1] += this_point[0][1]
                    cluster_info[cluster_id]["count"] += 1
                    if verified_by_myself == True:
                        cluster_info[cluster_id]["me_verified"] = True
            # Compute the average center of each cluster.
            for cluster_id, info in cluster_info.items():
                info["center"][0] /= info["count"]
                info["center"][1] /= info["count"]
            # for info in cluster_info.values():
            #    print(info["center"], info["me_verified"], [info["center"] for info in cluster_info.values() if info["me_verified"]==False])
            return [info["center"] for info in cluster_info.values() if info["me_verified"] == False], [info["center"] for info in cluster_info.values() if info["me_verified"] == True], [
                info["me_verified"] for info in cluster_info.values()]

        def sample_waypoint_from_freeespace_field(epsilon = 0.8, lb=0, ub=3):
            blockchain=w3.chain
            waypoint_list = []
            # Generate a random point fs within 95% of the arena's width and height dimensions

            # Check if the point fs is at least 0.7 units away from all free space waypoints
            while len(waypoint_list) < 30:
                fs = [(random.random() - 0.5) * eval(lp['environ']['ARENADIM']) * 0.98,
                      (random.random() - 0.5) * eval(lp['environ']['ARENADIMV']) * 0.98]
                # print(lp['environ']['ARENADIM'], lp['environ']['ARENADIMV'], fs,
                #       [kf_pred_pos[0][0], kf_pred_pos[1][0]],
                #       math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]], fs))
                if lb < math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]], fs) < ub:
                    waypoint_list.append(fs)
                # print("this distance: ", kf_pred_pos, fs, math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]],fs))
                # print("this distance: ", kf_pred_pos, fs, math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]],fs))
            waypoint_values = blockchain[-1].state.getGPValues(waypoint_list, False, l=0.2) #make sure everyone has chance to be selected
            wv_regulated= [wv+0.1 for wv in waypoint_values]
            total = sum(wv_regulated)
            wp_probabilities = [n / total for n in wv_regulated]
            if random.random()<epsilon:
                sampled_waypoint = waypoint_list[wp_probabilities.index(max(wp_probabilities))]
            else:
                sampled_waypoint = random.choices(waypoint_list, weights=wp_probabilities, k=1)[0]
            return sampled_waypoint, wv_regulated

        ##############################
        ##### STATE-MACHINE STEP #####
        ##############################

        #########################################################################################################
        #### State::EVERY
        #########################################################################################################

        # Perform submodules step
        for module in [erb, odo]:
            module.step()

        # Perform clock steps
        for clock in clocks.values():
            clock.time.step()


        if clocks['peering'].query():
            peering()



        w3.step()

        if clocks['log_pos_error'].query():
            blockchain = w3.chain
            logs['self_pos_error'].log([nav.slam_xEst[0][0],nav.slam_xEst[1][0]]+robot.position.get_position()[0:2]+[math.dist([nav.slam_xEst[0][0],nav.slam_xEst[1][0]], robot.position.get_position()[0:2]),blockchain[-1].timestamp])

        if clocks['save_points'].query():
            blockchain = w3.chain
            name = 'current_points.pkl'
            with open(f"{experimentFolder}/logs/{me.id}/{name}", "wb") as file:
                pickle.dump(
                    (blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports, blockchain[-1].state.task_tree.root.get_child("TaskNode2").reports, blockchain[-1].state.actionHistory, blockchain[-1].state.confirmedPoints), file)

        # Update blockchain state on the robot C++ object
        robot.variables.set_attribute("block", str(w3.get_block('last').height))
        robot.variables.set_attribute("block_hash", str(w3.get_block('last').hash))
        robot.variables.set_attribute("state_hash", str(w3.get_block('last').state.state_hash))

        # Get perfect position if at nest
        if robot.variables.get_attribute("at") == "cache":
            odo.setPosition()

        # report confirmation
        if txs['report'] and w3.get_transaction_receipt(txs['report'].id):
            txs['report'] = None

        blockchain = w3.chain

        # find sensed positions that are far enough from confirmed points
        confirmed_lm_points = [points[0] for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if points[1] == -1]
        unconfirmed_lm_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if
                                  points[1] != -1]  # confirmed points structure includes all supplementary fields
        if len(confirmed_lm_points) >= 3:
            robot.variables.set_attribute("wanttostop", "1")

        if unconfirmed_lm_points:
            unconfirmed_ids = []
            for this_unc_point in unconfirmed_lm_points:
                closest_id, closest_distance = min(
                    ((comp_pt_id, math.dist(comp_point, this_unc_point[0])) for comp_pt_id, comp_point in
                     enumerate(lp['landmark']['positions'])),
                    key=lambda x: x[1]
                )
                if closest_distance<1.5: #not record malicious proposals
                    unconfirmed_ids.append(closest_id)
            for closest_id in unconfirmed_ids:
                if closest_id not in num_lm_discovered_list:
                    num_lm_discovered_list.append(closest_id)
                    logs['lm_discovery'].log(
                        lp['landmark']['positions'][closest_id] + [closest_id, blockchain[-1].timestamp])

        if len(confirmed_lm_points) - num_lm_confirmed > 0:
            for new_lm_id in range(len(confirmed_lm_points) - num_lm_confirmed):
                this_confirmed_pt = confirmed_lm_points[-(new_lm_id + 1)]
                closest_distance = min(
                    (math.dist(comp_point, this_confirmed_pt) for comp_point in lp['landmark']['positions']),
                    key=lambda x: x
                )
                logs['lm_confirm'].log(this_confirmed_pt + [closest_distance, blockchain[-1].timestamp])
                num_lm_confirmed += 1

        #########################################################################################################
        #### State::IDLE
        #########################################################################################################
        if fsm.query(States.IDLE):
            # speed test, drive straight for 5 steps
            if counters['velocity_test'] < 6:
                if counters['velocity_test'] == 0:
                    previous_pos = robot.position.get_position()[0:2]
                    robot.epuck_wheels.set_speed(navSpeed / 2, navSpeed / 2)
                    counters['velocity_test'] += 1
                else:
                    curpos = robot.position.get_position()[0:2]
                    my_speed += math.dist(curpos, previous_pos) * 0.2
                    previous_pos = curpos
                    print("Robot: ",me.id, " ", counters['velocity_test'], 'my speed: ', my_speed)
                counters['velocity_test'] += 1
            elif counters['rotation_test'] < 6:
                if counters['rotation_test'] == 0:
                    previous_ori = robot.position.get_orientation()
                    robot.epuck_wheels.set_speed(navSpeed / 2, -navSpeed / 2)
                    counters['rotation_test'] += 1
                else:
                    cur_ori = robot.position.get_orientation()
                    my_rot_speed += pi_2_pi(cur_ori - previous_ori) * 0.2
                    previous_ori = cur_ori
                    print("Robot: ",me.id, " ", counters['rotation_test'], 'my max rotation speed: ', my_rot_speed)
                    counters['rotation_test'] += 1
            else:
                # Update KF's initial state and cmd->state transition matrix
                my_speed = 0.00688498982754996  # hardcoded speed in a specific simulator
                robotID = ''.join(c for c in robot.variables.get_id() if c.isdigit())
                if robot.variables.get_attribute("type") == 'malicious':
                    fsm.setState(States.Pending)
                    print(robotID, 'switched to Malicious')
                else:
                    fsm.setState(States.SCOUT, message="Initial planning")
                    print(robotID, 'switched to scout')
                robot.epuck_wheels.set_speed(0, 0)
                previous_pos = robot.position.get_position()[0:2]
                previous_ori = robot.position.get_orientation()
                # init kalman filter state as ground truth location
                nav.set_kfstate(previous_pos, previous_ori)
                print("Robot: ",me.id, " current balance:")

        #########################################################################################################
        #### State::SCOUT
        #########################################################################################################

        elif fsm.query(States.SCOUT):


            rw.step()
            l_speed, r_speed = rw.get_current_speed()
            curpos = robot.position.get_position()[0:2]
            curori = robot.position.get_orientation()
            ud = [[math.dist(curpos, previous_pos)], [pi_2_pi(curori - previous_ori)]]
            relative_lm_pose, self_pose, abs_lm_pose, noisy_dxdy, see_lm_inrange, relative_pose_for_nav, closest_lm_distance = sensing(withnoise=True)
            useful_lm_pose = []
            for this_pose in relative_pose_for_nav:
                useful_lm_pose.append(this_pose)

            kf_pred_pos = nav.fwd_cycle_kf(ud, useful_lm_pose, self_pose)  # with pose measure

            # check unverified clusters:
            blockchain = w3.chain

            # find sensed points that are far enough from confirmed points
            confirmed_lm_points = [points[0] for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if points[1] == -1]
            unconfirmed_lm_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if
                                      points[1] != -1]  # confirmed points structure includes all supplementary fields
            unconfirmed_freeespace_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode2").reports if
                                      points[1] != -1]  # confirmed points structure includes all supplementary fields

            if len(confirmed_lm_points) >=3:
                robot.variables.set_attribute("wanttostop", "1")




            if len(confirmed_lm_points)-num_lm_confirmed>0:
                for new_lm_id in range(len(confirmed_lm_points)-num_lm_confirmed):
                    this_confirmed_pt = confirmed_lm_points[-(new_lm_id+1)]
                    closest_distance = min(
                        (math.dist(comp_point, this_confirmed_pt) for comp_point in lp['landmark']['positions']),
                        key=lambda x: x
                    )
                    logs['lm_confirm'].log(this_confirmed_pt+[closest_distance, blockchain[-1].timestamp])
                    num_lm_confirmed+=1
            # calculate cluster center here
            for this_lm_g_pose in confirmed_lm_points:
                if abs_lm_pose:
                    closest_idx, closest_distance = min(
                        ((comp_point[2], math.dist(comp_point[:2], this_lm_g_pose)) for comp_point in abs_lm_pose),
                        key=lambda x: x[1]
                    )
                    if closest_distance < 0.3:
                        nav.set_lm_frozen(closest_idx,
                                          this_lm_g_pose)  # set LM location frozen to help self-location

            unconfirmed_lm_centers,_,  unconfirmed_lm_centers_ver = find_cluster_center(unconfirmed_lm_points)
            unconfirmed_freeespace_centers,_,  unconfirmed_freeespace_centers_ver = find_cluster_center(unconfirmed_freeespace_points)

            new_actions_count = len(blockchain[-1].state.actionHistory) - len(
                state_reward_history)  # update multi-arm bandit estimator

            if new_actions_count > 0:
                for action in blockchain[-1].state.actionHistory[-new_actions_count:]:
                    agent_id, action_chosen, reward, time = action
                    if agent_id == me.id:
                        eg_agent.update_estimation(action_chosen, reward)
                        my_action_count[action_chosen] += 1
                        print("Robot: ",me.id, " update action estimate: ", action_chosen, reward)
                state_reward_history = blockchain[-1].state.actionHistory

            if not txs['report']:
                if lp['generic']['load_eaparam'] == 0:
                    if ((sum(my_action_count) < 5 and (my_action_count[0]-my_action_count[1]))>=3) or (
                            any(my_ac == 0 for my_ac in my_action_count) and (my_action_count[0]-my_action_count[1])>=3):  # force exploration
                        this_action = my_action_count.index(min(my_action_count))
                        print("Robot: ",me.id, " unsufficient exploration, choose action: ", this_action)
                    else:
                        this_action = eg_agent.choose_action()
                else:
                    this_action = eg_agent.choose_action()

                if int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                    this_action=random.choice([0,1])

                if this_action == 0:  # verify or explorer landmarks
                    if unconfirmed_lm_centers:
                        point_to_verify = unconfirmed_lm_centers[0]
                        point_to_discover = []
                    else:
                        sampled_waypoint, waypoint_values = sample_waypoint_from_freeespace_field()

                        #print("sampled wp: ", sampled_waypoint, " wp values: ", waypoint_values)
                        point_to_discover = sampled_waypoint
                        point_to_verify = []

                    fsm.setState(States.LANDMARK, message="Goto landmark")
                    print("Robot: ",me.id," enters verification state, try to verify: ", point_to_verify, point_to_discover)
                    wb, fb = get_full_balance()
                    logs['balance'].log([wb, fb, blockchain[-1].timestamp])
                    logs['action_estimates'].log(eg_agent.getState()+[blockchain[-1].timestamp])
                elif this_action == 1:  # explorer unseen spaces
                    point_to_discover, _ = sample_waypoint_from_freeespace_field()
                    fsm.setState(States.UNSEEN, message="Goto explore free space")
                    wb, fb = get_full_balance()
                    logs['balance'].log([wb, fb, blockchain[-1].timestamp])
                    logs['action_estimates'].log(eg_agent.getState()+[blockchain[-1].timestamp])
                    print("Robot: ",me.id, " enters exploration state, goto virtual destination: ", point_to_discover)

            previous_pos = curpos
            previous_ori = curori


        elif fsm.query(States.LANDMARK):

            if point_to_verify:
                nav.navigate_with_obstacle_avoidance(target=point_to_verify)
                # rw.step()
                curpos = robot.position.get_position()[0:2]
                curori = robot.position.get_orientation()
                ud = [[math.dist(curpos, previous_pos)], [pi_2_pi(curori - previous_ori)]]
                relative_lm_pose, self_pose, abs_lm_pose, noisy_dxdy, see_lm_inrange, relative_pose_for_nav, closest_lm_distance  = sensing(withnoise=True)
                useful_lm_pose = []
                for this_pose in relative_pose_for_nav:
                    if this_pose[2] in nav.slam_frozen_lm_id:
                        useful_lm_pose.append(this_pose)

                kf_pred_pos = nav.fwd_cycle_kf(ud, useful_lm_pose, self_pose)  # with pose measure
                closest_lm_distance_est = 100
                if noisy_dxdy:
                    closest_lm = min(noisy_dxdy, key=lambda lms: math.sqrt((lms[0]) ** 2 + (lms[1]) ** 2))
                    closest_lm_distance_est = math.sqrt((closest_lm[0]) ** 2 + (closest_lm[1]) ** 2)
                # print(me.id, "predict pos: ", kf_pred_pos, " actual pos: ", self_pose, relative_lm_pose)
                if math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]], point_to_verify)>1 and closest_lm_distance_est<0.5:
                    #kf overflow
                    nav.set_kfstate(curpos, curori)
                if not txs['report'] and math.dist([kf_pred_pos[0][0],kf_pred_pos[1][0]], point_to_verify) < 0.4 and closest_lm_distance_est<=lp['generic']['lmobs_range']:
                    closest_lm = min(noisy_dxdy, key=lambda lms: math.sqrt((lms[0]) ** 2 + (lms[1]) ** 2))
                    votetkt = True
                    if int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                        votetkt = False
                    print("Robot: ",me.id, " try report seen landmark:",
                          ([kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]], votetkt, 0),
                          "abs pos: ",
                          abs_lm_pose[0], "self poses: ", self_pose, kf_pred_pos)
                    txdata = {'function': 'reportLM',
                              'inputs': (
                                  [kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]], votetkt, 0)}
                    txs['report'] = Transaction(sender=me.id, data=txdata, timestamp=w3.custom_timer.time())
                    w3.send_transaction(txs['report'])
                    fsm.setState(States.SCOUT, message="verified, return scout")
                elif not txs['report']  and math.dist([kf_pred_pos[0][0],kf_pred_pos[1][0]], point_to_verify) < 0.2 and closest_lm_distance_est>lp['generic']['lmobs_range']:
                    votetkt = False
                    if int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                        votetkt = True
                    print("Robot: ",me.id, " try report against landmark:",
                          ([kf_pred_pos[0][0], kf_pred_pos[1][0]], votetkt, 0))

                    txdata = {'function': 'reportLM',
                              'inputs': (
                                  [point_to_verify[0] + (random.random() - 0.5) * 0.01,
                                   point_to_verify[1] + (random.random() - 0.5) * 0.01], votetkt, 0)}
                    txs['report'] = Transaction(sender=me.id, data=txdata, timestamp=w3.custom_timer.time())
                    w3.send_transaction(txs['report'])
                    fsm.setState(States.SCOUT, message="verified, return scout")
            elif point_to_discover:
                nav.navigate_with_obstacle_avoidance(target=point_to_discover)
                curpos = robot.position.get_position()[0:2]
                curori = robot.position.get_orientation()
                ud = [[math.dist(curpos, previous_pos)], [pi_2_pi(curori - previous_ori)]]
                relative_lm_pose, self_pose, abs_lm_pose, noisy_dxdy, see_lm_inrange, relative_pose_for_nav, closest_lm_distance  = sensing(withnoise=True)
                closest_lm_distance_est = 100
                if noisy_dxdy:
                    closest_lm = min(noisy_dxdy, key=lambda lms: math.sqrt((lms[0]) ** 2 + (lms[1]) ** 2))
                    closest_lm_distance_est = math.sqrt((closest_lm[0]) ** 2 + (closest_lm[1]) ** 2)

                useful_lm_pose = []
                for this_pose in relative_pose_for_nav:
                    if this_pose[2] in nav.slam_frozen_lm_id:
                        useful_lm_pose.append(this_pose)

                kf_pred_pos = nav.fwd_cycle_kf(ud, useful_lm_pose, self_pose)  # with pose measure
                if math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]], point_to_discover)>1 and closest_lm_distance_est<0.5:
                    #kf overflow
                    nav.set_kfstate(curpos, curori)
                if not txs['report'] and math.dist([kf_pred_pos[0][0], kf_pred_pos[1][0]],
                                                   point_to_discover) < 0.4 and closest_lm_distance_est <= \
                        lp['generic']['lmobs_range']:
                    blockchain = w3.chain
                    confirmed_lm_points = [points[0] for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if points[1] == -1]
                    unconfirmed_lm_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if
                                              points[1] != -1]
                    _, verified_pos_centres, _ = find_cluster_center(
                        unconfirmed_lm_points)
                    all_excluded_pos_centres = confirmed_lm_points+verified_pos_centres
                    new_lm= [kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]]

                    #print("pre-report loop: ", all_excluded_pos_centres)
                    if not all_excluded_pos_centres or min([math.dist(new_lm, comp_point) for comp_point in all_excluded_pos_centres])>0.7:
                        closest_lm = min(noisy_dxdy, key=lambda lms: math.sqrt((lms[0]) ** 2 + (lms[1]) ** 2))

                        votetkt = True
                        if int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                            votetkt = False
                        print("Robot: ",me.id, " tries report newly explored landmark:",
                              ([kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]], votetkt, 0),
                              "abs pos: ", abs_lm_pose[0])
                        txdata = {'function': 'reportLM', 'inputs': (
                            [kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]], votetkt, 0)}
                        txs['report'] = Transaction(sender=me.id, data=txdata, timestamp=w3.custom_timer.time())
                        w3.send_transaction(txs['report'])
                        fsm.setState(States.SCOUT, message="verified, return scout")
                    else:
                        print("Robot: ",me.id, " skips reporting landmark:",
                              ([kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]], True, 0),
                              "abs pos: ", abs_lm_pose[0])
                if noisy_dxdy:
                    point_to_discover = [kf_pred_pos[0][0] + closest_lm[0], kf_pred_pos[1][0] + closest_lm[1]]
                # check if an unverified point is available now
                blockchain = w3.chain
                unconfirmed_lm_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if points[1] != -1]
                unconfirmed_lm_centers,_, unconfirmed_lm_centers_ver = find_cluster_center(unconfirmed_lm_points)
                if unconfirmed_lm_centers:
                    point_to_verify = unconfirmed_lm_centers[0]
                    point_to_discover = []
                elif nav.get_distance_to(point_to_discover) < 0.2:
                    if random.random() < 0.5:
                        point_to_discover, _ = sample_waypoint_from_freeespace_field(epsilon=0.5, lb=0, ub = 6)
                    else:
                        fsm.setState(States.SCOUT, message="LM discovery found nothing, return scout")
            previous_pos = curpos
            previous_ori = curori

        elif fsm.query(States.UNSEEN):
            nav.navigate_with_obstacle_avoidance(target=point_to_discover)

            curpos = robot.position.get_position()[0:2]
            curori = robot.position.get_orientation()
            ud = [[math.dist(curpos, previous_pos)], [pi_2_pi(curori - previous_ori)]]
            relative_lm_pose, self_pose, abs_lm_pose, noisy_dxdy, see_lm_inrange, relative_pose_for_nav, closest_lm_distance = sensing(withnoise=True)
            useful_lm_pose = []
            for this_pose in relative_pose_for_nav:
                if this_pose[2] in nav.slam_frozen_lm_id:
                    useful_lm_pose.append(this_pose)
            kf_pred_pos = nav.fwd_cycle_kf(ud, useful_lm_pose, self_pose)  # with pose measure
            blockchain = w3.chain
            all_freeespace_points = [points[0] for points in blockchain[-1].state.task_tree.root.get_child("TaskNode2").reports]
            closest_freeespace_pt = 100
            if all_freeespace_points:
                # Define the point to compare against
                mypos = [kf_pred_pos[0][0], kf_pred_pos[1][0]]
                # Use min() with a generator expression to find the closest distance
                closest_freeespace_pt = min(math.dist(mypos, npt) for npt in all_freeespace_points)

            if not txs['report'] and not see_lm_inrange and closest_freeespace_pt > (lp['generic']['lmsensing_range']*2):
                votetkt = False
                if int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                    votetkt = True
                print("try report newly explored unseen space:",
                      ([kf_pred_pos[0][0], kf_pred_pos[1][0]], votetkt, 1))

                if not int(robotID) > lp['generic']['num_robots'] - lp['generic']['num_m']:
                    txdata = {'function': 'reportLM',
                              'inputs': ([kf_pred_pos[0][0], kf_pred_pos[1][0]], votetkt, 1)}
                else:
                    myMaliciousID = int(robotID)-(lp['generic']['num_robots'] - lp['generic']['num_m'])-1
                    txdata = {'function': 'reportLM',
                              'inputs': (
                              [lp['landmark']['m_positions'][myMaliciousID][0] + (random.random() - 0.5) * 0.01, lp['landmark']['m_positions'][myMaliciousID][1] + (random.random() - 0.5) * 0.01],
                              votetkt, 1)}

                txs['report'] = Transaction(sender=me.id, data=txdata, timestamp=w3.custom_timer.time())
                w3.send_transaction(txs['report'])
                fsm.setState(States.SCOUT, message="verified, return scout")
            if nav.get_distance_to(point_to_discover) < 0.2:
                blockchain = w3.chain

                # find sensed points that are sufficiently far from confirmed points
                unconfirmed_lm_points = [points for points in blockchain[-1].state.task_tree.root.get_child("TaskNode1").reports if
                                          points[
                                              1] != -1]  # confirmed points structure includes all supplementary fields
                unconfirmed_lm_centers, _, unconfirmed_lm_centers_ver = find_cluster_center(unconfirmed_lm_points)

                if random.random() < 0.5 and not unconfirmed_lm_centers:
                    point_to_discover, _ = sample_waypoint_from_freeespace_field(epsilon=0.5, lb=0, ub = 6)
                else:
                    fsm.setState(States.SCOUT, message="randomly exit unseen state")
            previous_pos = curpos
            previous_ori = curori

            # try to stay away from confirmed points


#########################################################################################################################
#### RESET-DESTROY STEPS ################################################################################################
#########################################################################################################################

def reset():
    pass


def destroy():
    if startFlag:
        # w3.geth.miner.stop()
        w3.stop_mining()
        # w3.display_chain()
        txs = w3.get_all_transactions()
        if len(txs) != len(set([tx.id for tx in txs])):
            print(f'REPEATED TRANSACTIONS ON CHAIN: #{len(txs) - len(set([tx.id for tx in txs]))}')

        for key, value in w3.sc.state.items():
            print(f"{key}: {value}")

        name = 'sc.csv'
        header = ['TIMESTAMP', 'BLOCK', 'HASH', 'PHASH', 'BALANCE', 'TX_COUNT']
        logs['sc'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID=me.id)

        name = 'firm.csv'
        header = ['TSTART', 'FC', 'Q', 'C', 'MC', 'TC', 'ATC', 'PROFIT']
        logs['firm'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID=me.id)

        name = 'epoch.csv'
        header = ['RESOURCE_ID', 'NUMBER', 'BSTART', 'Q', 'TC', 'ATC', 'price', 'robots', 'TQ', 'AATC', 'AP']
        logs['epoch'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID=me.id)

        name = 'block.csv'
        header = ['TELAPSED', 'TIMESTAMP', 'BLOCK', 'HASH', 'PHASH', 'DIFF', 'TDIFF', 'SIZE', 'TXS', 'UNC', 'PENDING',
                  'QUEUED']
        logs['block'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID=me.id)

        name = 'source_pos.csv'
        header = ['x','y']
        logs['source_pos'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID=me.id)

        for lm_idx, lm_loc in enumerate(lp['landmark']['positions']):
            logs['source_pos'].log(lm_loc)
        # Log the result of the each trip performed by robot
        for trip in tripList:
            if trip.finished:
                logs['firm'].log([*str(trip).split()])

        # Log each epoch over the operation of the swarm
        epochs = w3.sc.getAllEpochs()
        for resource_id, resource_epochs in epochs.items():
            for epoch in resource_epochs:
                logs['epoch'].log([resource_id] + [str(x).replace(" ", "") for x in epoch.values()])

        # Log each block over the operation of the swarm
        blockchain = w3.chain
        for block in blockchain:
            logs['block'].log(
                [w3.custom_timer.time() - block.timestamp,
                 block.timestamp,
                 block.height,
                 block.hash,
                 block.parent_hash,
                 block.difficulty,
                 block.total_difficulty,
                 sys.getsizeof(block) / 1024,
                 len(block.data),
                 0
                 ])

            logs['sc'].log(
                [block.timestamp,
                 block.height,
                 block.hash,
                 block.parent_hash,
                 block.state.balances.get(me.id, 0),
                 block.state.n
                 ])
        # remove all Gaussian process data buffer
        for filename in os.listdir('./gpdatabuffer'):
            file_path = os.path.join('./gpdatabuffer', filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path)  # Remove the file
            except Exception as e:
                print(f'Failed to delete {file_path}. Reason: {e}')

    print('Killed robot ' + me.id)


#########################################################################################################################
#########################################################################################################################
#########################################################################################################################


def getEnodes():
    return [peer['enode'] for peer in w3.geth.admin.peers()]


def getEnodeById(__id, gethEnodes=None):
    if not gethEnodes:
        gethEnodes = getEnodes()

    for enode in gethEnodes:
        if readEnode(enode, output='id') == __id:
            return enode


def getIds(__enodes=None):
    if __enodes:
        return [enode.split('@', 2)[1].split(':', 2)[0].split('.')[-1] for enode in __enodes]
    else:
        return [enode.split('@', 2)[1].split(':', 2)[0].split('.')[-1] for enode in getEnodes()]


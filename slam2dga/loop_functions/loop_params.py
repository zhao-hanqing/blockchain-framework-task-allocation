#!/usr/bin/env python3
# Experimental parameters used in loop and qt_user functions
# Reqs: parameter dictionary is named "params"

import math
import os




# All environment variables
params = dict()
params['environ'] = os.environ

# Generic parameters; include adaptations of environment variables
params['generic'] = dict()
params['generic']['time_limit'] = float(os.environ["TIMELIMIT"]) * 60
params['generic']['arena_size'] = float(os.environ["ARENADIM"])
params['generic']['num_robots'] = int(os.environ["NUMROBOTS"])
params['generic']['seed']       = 358 # None for randomgen
params['generic']['tps'] = eval(os.environ["TPS"])
params['generic']['num_1'] = eval(os.environ["NUM1"])
params['generic']['num_2'] = eval(os.environ["NUM2"])
params['generic']['num_m'] = eval(os.environ["NUMM"])
params['generic']['density'] = eval(os.environ["DENSITY"])
params['generic']['arena_dim'] = eval(os.environ["ARENADIM"])
params['generic']['rab_range'] = eval(os.environ["RABRANGE"])
params['generic']['block_period'] = eval(os.environ["BLOCKPERIOD"])
params['generic']['max_workers'] = eval(os.environ["MAXWORKERS"])
params['generic']['load_eaparam'] = eval(os.environ["LOADEAPARAMS"])
params['generic']['num_landmark'] = eval(os.environ["NUMLANDMARKS"])
params['generic']['home_position'] = [0,0]
params['generic']['sp_noise'] = 0.3
params['generic']['lmobs_noise'] = 0.2
params['generic']['lmobs_range'] = 0.5
params['generic']['lmsensing_range'] = 1.5
params['generic']['inter_unseen_dist'] = 1.2


# Parameters for marketplace
params['market'] = dict()
params['market']['x'] = -0.25*params['generic']['arena_size']
params['market']['y'] = -0.25*params['generic']['arena_size']
# params['market']['r'] = 0.15 *params['generic']['arena_size'] * math.sqrt(1/math.pi)
params['market']['r'] = 2.5 * 0.073/2 * math.sqrt(params['generic']['num_robots'])

# Parameters for cache
params['cache'] = dict()
params['cache']['x'] = params['market']['x']
params['cache']['y'] = params['market']['y']
params['cache']['r'] = 0.07 +params['market']['r']

params['patches'] = dict()
# params['patches']['distribution'] = 'uniform' 
# params['patches']['distribution'] = 'patchy'
# params['patches']['hotspots']      = [{'x_mu': 0.25 * params['generic']['arena_size'], 
# 									     'y_mu': 0.25 * params['generic']['arena_size'], 
# 									     'x_sg': 0.15 * params['generic']['arena_size'], 
# 									     'y_sg': 0.15 * params['generic']['arena_size']}]


params['patches']['distribution'] = 'fixed' 

params['patches']['x'] = [ 0.25]
params['patches']['y'] = [ 0.25]

# params['patches']['x'] = [ -0.25, 0.25]
# params['patches']['y'] = [ 0.25, -0.25]

params['patches']['respawn']   = False
params['patches']['known']     = True
params['patches']['radius']    = 0.35
params['patches']['qtty_min']  = 50
params['patches']['qtty_max']  = 50
params['patches']['dist_min']  = 1.5 * params['cache']['r'] 
params['patches']['dist_max']  = 5 * params['cache']['r']

params['patches']['qualities'] = {'red', 'green' , 'blue', 'yellow'}
params['patches']['counts'] = {'red': 0, 'green': 0 , 'blue': 1, 'yellow': 0}
params['patches']['radii']  = {k: params['patches']['radius'] for k in params['patches']['qualities']}

# Parameters for resource economy
params['patches']['utility']     = {'red': 1, 'green':  100, 'blue': 200, 'yellow': 250}
params['patches']['forage_rate'] = {'red': 10, 'green':  8, 'blue': 1, 'yellow': 1.5}
params['patches']['regen_rate']  = {'red': 1, 'green':  3, 'blue': 8, 'yellow': 6}

params['patches']['dec_returns'] = dict()
params['patches']['dec_returns']['func']   = 'linear'                       # constant, linear or logarithmic decreasing returns
params['patches']['dec_returns']['thresh'] = params['patches']['qtty_max']  # qqty of resource before dec returns starts
params['patches']['dec_returns']['slope']  = 1

params['patches']['dec_returns']['func_robot']  = 'linear'                  # seconds each resource is slower than previous
params['patches']['dec_returns']['slope_robot'] = 1.5
params['patches']['forage_together'] = True

# params['patches']['dec_returns']['func_robot']  = 'exp'                  # seconds each resource is slower than previous
# params['patches']['dec_returns']['slope_robot'] = 3

# params['patches']['area_percent'] = 0.005 * (10/generic_params['num_robots'])
# params['patches']['radius']    = params['generic']['arena_size']  * math.sqrt(resource_params['area_percent']/math.pi) 

# params['patches']['radius']    = params['generic']['arena_size']  * math.sqrt(resource_params['area_percent']/math.pi) 
# params['patches']['abundancy']    = 0.03
# params['patches']['frequency'] = {'red': 0.25, 'green': 0.25 , 'blue': 0.25, 'yellow': 0.25}

# Parameters for the economy
params['economy'] = dict()
params['economy']['efficiency_distribution'] = 'linear' 
params['economy']['efficiency_best'] = 1  # amps/second of best robot
params['economy']['efficiency_step'] = 0  # amps/second increase per robot ID

# Initialize the files which store QT_draw information 
params['files'] = dict()
params['files']['patches'] = 'loop_functions/patches.txt'

def load(path):
    fs_list=[]
    ffs_list = []
    if os.path.exists(path):
        with open(path,'r') as file:
            l= list(map(float,file.read().split()))
        for idx in range(params['generic']['num_landmark']):
            fs_list.append([l[idx*2],l[idx*2+1]])
        # for idy in range(params['generic']['num_2']):
        #     ffs_list.append([l[(params['generic']['num_landmark']+idy)*2],l[(params['generic']['num_landmark']+idy)*2+1]])
        print("load landmark pts: ", fs_list,  ffs_list)
        return fs_list, ffs_list
    else:
        return  [[0,0]], [[0,0]]

def load_m(path):
    fs_list=[]
    ffs_list = []
    if os.path.exists(path):
        with open(path,'r') as file:
            l= list(map(float,file.read().split()))
        for idx in range(params['generic']['num_m']):
            fs_list.append([l[idx*2],l[idx*2+1]])
        # for idy in range(params['generic']['num_2']):
        #     ffs_list.append([l[(params['generic']['num_landmark']+idy)*2],l[(params['generic']['num_landmark']+idy)*2+1]])
        print("load malicious pts: ", fs_list,  ffs_list)
        return fs_list, ffs_list
    else:
        return  [[0,0]], [[0,0]]

fs_list, ffs_list = load('loop_functions/source_pos.txt')
ml_list, _ = load_m('loop_functions/malicious_pos.txt')
params['landmark']=dict()
params['landmark']['positions'] = fs_list
params['landmark']['m_positions'] = ml_list
params['landmark']['fake_positions'] = ffs_list
params['landmark']['radius'] = 1.1

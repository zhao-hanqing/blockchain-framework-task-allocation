#!/usr/bin/env python3
import os, math
import random
if os.path.exists('loop_functions/source_pos.txt'):
    os.remove('loop_functions/source_pos.txt')
if os.path.exists('loop_functions/malicious_pos.txt'):
    os.remove('loop_functions/malicious_pos.txt')
from loop_params import params as lp

def distance(list1, list2):
    """Distance between two vectors."""
    squares = [(p-q) ** 2 for p, q in zip(list1, list2)]
    return sum(squares) ** .5




def save(path,fs_list):
    l=[]
    for fs in fs_list:
        for coor in fs:
            l.append(coor)
    with open(path,'w') as file:
        file.write(' '.join(map(str,l)))

def load(path):
    fs_list=[]
    with open(path,'r') as file:
        l= list(map(float,file.read().split()))
    for idx in range(lp['generic']['num_landmark']):
        fs_list.append([l[idx*2],l[idx*2+1]])
    return fs_list

fs_list=[]
ml_list=[]
while len(fs_list)<(lp['generic']['num_landmark']):
    print(len(fs_list))
    fs=[0,0]
    minIntSrcDist = eval(lp['environ']['INTERLMDIST'])
    interSource = False
    while not interSource:
        fs = [((random.random()-0.5)*eval(lp['environ']['ARENADIM'])*0.8),
              ((random.random() - 0.5) * eval(lp['environ']['ARENADIM'])*0.8)-(eval(lp['environ']['ARENADIMHV'])-eval(lp['environ']['ARENADIMH']))]
        interSource = True
        for pt in fs_list:
            if distance(fs,pt)<minIntSrcDist:
                interSource=False
    fs_list.append(fs)

while len(ml_list)<(lp['generic']['num_m']):
    fs = [0, 0]
    minIntSrcDist = eval(lp['environ']['INTERLMDIST'])
    interSource = False
    while not interSource:
        fs = [((random.random() - 0.5) * eval(lp['environ']['ARENADIM']) * 0.8),
              ((random.random() - 0.5) * eval(lp['environ']['ARENADIM']) * 0.8) + (eval(lp['environ']['ARENADIMHV']) - eval(lp['environ']['ARENADIMH']))]
        interSource = True
        for pt in fs_list:
            if distance(fs, pt) < minIntSrcDist:
                interSource = False
    print("ml list: ", ml_list)
    ml_list.append(fs)


save('loop_functions/source_pos.txt',fs_list)
save('loop_functions/malicious_pos.txt',ml_list)

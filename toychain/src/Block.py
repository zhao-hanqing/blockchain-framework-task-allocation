from random import randint
from toychain.src.utils import compute_hash, transaction_to_dict, shift_and_normalize, calculate_centroid, \
    euclidean_distance
from os import environ, path, makedirs, remove

import pickle
import hashlib
import toychain.src.GPClustering_sym as gpc
import math
from itertools import zip_longest
from typing import Any, Dict, List, Optional, Callable

class Block:
    """
    Class representing a block of a blockchain containing transactions
    """

    def __init__(self, height, parent_hash, data, miner_id, timestamp, difficulty, total_diff, nonce=None,
                 state_var=None, state=None):
        self.height = height
        self.number = height
        self.parent_hash = parent_hash
        self.data = data
        self.miner_id = miner_id
        self.timestamp = timestamp
        self.difficulty = difficulty
        self.total_difficulty = total_diff + difficulty

        if state:
            self.state = state
        else:
            self.state = State(state_var)

        self.nonce = nonce
        if nonce is None:
            self.nonce = randint(0, 1000)

        self.transactions_root = self.transactions_hash()
        self.hash = self.compute_block_hash()

    def compute_block_hash(self):
        """
        computes the hash of the block header
        :return: hash of the block
        """
        _list = [self.height, self.parent_hash, self.transactions_hash(), self.miner_id, self.timestamp,
                 self.difficulty,
                 self.total_difficulty, self.nonce]

        self.hash = compute_hash(_list)

        return self.hash

    def transactions_hash(self):
        """
        computes the hash of the block transactions
        :return: the hash of the transaction list
        """
        transaction_list = [transaction_to_dict(t) for t in self.data]
        self.transactions_root = compute_hash(transaction_list)
        return self.transactions_root

    def get_header_hash(self):
        header = [self.parent_hash, self.transactions_hash(), self.timestamp, self.difficulty, self.nonce]
        return compute_hash(header)

    def increase_nonce(self):  ###### POW
        self.nonce += 1

    def __repr__(self):
        """
        Translate the block object in a string object
        """
        return f"## H: {self.height}, D: {self.difficulty}, TD: {self.total_difficulty}, P: {self.miner_id}, BH: {self.hash[0:5]}, TS:{self.timestamp}, #T:{len(self.data)}, SH:{self.state.state_hash[0:5]}##"


class StateMixin:
    @property
    def getBalances(self):
        return self.balances

    @property
    def getN(self):
        return self.n

    @property
    def call(self):
        return None

    @property
    def state_variables(self):
        return {k: v for k, v in vars(self).items() if
                not (k.startswith('_') or k == 'msg' or k == 'block' or k == 'private')}

    @property
    def state(self):
        return {k: v for k, v in vars(self).items() if
                not (k.startswith('_') or k == 'msg' or k == 'block' or k == 'private')}

    @property
    def state_hash(self):
        return compute_hash(self.state.values())

    def apply_transaction(self, tx, block):

        self.msg = tx
        self.block = block

        self.balances.setdefault(tx.sender, 0)
        self.balances.setdefault(tx.receiver, 0)

        # Check sender funds
        if tx.value and self.balances[tx.sender] < tx.value:
            print("Transaction failed: No sufficient fund")
            return

        # Apply the transfer of value
        self.balances[tx.sender] -= tx.value
        self.balances[tx.receiver] += tx.value

        # Apply the other functions contained in data
        self.n += 1
        if tx.data and 'function' in tx.data and 'inputs' in tx.data:
            function = getattr(self, tx.data.get("function"))
            inputs = tx.data.get("inputs")
            try:
                function(*inputs)
            except Exception as e:
                raise e

#Task Tree structure is implemented in a simplified way to avoid pickling problems
class TaskNode:
    def __init__(self, name: str, reports: Optional[List]=None, seps: Optional[List]=None):
        self.name = name
        self.reports = [] if reports is None else reports
        self.seps = [] if seps is None else seps
        self.children: Dict[str, "TaskNode"] = {}

    def set_child(self, name: str, reports: Optional[List]=None, seps: Optional[List]=None):
        node = TaskNode(name, reports, seps)
        self.children[name] = node
        return node

    def get_child(self, name: str):
        return self.children.get(name)


class TaskTree:
    def __init__(self, task1_data, task2_data):
        self.root = TaskNode("RootNode", reports=task1_data + task2_data)
        self.root.set_child("TaskNode1", reports=task1_data)
        self.root.set_child("TaskNode2", reports=task2_data)

    def aggregate(self):
        self.root.reports = self.root.get_child("TaskNode1").reports + self.root.get_child("TaskNode2").reports


class State(StateMixin):

    def __init__(self, state_variables=None):

        if state_variables is not None:
            for var, value in state_variables.items(): setattr(self, var, value)

        else:
            self.private = {}
            self.n = 0
            self.balances = {}

            self.patches = []
            self.robots = {}
            self.epochs = {}
            self.allepochs = {}
            self.clusters = {}

            self.task_tree = TaskTree([], [])

            self.numpOpenPts = 0
            self.numnOpenPts = 0
            self.lastpInv = []
            self.lastnInv = []

            self.tokens = [int(environ["INITBALANCE"])] * int(
                environ["NUMROBOTS"])  # number of tokens are managed here (id, token)
            self.maxClusters = float(environ["MAXCLUSTERS"])
            self.maxNegClusters = float(environ["MAXNEGCLUSTERS"])
            self.saveConfirm = float(environ["SAVECONFIRMATION"])
            self.inflationCst = float(environ["INITBALANCE"]) * float(environ["NUMROBOTS"]) * (2 / 3) * 0.1
            self.totalCircAssets = float(environ["INITBALANCE"]) * float(environ["NUMROBOTS"])
            self.task_reward = int(environ["TASKREWARD"])
            self.task2count = 0
            self.frozen_threshold = 0

            self.l_cluster = 7.0
            self.l_eval = 5.0

            self.actionHistory = []
            self.confirmedPoints = []
            self.maxAHLength = 300

    def robot(self, task=-1):
        return {'task': task}

    def patch(self, x, y, qtty, util, qlty, json):
        return {
            'x': x,
            'y': y,
            'qtty': qtty,
            'util': util,
            'qlty': qlty,
            'json': json,
            'id': len(self.patches),
            'maxw': int(environ['MAXWORKERS']),
            'totw': 0,
            'last_assign': -1,
            'epoch': self.epoch(0, 0, [], [], [], self.linearDemand(0))
        }

    def epoch(self, number, start, Q, TC, ATC, price):
        return {
            'number': number,
            'start': start,
            'Q': Q,
            'TC': TC,
            'ATC': ATC,
            'price': price
        }

    def cluster(self, x, y, lmid, repList, depoList, status):
        return {
            'x': x,
            'y': y,
            'ID': lmid,
            'replist': repList,
            'depolist': depoList,
            'status': status
        }

    def register(self):
        self.robots[self.msg.sender] = self.robot()

    def calculateDeposit(self):
        myBalance = self.tokens[int(self.msg.sender) - 1]
        for dataPoint in self.task_tree.root.get_child("TaskNode1").reports:
            if dataPoint[2] == 0 and int(dataPoint[1]) == int(self.msg.sender):
                myBalance += dataPoint[5]
        for dataPoint in self.task_tree.root.get_child("TaskNode2").reports:
            if dataPoint[2] == 0 and int(dataPoint[1]) == int(self.msg.sender):
                myBalance += dataPoint[5]
        requiredDeposit = myBalance / self.maxClusters
        if requiredDeposit > self.tokens[int(self.msg.sender) - 1] or requiredDeposit == 0:
            return -1
        elif requiredDeposit > self.tokens[int(self.msg.sender) - 1] and 0.95 * requiredDeposit < self.tokens[
            int(self.msg.sender) - 1]:  # prevent rounding error
            return self.tokens[int(self.msg.sender) - 1]
        else:
            return requiredDeposit

    def getMyBalance(self):
        return self.tokens[int(self.msg.sender) - 1]

    def spendMoney(self, amount):
        if self.tokens[int(self.msg.sender) - 1] >= amount:
            self.tokens[int(self.msg.sender) - 1] -= amount

    def returnMoney(self, receiver, amount):
        self.tokens[int(receiver) - 1] += amount

    def reportLM(self, data_pt, vote, intention):
        deposit = self.calculateDeposit()
        if deposit != -1 and intention != 2:  # if intention == 2, agent send a unseen space holder
            self.spendMoney(deposit)
            if vote == True:
                self.task_tree.root.get_child("TaskNode1").reports.append([data_pt, self.msg.sender, 0, -1, 0, deposit,
                                   intention])  # Point status = 0: an open point, CLuster status = -1: cluster un assigned, init instantaneous reward = 0
                self.task_tree.root.get_child("TaskNode1").seps.append([])
            else:
                self.task_tree.root.get_child("TaskNode2").reports.append([data_pt, self.msg.sender, 0, -1, 0, deposit, intention])
                self.task_tree.root.get_child("TaskNode2").seps.append([])
            self.updateCLuster(vote)
        elif intention == 2:
            self.task_tree.root.get_child("TaskNode2").reports.append([data_pt, self.msg.sender, 0, -1, 0, 0, intention])
            self.task_tree.root.get_child("TaskNode2").seps.append([])
            self.updateCLuster(vote)
        else:
            print("report failed, no enough assets: ", deposit, self.tokens[int(self.msg.sender) - 1])

    def hash_MyStates(self):
        # Create a SHA-1 hash object
        hash_obj = hashlib.sha1()
        # Iterate over each vector and update the hash object
        for vec in self.task_tree.root.get_child("TaskNode1").reports:
            # Convert vector to string and encode to bytes, then update hash
            for item in vec:
                hash_obj.update(str(item).encode())
        for vec in self.task_tree.root.get_child("TaskNode2").reports:
            # Convert vector to string and encode to bytes, then update hash
            for item in vec:
                hash_obj.update(str(item).encode())

        # Return the hexadecimal digest of the hash
        return hash_obj.hexdigest()

    def dataFileName(self, which):
        return './gpdatabuffer/gp' + '_' + str(len(self.task_tree.root.get_child("TaskNode2").reports)) + '_' + str(len(self.task_tree.root.get_child("TaskNode1").reports)) + '_' + str(
            int(which)) + '_' + str(self.hash_MyStates()) + '.pkl'

    def getInverseMatrix(self, data, v0, v1, l):
        C = gpc.getCovarianceMatrix(data, v0, v1, l)
        return gpc.getInversionMatrix(data, C)

    def update_seps(self, seps, data, offset, gpc, inv):
        for i, sep in enumerate(seps):
            if not sep:
                seps[i] = gpc.getEquilibriumPoint(data[:, offset + i], data, inv, 0.1, 1000)

    def check_and_update(self, seps, other_seps, data, offset, other_offset, gpc, inv, l_c):
        # update all sep points with regard to recent changes in seps
        modified_seps = []
        for i, sep in enumerate(seps):
            if not sep:
                seps[i] = gpc.getEquilibriumPoint(data[:, offset + i], data, inv, 0.1, 1000, l=l_c)
            modified_seps.append(i)
        for i in modified_seps:
            for j in range(len(other_seps)):
                if math.sqrt((data[0, offset + i] - data[0, other_offset + j]) ** 2 + (
                        data[1, offset + i] - data[1, other_offset + j]) ** 2) < 1:
                    other_seps[j] = gpc.getEquilibriumPoint(data[:, other_offset + j], data, inv, 0.1, 1000, l=l_c)
            for j in range(len(seps)):
                if i != j and math.sqrt((data[0, offset + i] - data[0, offset + j]) ** 2 + (
                        data[1, offset + i] - data[1, offset + j]) ** 2) < 1:
                    seps[j] = gpc.getEquilibriumPoint(data[:, offset + j], data, inv, 0.1, 1000, l=l_c)

    def process_seps(self, which, gpc, data, inv, l):
        self.check_and_update(self.task_tree.root.get_child("TaskNode1").seps, self.task_tree.root.get_child("TaskNode2").seps, data, 0, len(self.task_tree.root.get_child("TaskNode1").seps), gpc, inv, l_c=l)
        self.check_and_update(self.task_tree.root.get_child("TaskNode2").seps, self.task_tree.root.get_child("TaskNode1").seps, data, len(self.task_tree.root.get_child("TaskNode1").seps), 0, gpc, inv, l_c=l)

    def getGPValues(self, pointlist, which, l):
        import sympy as sp

        if which:
            if self.task_tree.root.get_child("TaskNode1").reports:
                rData = sp.Matrix([this_pt[0] for this_pt in self.task_tree.root.get_child("TaskNode1").reports]).T
            else:
                rData = []
        else:
            if self.task_tree.root.get_child("TaskNode2").reports:
                rData = sp.Matrix([this_pt[0] for this_pt in self.task_tree.root.get_child("TaskNode2").reports]).T
            else:
                rData = []
        value_list = []

        if rData:
            for this_pt in pointlist:
                this_inv = self.getInverseMatrix(rData, v0=1, v1=0, l=l)
                # print('value east: ', rData, this_pt, gpc.varianceFunction(this_pt, rData, this_inv,l=l))
                value_list.append(gpc.varianceFunction(this_pt, rData, this_inv, v0=1, v1=0, l=l))
        else:
            for _ in pointlist:
                value_list.append(1.0)
        return value_list

    def updateCLuster(self, which):
        import sympy as sp
        results = []
        dataFilename = self.dataFileName(which)
        needRecalculate = True
        if path.exists(dataFilename):
            # Load the content from the file
            try:
                # Attempt to load the content from the file
                with open(dataFilename, 'rb') as file:
                    self.task_tree.root.get_child("TaskNode1").reports, self.task_tree.root.get_child("TaskNode2").reports, self.task_tree.root.get_child("TaskNode1").seps, self.task_tree.root.get_child("TaskNode2").seps, inv, results = pickle.load(file)
                print("Pickle file buffer loaded successfully.", len(self.task_tree.root.get_child("TaskNode1").reports), len(self.task_tree.root.get_child("TaskNode2").reports), len(self.task_tree.root.get_child("TaskNode1").seps),
                      len(self.task_tree.root.get_child("TaskNode2").seps), len(results))
                needRecalculate = False
            except Exception as e:
                print(f"Failed to load pickle file: {e}")
                # Attempt to remove the file if loading failed
                remove(dataFilename)

        if needRecalculate:
            self.task_tree.aggregate()
            data = sp.Matrix([this_pt[0] for this_pt in self.task_tree.root.reports])
            data = data.T

            inv = self.getInverseMatrix(data, v0=1, v1=0, l=self.l_cluster)
            # reclustering, save separation points
            self.process_seps(which, gpc, data, inv, self.l_cluster)

            SEPs = self.task_tree.root.get_child("TaskNode1").seps + self.task_tree.root.get_child("TaskNode2").seps
            # SEPCs = [gpc.getEquilibriumPoint(data[:, i], data, inv, 0.1, 1000) for i in range(len(data[0, :]))]

            sepList, sepIndexMap = gpc.reduceSEPs(SEPs, 0.7, l_c=self.l_cluster)
            A = gpc.getAdjacencyMatrix(sp.Matrix(sepList), 0.20, 2, data, inv, l=self.l_cluster)
            sepsClusters = gpc.getSEPsClusters(A, sepList)
            results = gpc.getPointClusters(sepsClusters, sepIndexMap)

            if len(self.task_tree.root.get_child("TaskNode1").reports) + len(self.task_tree.root.get_child("TaskNode2").reports) == len(results):
                if not path.exists('./gpdatabuffer'):
                    makedirs('./gpdatabuffer')
                with open(dataFilename, "wb") as file:
                    pickle.dump((list(self.task_tree.root.get_child("TaskNode1").reports), self.task_tree.root.get_child("TaskNode2").reports, self.task_tree.root.get_child("TaskNode1").seps, self.task_tree.root.get_child("TaskNode2").seps, inv, results), file)

        lastPtReg, clusterDepositLst, task1_clustering_results, task2_clustering_results = self.applyResults(results, which)

        # uncertainty decrease for reward purpose:
        if lastPtReg:  # check if the last pose point has been registered, this part is conducted separately for two fields
            if which:
                all_pdata = [pt[0] for pt in self.task_tree.root.get_child("TaskNode1").reports[:-1]]
                if all_pdata:  # at least 2 dps
                    ppData = sp.Matrix(all_pdata).T  # only take into account confirmed points
                    previousV = gpc.varianceFunction([self.task_tree.root.get_child("TaskNode1").reports[-1][0][0], self.task_tree.root.get_child("TaskNode1").reports[-1][0][1]], ppData,
                                                     self.getInverseMatrix(ppData, v0=1, v1=0, l=self.l_eval), v0=1,
                                                     v1=0, l=self.l_eval)
                else:
                    previousV = 1
                pData = sp.Matrix([this_pt[0] for this_pt in self.task_tree.root.get_child("TaskNode1").reports]).T
                curV = gpc.varianceFunction([self.task_tree.root.get_child("TaskNode1").reports[-1][0][0], self.task_tree.root.get_child("TaskNode1").reports[-1][0][1]], pData,
                                            self.getInverseMatrix(pData, v0=1, v1=0, l=self.l_eval), v0=1, v1=0,
                                            l=self.l_eval)


                uncDes = abs(curV - previousV)
                self.task_tree.root.get_child("TaskNode1").reports[-1][4] = uncDes
            else:
                all_ndata = [this_pt[0] for this_pt in self.task_tree.root.get_child("TaskNode2").reports[:-1]]
                if len(all_ndata) > 1:  # at least 2 dps
                    pnData = sp.Matrix(all_ndata).T
                    previousV = gpc.varianceFunction([self.task_tree.root.get_child("TaskNode2").reports[-1][0][0], self.task_tree.root.get_child("TaskNode2").reports[-1][0][1]], pnData,
                                                     self.getInverseMatrix(pnData, v0=1, v1=0, l=self.l_eval), v0=1,
                                                     v1=0, l=self.l_eval)
                else:
                    previousV = 1
                nData = sp.Matrix([this_pt[0] for this_pt in self.task_tree.root.get_child("TaskNode2").reports]).T
                curV = gpc.varianceFunction([self.task_tree.root.get_child("TaskNode2").reports[-1][0][0], self.task_tree.root.get_child("TaskNode2").reports[-1][0][1]], nData,
                                            self.getInverseMatrix(nData, v0=1, v1=0, l=self.l_eval), v0=1, v1=0,
                                            l=self.l_eval)

                uncDes = abs(curV - previousV)
                self.task_tree.root.get_child("TaskNode2").reports[-1][4] = uncDes
        # perform voting game and calculate rewards:
        self.immediateFreeSpaceReward(clusterResult=results, task1_cluster_deposits=task1_clustering_results,
                                     task2_cluster_deposits=task2_clustering_results)
        self.removeRedundentLandmarkClusters(clusterResult=results, task1_cluster_deposits=task1_clustering_results,
                                                task2_cluster_deposits=task2_clustering_results)
        self.votingGame(clusterDepositLst)


        if len(self.actionHistory) > self.maxAHLength:
            self.actionHistory = self.actionHistory[-self.maxAHLength:]

        return results

    def immediateFreeSpaceReward(self, clusterResult, task1_cluster_deposits, task2_cluster_deposits):
        while len(task1_cluster_deposits) < len(task2_cluster_deposits):
            task1_cluster_deposits.append(0)
        while len(task2_cluster_deposits) < len(task1_cluster_deposits):
            task2_cluster_deposits.append(0)
        all_freeespace_clusters = []
        for idx in range(max(clusterResult) - 1):
            if task1_cluster_deposits[idx] == 0 and task2_cluster_deposits[idx] != 0:  # free space only deposit
                all_freeespace_clusters.append(idx + 1)
        # process all task 2 votes
        for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode2").reports):

            if self.task_tree.root.get_child("TaskNode2").reports[dpidx][3] in all_freeespace_clusters and self.task_tree.root.get_child("TaskNode2").reports[dpidx][5] != 0:
                print("this cluster: ", self.task_tree.root.get_child("TaskNode2").reports[dpidx][3], all_freeespace_clusters)
                self.task2count += 1
                if self.task_reward == 1:
                    bonus_reward = max(self.task_tree.root.get_child("TaskNode2").reports[dpidx][4] + 0.2, 0.2) * (
                                self.inflationCst / ((2 / 3) * float(environ["NUMROBOTS"])))
                else:
                    bonus_reward = 0
                self.totalCircAssets += bonus_reward
                print("isolate task 2 vote, returen immediate bonus: ", self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], bonus_reward,
                      self.task2count)

                self.returnMoney(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][5])
                self.task_tree.root.get_child("TaskNode2").reports[dpidx][5] = 0
                remain_bonus = self.payBackBonusToVotes(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], bonus_reward)
                self.returnMoney(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], remain_bonus)
                self.actionHistory.append(
                    [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], bonus_reward, self.block.timestamp])

    def removeRedundentLandmarkClusters(self, clusterResult, task1_cluster_deposits, task2_cluster_deposits):
        while len(task1_cluster_deposits) < len(task2_cluster_deposits):
            task1_cluster_deposits.append(0)
        while len(task2_cluster_deposits) < len(task1_cluster_deposits):
            task2_cluster_deposits.append(0)
        all_freeespace_clusters = []
        for idx in range(max(clusterResult) - 1):
            if task1_cluster_deposits[idx] == 0 and task2_cluster_deposits[idx] != 0:  # free space only deposit
                all_freeespace_clusters.append(idx + 1)
        posClusters = []
        posClustersDeposits = []
        for idx in range(max(clusterResult) - 1):
            if task1_cluster_deposits[idx] != 0:
                posClusters.append(idx + 1)
                posClustersDeposits.append(task1_cluster_deposits[idx] + task2_cluster_deposits[idx])
        cluster_to_keep = []
        if len(posClusters) > self.maxClusters:
            while len(cluster_to_keep) < self.maxClusters:
                cluster_to_keep.append(posClusters.index(max(posClustersDeposits)))
                posClustersDeposits[max(posClustersDeposits)] = 0
            cluster_to_keep += all_freeespace_clusters
            for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode2").reports):
                if self.task_tree.root.get_child("TaskNode2").reports[dpidx][1] != -1 and self.task_tree.root.get_child("TaskNode2").reports[dpidx][2] == 0 and self.task_tree.root.get_child("TaskNode2").reports[dpidx][5] != 0 and \
                        self.task_tree.root.get_child("TaskNode2").reports[dpidx][3] not in cluster_to_keep:
                    self.task_tree.root.get_child("TaskNode2").reports[dpidx][2] = 4
                    self.returnMoney(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][5])
            for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode1").reports):
                if self.task_tree.root.get_child("TaskNode1").reports[dpidx][1] != -1 and self.task_tree.root.get_child("TaskNode1").reports[dpidx][2] == 0 and self.task_tree.root.get_child("TaskNode1").reports[dpidx][
                    3] not in cluster_to_keep:
                    self.task_tree.root.get_child("TaskNode1").reports[dpidx][2] = 4
                    self.returnMoney(self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], self.task_tree.root.get_child("TaskNode1").reports[dpidx][5])

    def payBackBonusToVotes(self, receiver, bonus):
        remaining_bonus = bonus
        for myVotes in range(len(self.task_tree.root.get_child("TaskNode1").reports)):
            if self.task_tree.root.get_child("TaskNode1").reports[myVotes][2] == 0 and self.task_tree.root.get_child("TaskNode1").reports[myVotes][5] > 0 and int(self.task_tree.root.get_child("TaskNode1").reports[myVotes][1]) == int(
                    receiver):
                self.task_tree.root.get_child("TaskNode1").reports[myVotes][5] += (bonus / self.maxClusters)
                remaining_bonus -= (bonus / self.maxClusters)
                print("payback remain bonus to: ", (bonus / self.maxClusters), self.task_tree.root.get_child("TaskNode1").reports[myVotes])
        for myVotes in range(len(self.task_tree.root.get_child("TaskNode2").reports)):
            if self.task_tree.root.get_child("TaskNode2").reports[myVotes][2] == 0 and self.task_tree.root.get_child("TaskNode2").reports[myVotes][5] > 0 and int(self.task_tree.root.get_child("TaskNode2").reports[myVotes][1]) == int(
                    receiver):
                self.task_tree.root.get_child("TaskNode2").reports[myVotes][5] += (bonus / self.maxClusters)
                remaining_bonus -= (bonus / self.maxClusters)
                print("payback remain bonus to: ", self.task_tree.root.get_child("TaskNode2").reports[myVotes])
        print("distribute remain bonus: ", bonus, remaining_bonus)
        return remaining_bonus

    def postClusteringProcessing(self, targetData, clusterResult, knownClusters, which):
        #print("process beginning: ", targetData, clusterResult, knownClusters, which)
        seen_ids = set()
        result = []
        singleSepresult = []
        last_pt_reg = False
        singleClusterDeposits = []

        if clusterResult:
            singleClusterDeposits = [0] * max(clusterResult)
        isolate_freeespace_vote_count = 0
        for record in targetData:
            if record[1] != -1 and record[5] == 0:
                isolate_freeespace_vote_count += 1
        freeespace_wp_count = 0
        for idx, record in enumerate(targetData):
            if record[2] == 0 and (record[1], clusterResult[idx]) not in seen_ids and record[1] != -1 and clusterResult[
                idx] not in knownClusters and record[5] != 0:  # sender, new cluster id
                seen_ids.add((record[1], clusterResult[idx]))
                record[3] = clusterResult[idx]  # assign cluster result
                result.append(record)
                if which:
                    singleSepresult.append(self.task_tree.root.get_child("TaskNode1").seps[idx])
                else:
                    singleSepresult.append(self.task_tree.root.get_child("TaskNode2").seps[idx])
                singleClusterDeposits[clusterResult[idx] - 1] += record[5]
                if idx == len(targetData) - 1:
                    last_pt_reg = True
            elif record[2] == 0 and (record[1], clusterResult[idx]) in seen_ids and record[1] != -1 and record[
                5] != 0:  # sender has already a non-zero deposit open report in this cluster
                self.returnMoney(record[1], record[5])
            elif record[1] != -1 and clusterResult[idx] in knownClusters and record[
                2] == 0:  # sender report in a confirmed cluster
                self.returnMoney(record[1], record[5])
            elif record[1] == -1:  # placeholder of existing knowledge
                record[3] = clusterResult[idx]  # assign cluster result
                result.append(record)
                if which:
                    singleSepresult.append(self.task_tree.root.get_child("TaskNode1").seps[idx])
                else:
                    singleSepresult.append(self.task_tree.root.get_child("TaskNode2").seps[idx])
            elif record[1] != -1 and record[5] == 0:  # redistributed isolated free space vote
                if freeespace_wp_count > (isolate_freeespace_vote_count - self.maxNegClusters):  # add newest neg markers
                    record[3] = clusterResult[idx]
                    result.append(record)
                    if which:
                        singleSepresult.append(self.task_tree.root.get_child("TaskNode1").seps[idx])
                    else:
                        singleSepresult.append(self.task_tree.root.get_child("TaskNode2").seps[idx])
                freeespace_wp_count += 1
        return result, singleSepresult, singleClusterDeposits, last_pt_reg

    def applyResults(self, clusterResult, which):
        knownClusters = []  # cluster IDs that are associated with existing knowledge
        self.task_tree.aggregate()
        for idx, record in enumerate(self.task_tree.root.reports):
            if record[1] == -1:  # sender = -1 means this is a confirmed placeholder
                knownClusters.append(clusterResult[idx])

        task1_clustering_result = clusterResult[:len(self.task_tree.root.get_child("TaskNode1").reports)]
        task2_clustering_result = clusterResult[len(self.task_tree.root.get_child("TaskNode1").reports):]
        task1_result, task1_sep_result, task1_cluster_deposits, last_pos_pt_reg = self.postClusteringProcessing(self.task_tree.root.get_child("TaskNode1").reports,
                                                                                                     task1_clustering_result,
                                                                                                     knownClusters,
                                                                                                     True)

        self.task_tree.root.get_child("TaskNode1").reports = task1_result
        self.task_tree.root.get_child("TaskNode1").seps = task1_sep_result

        task2_result, task2_sep_result, task2_cluster_deposits, last_freeespace_pt_reg = self.postClusteringProcessing(self.task_tree.root.get_child("TaskNode2").reports,
                                                                                                     task2_clustering_result,
                                                                                                     knownClusters,
                                                                                                     False)
        clusterDeposits = [sum(values) for values in zip_longest(task1_cluster_deposits, task2_cluster_deposits, fillvalue=0)]

        if which:
            last_pt_reg = last_pos_pt_reg
        else:
            last_pt_reg = last_freeespace_pt_reg
        self.task_tree.root.get_child("TaskNode2").reports = task2_result
        self.task_tree.root.get_child("TaskNode2").seps = task2_sep_result

        return last_pt_reg, clusterDeposits, task1_cluster_deposits, task2_cluster_deposits

    def votingGame(self, clusterDeposits):
        #For each root node cluster
        for clusterId, cd in enumerate(clusterDeposits):
            if self.frozen_threshold == 0:
                voting_threshold = (2 / 3) * self.totalCircAssets / self.maxClusters
            else:
                voting_threshold = (2 / 3) * self.totalCircAssets / self.maxClusters
            print("Cluster: ", clusterId, "voting threshold: ", cd, voting_threshold)
            if cd >= voting_threshold:
                self.frozen_threshold = 0
                self.task2count = 0
                posDeposit = 0
                posVotes = []
                for voteDp in self.task_tree.root.get_child("TaskNode1").reports:
                    if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                        posDeposit += voteDp[5]
                        posVotes.append(voteDp)
                negDeposit = 0
                negVotes = []
                for voteDp in self.task_tree.root.get_child("TaskNode2").reports:
                    if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                        negDeposit += voteDp[5]
                        negVotes.append(voteDp)
                if posDeposit >= negDeposit:  # cluster confirmed
                    # normalize all uncertainty reduction values
                    posCentroid = calculate_centroid([voteDp[0] for voteDp in posVotes],
                                                     [voteDp[5] for voteDp in posVotes])
                    distancesToCentroid = [euclidean_distance(point, posCentroid) for point in
                                           [voteDp[0] for voteDp in posVotes]]
                    invDistancesToCentroid = shift_and_normalize(
                        [max(distancesToCentroid) - v for v in distancesToCentroid])
                    totalNegDeposit = 0
                    additional_freeespace_deposit = 0
                    for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode2").reports):
                        #Process task2 reward and update task2 node data
                        if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                            if self.task_tree.root.get_child("TaskNode2").reports[dpidx][5] != 0:
                                self.task_tree.root.get_child("TaskNode2").reports[dpidx][2] = 2  # voted, losing coalition
                                totalNegDeposit += self.task_tree.root.get_child("TaskNode2").reports[dpidx][5]
                                self.actionHistory.append(
                                    [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], -self.task_tree.root.get_child("TaskNode2").reports[dpidx][5],
                                     self.block.timestamp])
                                # print("cluster confirmed, action history append (lc-a): ",
                                #       [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], -self.task_tree.root.get_child("TaskNode2").reports[dpidx][5],
                                #        self.block.timestamp])
                            else:
                                self.task_tree.root.get_child("TaskNode2").reports[dpidx][2] = 3  # vote for unseen space has been confirmed
                                # Clear all votes from this agent in pdata and ndata
                                for dataset in [self.task_tree.root.get_child("TaskNode1").reports, self.task_tree.root.get_child("TaskNode2").reports]:
                                    for dpidx1, voteDp1 in enumerate(dataset):
                                        if voteDp1[1] == self.task_tree.root.get_child("TaskNode2").reports[dpidx][1]:
                                            dataset[dpidx1][2] = 3
                                            self.tokens[int(dataset[dpidx1][1]) - 1] += dataset[dpidx1][5]
                                assets_loss = self.tokens[int(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1]) - 1] * (
                                            1 / self.maxClusters)  # reduce 1/p_r of its assets
                                additional_freeespace_deposit += assets_loss
                                self.actionHistory.append(
                                    [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], -assets_loss,
                                     self.block.timestamp])
                                self.tokens[int(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1]) - 1] -= assets_loss  # loss 1/3 assets
                                # print("cluster confirmed, action history append (lc): ",
                                #       [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], -assets_loss, self.block.timestamp])

                    vote_idx = 0
                    for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode1").reports):
                        # Process task1 reward and update task1 node data
                        if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                            self.task_tree.root.get_child("TaskNode1").reports[dpidx][2] = 1  # voted, wining coalition
                            self.tokens[int(self.task_tree.root.get_child("TaskNode1").reports[dpidx][1]) - 1] += self.task_tree.root.get_child("TaskNode1").reports[dpidx][
                                5]  # return original deposit
                            if self.task_reward == 1:
                                bonus_reward = invDistancesToCentroid[vote_idx] * (
                                    self.inflationCst)  # landmark report quality
                                bonus_reward += totalNegDeposit / len(invDistancesToCentroid)
                                bonus_reward += additional_freeespace_deposit / len(invDistancesToCentroid)
                                self.totalCircAssets += self.inflationCst * invDistancesToCentroid[vote_idx]
                            else:
                                bonus_reward = totalNegDeposit / len(invDistancesToCentroid)
                                bonus_reward += additional_freeespace_deposit / len(invDistancesToCentroid)

                            remain_bonus = self.payBackBonusToVotes(self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], bonus_reward)
                            self.returnMoney(self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], remain_bonus)
                            self.actionHistory.append(
                                [self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], self.task_tree.root.get_child("TaskNode1").reports[dpidx][6], bonus_reward, self.block.timestamp])
                            # print("cluster confirmed, action history append: ",
                            #       [self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], self.task_tree.root.get_child("TaskNode1").reports[dpidx][6], bonus_reward, self.block.timestamp])
                            vote_idx += 1
                    self.confirmedPoints.append(posCentroid)
                    if self.saveConfirm == 1:
                        self.task_tree.root.get_child("TaskNode1").reports.append(
                            [posCentroid, -1, -1, -1, 0, 0, -1])  # add a point represents the confirmed knowledge
                        self.task_tree.root.get_child("TaskNode1").seps.append([])  # to be recalculated
                elif negDeposit > posDeposit:  # cluster rejected
                    uncReduce = shift_and_normalize([voteDp[4] for voteDp in negVotes])
                    totalPosDeposit = 0
                    for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode1").reports):
                        if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                            self.task_tree.root.get_child("TaskNode1").reports[dpidx][2] = 2  # voted, losing coalition
                            totalPosDeposit += self.task_tree.root.get_child("TaskNode1").reports[dpidx][5]
                            self.actionHistory.append(
                                [self.task_tree.root.get_child("TaskNode1").reports[dpidx][1], self.task_tree.root.get_child("TaskNode1").reports[dpidx][6], -self.task_tree.root.get_child("TaskNode1").reports[dpidx][5],
                                 self.block.timestamp])
                    vote_idx = 0
                    for dpidx, voteDp in enumerate(self.task_tree.root.get_child("TaskNode2").reports):
                        if voteDp[2] == 0 and voteDp[3] == clusterId + 1:
                            self.task_tree.root.get_child("TaskNode2").reports[dpidx][2] = 1  # voted, wining coalition
                            self.tokens[int(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1]) - 1] += self.task_tree.root.get_child("TaskNode2").reports[dpidx][
                                5]  # return original deposit
                            if self.task_reward == 1:
                                bonus_reward = uncReduce[vote_idx] * (self.inflationCst)  # uncertainty reduction
                                bonus_reward += totalPosDeposit / len(uncReduce)
                                self.totalCircAssets += uncReduce[vote_idx] * self.inflationCst
                            else:
                                bonus_reward = totalPosDeposit / len(uncReduce)

                            remain_bonus = self.payBackBonusToVotes(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], bonus_reward)
                            self.returnMoney(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], remain_bonus)
                            self.tokens[int(self.task_tree.root.get_child("TaskNode2").reports[dpidx][1]) - 1] += bonus_reward
                            self.actionHistory.append(
                                [self.task_tree.root.get_child("TaskNode2").reports[dpidx][1], self.task_tree.root.get_child("TaskNode2").reports[dpidx][6], bonus_reward, self.block.timestamp])
                            vote_idx += 1

                    if self.saveConfirm == 1:
                        posCentroid = calculate_centroid([voteDp[0] for voteDp in negVotes],
                                                         [voteDp[5] for voteDp in negVotes])
                        self.task_tree.root.get_child("TaskNode2").reports.append(
                            [posCentroid, -1, -1, -1, 0, 0, -1])  # add a point represents the confirmed knowledge
                        self.task_tree.root.get_child("TaskNode2").seps.append([])  # to be recalculated
            elif self.task2count < (2 / 3) * float(environ["NUMROBOTS"]) and self.frozen_threshold == 0:
                self.frozen_threshold = ((2 / 3) * self.totalCircAssets / self.maxClusters)

    def getPatches(self):
        return self.patches

    def getMyPatch(self, id):
        if id not in self.robots:
            return None

        if self.robots[id]['task'] == -1:
            return None

        return self.patches[self.robots[id]['task']]

    def getAvailiable(self):
        for i, patch in enumerate(self.patches):
            if patch['totw'] < patch['maxw'] and patch['epoch']['number'] > patch['last_assign']:
                return True
        return False

    def getEpochs(self):
        return self.epochs, self.patches

    def getAllEpochs(self):
        return self.allepochs

    def linearDemand(self, Q):
        P = 0
        demandA = 0
        demandB = 1

        if demandB > demandA * Q:
            P = demandB - demandA * Q
        return P
# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <markdowncell>

# #Algorithm Implementation

# <codecell>



import math
import sympy as sp


dimension = 2
v2 = 1




# Step 1. Construct a variance function and compute the level value $r^* = max_{x_i} \sigma ^2 (x_i) $

# Calculate Covariance Matrix C
def zeros_matrix(rows, cols):
    return [[0]*cols for _ in range(rows)]
def kernelFunction(x1, x2, v0, v1, l):
    lx = [l for i in range(dimension)]
    c = 0.0
    for i in range(dimension):
        c += lx[i] * ((x1[i] - x2[i]) ** 2)
    c = v0 * math.exp(- 0.5 * c) + v1
    return c


# calculate the whole covariance matrix
def getCovarianceMatrix(data, v0, v1, l_c):
    size = len(data[0,:])
    C = zeros_matrix(size,size)
    for i in range(size):
        for j in range(size):
            C[i][j] = kernelFunction(data[:, i], data[:, j], v0, v1, l=l_c)
    return C


# calculate variance of dataset
def getVariance(data):
    variance = 0
    for i in range(dimension):  # Assuming 'dimension' is defined in your context
        row = data.row(i)
        mean = sum(row) / len(row)
        variance += sum([(x - mean) ** 2 for x in row]) / len(row)
    return variance

def get_matrix_minor(matrix, row, col):
    matrix = [list(x) for x in list(matrix)]
    minor = [row[:col] + row[col + 1:] for row in (list(matrix[:row]) + list(matrix[row + 1:]))]
    return minor

def get_matrix_det(matrix):
    if len(matrix) == 2:
        return matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]

    determinant = 0
    for c in range(len(matrix)):
        determinant += ((-1) ** c) * matrix[0][c] * get_matrix_det(get_matrix_minor(matrix, 0, c))
    return determinant

# calculate inversion of Matrix A

def getInversionMatrix(data, C):
    import sympy as spp
    size = len(data[0, :])
    var = getVariance(data)
    Cx = spp.Matrix(C)
    spp.Matrix([[1.0]]).inv()
    return (Cx + v2 * var * spp.eye(size)).inv()


# calculate the variance for a given point x

def matrix_multiply(A, B):
    return [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]

def scalar_multiply(scalar, matrix):
    return [[scalar * element for element in row] for row in matrix]

def transpose_matrix(matrix):
    return list(map(list, zip(*matrix)))


def varianceFunction(x, data, inv, v0, v1, l):
    import sympy as sp
    size = len(data[0,:])
    k = sp.Matrix([kernelFunction(x, data[:, i], v0, v1, l) for i in range(size)])
    result = kernelFunction(x, x, v0, v1, l) - float((k.T * inv * k)[0,0])
    return result



# Step 2. Compute Stable Equilibrium Points



# nabla_variance_function
def nablaVarianceFunction(x, data, inv, v0, v1, l):
    size = len(data[0, :])

    kernel_values = [kernelFunction(x, data[:, i], v0, v1, l) for i in range(size)]

    k = [kernel_values[i] for i in range(size)]

    # Compute nablaK_arr
    nablaK_arr = []
    for d in range(dimension):
        nablaK = [-kernel_values[i] * (x[d] - data[d, i]) for i in range(size)]
        nablaK_arr.append(nablaK)
    deltaX_arr = []
    for d in range(dimension):
        term1 = sum([nablaK_arr[d][i] * sum([inv[j,i] * k[j] for j in range(size)]) for i in range(size)])
        term2 = sum([k[i] * sum([inv[i,j] * nablaK_arr[d][j] for j in range(size)]) for i in range(size)])
        deltaX = term1 + term2
        deltaX_arr.append(deltaX)
    return deltaX_arr


# gradient descent iteration
def gradientDescentIteration(x, data, inv, ita, v0, v1, l): #l value for clustering purppse
    delta_x = nablaVarianceFunction(x, data, inv, v0, v1, l)
    return [x[i] + ita * delta_x[i] for i in range(dimension)]


def getEquilibriumPoint(x, data, inv, ita, maxIteration,l):
    x_old = x
    iteration = 1
    for i in range(maxIteration):
        x_new = gradientDescentIteration(x_old, data, inv, ita,v0=1, v1=0,l=l)

        stopFlags = [(x_new[d] - x_old[d]) / x_old[d] < 0.01 for d in range(dimension)]

        if (all(stopFlags)):
            break
        else:
            x_old = x_new
            iteration += 1
    return x_new

def getNestedEqPoints(data, inv):
    return sp.Martix([getEquilibriumPoint(data[:, i], data, inv, 0.5, 1000) for i in range(len(data[0, :]))])




def isExistInList(sepList, point, min_accepted_covariance, v0, v1, l):
    for i in range(len(sepList)):
        covariance = kernelFunction(sepList[i], point, v0,v1,l)
        if (covariance > min_accepted_covariance):
            return i
    return -1


# remove duplicate SEPs and return a reduced SEPs list
# min_accepted_covariance is set to judge if two SEPs are the same one
# if the covariance of two points are larger than the min_accepted_covariance then we determine that they are one SEP point
def reduceSEPs(seps, min_accepted_covariance,l_c):
    sepList = []
    sepIndexMap = {}
    for i in range(len(seps)):
        index = isExistInList(sepList, seps[i], min_accepted_covariance, v0=1, v1=0, l=l_c)
        if index == -1:
            index = len(sepList)
            sepList.append(seps[i])
        sepIndexMap[i] = index
    return sepList, sepIndexMap



# Step 3. Construct Adjacency Matrix A


def getGeometricDistance(x1, x2):
    d = 0.
    for i in range(dimension):
        d += (x1[i] - x2[i]) ** 2
    return math.sqrt(d)


def getAdjacencyMatrix(sepList, maxVar, pointsNumPerDistanceUnit, data, invA, l):
    n = len(sepList[:,0])
    A = [[-1 if i != j else 1 for j in range(n)] for i in range(n)]
    for i in range(n):
        for j in range(i+1,n):
            delta = sepList[i,:] - sepList[j,:]
            distance = getGeometricDistance(sepList[i,:], sepList[j,:])
            pointsNum = int(pointsNumPerDistanceUnit * distance)
            isConnected = all(
                [varianceFunction(sepList[j,:] + (m + 1) * delta / pointsNum, data, invA, v0=1, v1=0, l=l) <= maxVar for m in range(pointsNum)]
            )
            A[i][j] = A[j][i] = 1 if isConnected else 0
    return A


#Step 4. Assign cluster Labels to training data Points

def getSEPsClusters(adjacencyMatrix, sepList):
    clusters = [-1 for i in range(len(sepList))]
    clusterIndex = 1
    for i in range(len(sepList)):
        isNewCluster = True
        clusterNo = clusterIndex
        for j in range(len(sepList)):
            if adjacencyMatrix[i][j] == 1 and clusters[j] != -1:
                isNewCluster = False
                clusterNo = clusters[j]
                break
        for j in range(i, len(sepList)):
            if adjacencyMatrix[i][j] == 1:
                clusters[j] = clusterNo
        if isNewCluster:
            clusterIndex += 1

    return clusters


def getPointClusters(sepsClusters, sepIndexMap):
    clusters = [-1 for i in range(len(sepIndexMap))]
    for i in range(len(sepIndexMap)):
        clusters[i] = sepsClusters[sepIndexMap[i]]
    return clusters




def getCountNrc(references, clusters, referencesNum, clustersNum):
    size = len(clusters)
    Nrc = [[0 for c in range(clustersNum)] for r in range(referencesNum)]
    for r in range(referencesNum):
        for c in range(clustersNum):
            for i in range(size):
                if (references[i] - 1 == r and clusters[i] - 1 == c):
                    Nrc[r][c] += 1
    return Nrc


def getRE(references, clusters, referencesNum, clustersNum, Nrc):
    size = len(clusters)
    sumTmp = 0.
    for r in range(referencesNum):
        maxTmp = 0.
        for c in range(clustersNum):
            if (maxTmp < Nrc[r][c]):
                maxTmp = Nrc[r][c]
        sumTmp += maxTmp
    return 1 - sumTmp / size




def getCE(references, clusters, referencesNum, clustersNum, Nrc):
    size = len(clusters)
    sumTmp = 0.
    for c in range(clustersNum):
        maxTmp = 0.
        for r in range(referencesNum):
            if (maxTmp < Nrc[r][c]):
                maxTmp = Nrc[r][c]
        sumTmp += maxTmp
    return 1 - sumTmp / size


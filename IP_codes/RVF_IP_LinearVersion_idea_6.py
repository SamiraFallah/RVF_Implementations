#!/usr/bin/env python
# coding: utf-8

# In[1]:


'''
Created on Oct 01, 2022

@author: Samira Fallah (saf418@lehigh.edu)
'''


# In[2]:


import time
import argparse
from gurobipy import *
import math
import numpy as np


# # Insert the instance manually

# In[3]:


# numVars = 11
# numIntVars = 10
# numConsFixed = 1
# numConsVar = 1
# INTVARS = range(numIntVars)
# SLACKVARS = range(numIntVars, numVars)
# CONSVARRHS = range(numConsVar)
# CONSFIXEDRHS = range(numConsFixed)

# OBJ = [-566, -611, -506, -180, -817, -184, -585, -423, -26, -317, 0]
# MAT = {(0, 0):-62, (0, 1):-84, (0, 2):-977, (0, 3):-979, (0, 4):-874, (0, 5):-54, (0, 6):-269, (0, 7):-93, (0, 8):-881, (0, 9):-563, (0, 10):0}
# MATFixed = {(0, 0):557, (0, 1):898, (0, 2):148, (0, 3):63, (0, 4):78, (0, 5):964, (0, 6):246, (0, 7):662, (0, 8):386, (0, 9):272, (0, 10):1}
# RHS = {0:2137}
# eps = 0.5
# M = 4837
# UB_I = 1


# In[4]:


parser = argparse.ArgumentParser()
parser.add_argument("-i", "--instance", help = "specify the instance")
flags = parser.parse_args()
instance = flags.instance 
# f = open("InstancesTest_SPP/{}".format(instance), "r")
f = open("InstancesTest_Knapsack/{}".format(instance), "r")


# # Read the Knapsack instances

# In[5]:


# instance = 'KP_p-3_n-10_ins-1.dat'
# f = open("InstanceTest/{}".format(instance), "r")
content = f.read()   
cont = content.split("\n")
cont = [i.rstrip() for i in cont]

numObj = int(cont[0])
numVar = int(cont[1])
capacity = int(cont[2])

mapping = [(i, ' ') for i in ['[',']',',']]

MAT = {}
OBJ = []
for i in range(numObj):
    tmp = cont[3 + i]
    for k, v in mapping:
        tmp = tmp.replace(k, v)
    tmp = list(map(int, tmp.split())) 
    tmp.append(0)
    if i == 0:
        OBJ = [-tmp[k] for k in range(numVar+1)]
    else:
        MAT.update({(i-1, j):-tmp[j] for j in range(numVar+1)})

numConst = 1

MATFixed = {}
for i in range(numConst):
    tmp = cont[i + 3 + numObj]
    for k, v in mapping:
        tmp = tmp.replace(k, v)
    tmp = list(map(int, tmp.split())) 
    for j in range(numVar):
        MATFixed[(i, j)] = tmp[j] 
MATFixed[(0, numVar)] = 1  
RHSList = [capacity]

numVars = numVar + numConst
numIntVars = numVar    
numConsVar = numObj - 1
numConsFixed = 1
INTVARS = range(numIntVars)
SLACKVARS = range(numIntVars, numVars)
CONSVARRHS = range(numConsVar)
CONSFIXEDRHS = range(numConsFixed)

RHS = {i : RHSList[i] for i in range(0, len(RHSList))}
UB_I = 1


# # Read the SPP instances

# In[6]:


# instance = '2mis100_300D.dat'
# f = open("InstanceTest/{}".format(instance), "r")
# content = f.read()  
# cont = content.split("\n")

# numConst, numVar = int(cont[0].split()[0]), int(cont[0].split()[1])
# c1_coefs = [int(i) for i in cont[1].split()]
# c2_coefs = [int(i) for i in cont[2].split()]
# cont = cont[3:]
# listIndexes = [[int(j) - 1 for j in i.split()] for i in cont[1::2] if i != ''] 
# c1_coefs = [-i for i in c1_coefs] + [0 for i in range(numConst)]
# c2_coefs = [-i for i in c2_coefs] + [0 for i in range(numConst)]

# numVars = numVar + numConst
# numIntVars = numVar 
# numContVars = 0
# numConsVar = 1
# numConsFixed = numConst
# INTVARS = range(numIntVars)
# SLACKVARS = range(numIntVars, numVars)
# VARS = range(numVars)
# CONSVARRHS = range(numConsVar)
# CONSFIXEDRHS = range(numConsFixed)

# OBJ = c1_coefs
# MAT = {(0, i):c2_coefs[i] for i in range(numIntVars)}
# RHS = {i:1 for i in range(numConst)}

# MATFixed = {}
# for j in range(numConst):
#     for i in range(numVar + numConst):
#         MATFixed[(j, i)] = 0

# k = 0
# for j in range(numConst):
#     for i in listIndexes[j]:
#         MATFixed[(j, i)] = 1
#     listIndexes[j].append(numVar + k)
#     MATFixed[(j, numVar + k)] = 1
#     k += 1
# UB_I = 1


# In[7]:


eps = 0.5

M = {}

for i in CONSVARRHS:
    M[i] = sum(abs(MAT[(i, j)]) for j in range(numVar)) + 1


# In[8]:


timeLimit = 86400
debug_print = False
ipForU = True


# In[9]:


def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value


# In[10]:


# Generate a feasible solution to calculate the upper bound U
def generatePointForU():
    m = Model()
    m.setParam("LogToConsole", 0);
    
    if ipForU:
        intVarsInit = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
        slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    else:
        intVarsInit = m.addVars(list(INTVARS), vtype = GRB.CONTINUOUS, lb = 0, ub = UB_I, name = "integer variable")
        slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
        
    m.setObjective(sum(OBJ[i] * intVarsInit[i] for i in INTVARS), GRB.MAXIMIZE)
    
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsInit[i] for i in INTVARS) +
                    sum(MATFixed[(j, i)] * slackVarsInit[i] for i in SLACKVARS) == RHS[j])
    m.optimize()
    
    return math.ceil(m.objVal)


# In[11]:


start = time.time() 
U = generatePointForU()  


# In[12]:


# Generate a feasible solution to start with
def generateInitPoint():
    m = Model()
    m.setParam("LogToConsole", 0);
    
    intVarsInit = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsInit[i] for i in INTVARS), GRB.MINIMIZE)
    
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsInit[i] for i in INTVARS) +
                    sum(MATFixed[(j, i)] * slackVarsInit[i] for i in SLACKVARS) == RHS[j])
    m.optimize()
    
    temp = ()
    for v in intVarsInit.values():
        temp = temp + (round(changeValue(v.X)),)
    
    return temp

intVarsInit = generateInitPoint()


# In[13]:


# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP(_intVarsInit, _print=False):
    m = Model()
    m.setParam("LogToConsole", 0);
    
    intVarsStrong = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    slackVarsStrong = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) 
                   + sum(MAT[(i, j)] * intVarsStrong[j] for j in INTVARS for i in CONSVARRHS), GRB.MINIMIZE)
    
    RHSFirstObj = 0
    for i in INTVARS:
        RHSFirstObj += OBJ[i] * _intVarsInit[i]
      
    m.addConstr(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) <= RHSFirstObj)
    
    for j in CONSVARRHS:
        m.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) <= sum(MAT[(j, i)] * _intVarsInit[i] for i in INTVARS))
        
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS)  
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m.optimize()
    
    if _print and debug_print:
        with open("Results_IP_Linear_details_idea_6_{}.txt".format(instance.split('.')[0]), "a") as _filed:
            _filed.write('Solution in iteration 0' + '\n\n') 
            _filed.write("Integer Variables:\n")
            for k, v in intVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write('\n' + "Slack Variables:\n")  
            for k, v in slackVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write('-------------------------------------' + '\n')
    
    temp = ()
    for v in intVarsStrong.values():
        temp = temp + (round(changeValue(v.X)),)
    
    return temp
    
intVarsInitStrong = convertWeakToStrongNDP(intVarsInit, _print=True)


# In[14]:


initPoint = intVarsInitStrong


# In[15]:


intPartList = []


# In[16]:


allIntPartDict = {}
allIntPartDict[0] = intVarsInitStrong


# In[17]:


EF = {}

temp_ndp = ()
temp_ndp = temp_ndp + (round(sum(OBJ[j]*intVarsInitStrong[j] for j in INTVARS)),)

for k in CONSVARRHS: 
    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*intVarsInitStrong[l] for l in INTVARS)),)

EF[0] = temp_ndp


# In[18]:


def calculate_distance(point_a_key, point_b_key): 
    values_dict_a = np.array(list(EF[point_a_key]))
    values_dict_b = np.array(list(EF[point_b_key]))
    
    return np.linalg.norm(values_dict_a - values_dict_b)


# In[19]:


def generateSubIntLst(allIntPartDict, initPoint, initPointIdx, _totalIter):    
    if _totalIter <= 3:
        return allIntPartDict

    tempAllIntPartDict = allIntPartDict.copy()
    subIntPartDict = {}
    subIntPartDict[initPointIdx] = initPoint
    
    del tempAllIntPartDict[initPointIdx]
    
    for _it in range(min(int(numSubRatio*_totalIter), _iter)):
        max_min_distance = float('-inf')
        for key_all, val_all in tempAllIntPartDict.items():
            min_distance = float('inf')
            for key_subset in subIntPartDict:
                distance = calculate_distance(key_subset, key_all)
                if distance < min_distance:
                    min_distance = distance

            if min_distance > max_min_distance:
                max_min_distance = min_distance
                selected_point = key_all

        subIntPartDict[selected_point] = val_all
        del tempAllIntPartDict[selected_point]
        
    return list(subIntPartDict.values())


# In[20]:


shouldCallSubList = True
allIntPart = False
initPointIdx = 1
_iter = 0
totalIter = 1
numSubRatio = 0.5

while True:
    if shouldCallSubList:
        intPartList = generateSubIntLst(allIntPartDict, initPoint, initPointIdx, totalIter)
        allIntPart = False
    else:
        intPartList = allIntPartDict
        shouldCallSubList = True
        allIntPart = True
    
    m = Model()
    m.setParam("LogToConsole", 0);
    m.params.SolutionLimit = 1

    thetaVar = m.addVar(vtype = GRB.CONTINUOUS, name = "theta")
    intVars = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "int_var")
    slackVars = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack_var")
    alphaVars = m.addVars(len(CONSVARRHS), len(intPartList), vtype = GRB.BINARY, name = "alpha_var")
    betaVars = m.addVars(len(intPartList), vtype = GRB.BINARY, name = "beta_var")

    m.setObjective(1, GRB.MAXIMIZE)

    # theta constraint -- const 1
    for k in range(len(intPartList)):
        m.addConstr(thetaVar <= - sum(OBJ[i]*intVars[i] for i in INTVARS) +   
            (1 - betaVars[k])*sum(OBJ[j]*intPartList[k][j] for j in INTVARS) + betaVars[k]*(U+1))

    # const 2 
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(sum(MAT[(i, j)]*(intVars[j] - intPartList[k][j]) for j in INTVARS) + eps 
                                                    <= M[i]*(1 - alphaVars[(i, k)]))

    # const 3 
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(sum(MAT[(i, j)]*(intPartList[k][j] - intVars[j]) for j in INTVARS) 
                            <= M[i]*alphaVars[(i, k)])

    # const 4
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(betaVars[k] >= alphaVars[(i, k)])

    # const 5
    for k in range(len(intPartList)):
        m.addConstr(betaVars[k] <= sum(alphaVars[(i, k)] for i in CONSVARRHS))

    # const 6
    for k in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(k, j)]*intVars[j] for j in INTVARS) + 
                    sum(MATFixed[(k, j)]*slackVars[j] for j in SLACKVARS) == RHS[k])
        
    m.addConstr(thetaVar >= 0.001)    
    
    m.optimize()
              
    end = time.time()
    elapsedTime = end - start
    
    totalIter += 1
    
    if (allIntPart and (m.getAttr('Status') in [3, 4])) or elapsedTime > timeLimit:
        if allIntPart:
            print("Finished!")
        else:
            print("Finished due to time limit!")

        if allIntPart:
            with open("Results_IP_Linear_idea_6_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(totalIter))
        else:
            with open("Results_IP_Linear_idea_6_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(totalIter))
        
        break
        
    int_part = convertWeakToStrongNDP(dict((k, round(changeValue(v.X))) for k,v in intVars.items()), _print=False)
    
    temp_ndp = () 
    temp_ndp = temp_ndp + (round(sum(OBJ[j]*int_part[j] for j in INTVARS)),)
    
    for k in CONSVARRHS:
        temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*int_part[l] for l in INTVARS)),) 

    if (temp_ndp in EF.values()) and (not allIntPart):
        shouldCallSubList = False
        continue
    
    if temp_ndp not in EF.values():
        _iter += 1  
        EF[_iter] = temp_ndp 
        allIntPartDict[_iter] = int_part
        initPoint = int_part
        initPointIdx = _iter
    
        if debug_print:
            with open("Results_IP_Linear_details_idea_6_{}.txt".format(instance.split('.')[0]), "a") as _filed:
                _filed.write('Solution in iteration' + ' ' + str(_iter) + '\n\n') 
                _filed.write("Integer Variables:\n")
                for k, v in intVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
                _filed.write('\n' + "Slack Variables:\n")  
                for k, v in slackVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
                _filed.write('\n' + "Alpha Variables:\n") 
                for k, v in alphaVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
                _filed.write('\n' + "Beta Variables:\n") 
                for k, v in betaVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
                _filed.write('-------------------------------------' + '\n')


# In[23]:


# EF


# In[24]:


# len(EF)


# In[ ]:


# allIntPartDict


# In[ ]:


# elapsedTime


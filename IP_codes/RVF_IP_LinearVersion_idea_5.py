#!/usr/bin/env python
# coding: utf-8

# In[44]:


'''
Created on Oct 01, 2022

@author: Samira Fallah (saf418@lehigh.edu)
'''


# In[45]:


import time
import argparse
from gurobipy import *
import math
import random


# # Insert the instance manually

# In[46]:


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


# In[47]:


parser = argparse.ArgumentParser()
parser.add_argument("-i", "--instance", help = "specify the instance")
flags = parser.parse_args()
instance = flags.instance 
f = open("InstancesTest_SPP/{}".format(instance), "r")
# f = open("InstancesTest_Knapsack/{}".format(instance), "r")


# # Read the Knapsack instances

# In[48]:


# instance = 'KP_p-3_n-10_ins-1.dat'
# f = open("InstanceTest/{}".format(instance), "r")
# content = f.read()   
# cont = content.split("\n")
# cont = [i.rstrip() for i in cont]

# numObj = int(cont[0])
# numVar = int(cont[1])
# capacity = int(cont[2])

# mapping = [(i, ' ') for i in ['[',']',',']]

# MAT = {}
# OBJ = []
# for i in range(numObj):
#     tmp = cont[3 + i]
#     for k, v in mapping:
#         tmp = tmp.replace(k, v)
#     tmp = list(map(int, tmp.split())) 
#     tmp.append(0)
#     if i == 0:
#         OBJ = [-tmp[k] for k in range(numVar+1)]
#     else:
#         MAT.update({(i-1, j):-tmp[j] for j in range(numVar+1)})

# numConst = 1

# MATFixed = {}
# for i in range(numConst):
#     tmp = cont[i + 3 + numObj]
#     for k, v in mapping:
#         tmp = tmp.replace(k, v)
#     tmp = list(map(int, tmp.split())) 
#     for j in range(numVar):
#         MATFixed[(i, j)] = tmp[j] 
# MATFixed[(0, numVar)] = 1  
# RHSList = [capacity]

# numVars = numVar + numConst
# numIntVars = numVar    
# numConsVar = numObj - 1
# numConsFixed = 1
# INTVARS = range(numIntVars)
# SLACKVARS = range(numIntVars, numVars)
# CONSVARRHS = range(numConsVar)
# CONSFIXEDRHS = range(numConsFixed)

# RHS = {i : RHSList[i] for i in range(0, len(RHSList))}
# UB_I = 1


# # Read the SPP instances

# In[49]:


# instance = '2spp101_300D.dat'
# f = open("InstanceTest/{}".format(instance), "r")
content = f.read()  
cont = content.split("\n")

numConst, numVar = int(cont[0].split()[0]), int(cont[0].split()[1])
c1_coefs = [int(i) for i in cont[1].split()]
c2_coefs = [int(i) for i in cont[2].split()]
cont = cont[3:]
listIndexes = [[int(j) - 1 for j in i.split()] for i in cont[1::2] if i != ''] 
c1_coefs = [-i for i in c1_coefs] + [0 for i in range(numConst)]
c2_coefs = [-i for i in c2_coefs] + [0 for i in range(numConst)]

numVars = numVar + numConst
numIntVars = numVar 
numContVars = 0
numConsVar = 1
numConsFixed = numConst
INTVARS = range(numIntVars)
SLACKVARS = range(numIntVars, numVars)
VARS = range(numVars)
CONSVARRHS = range(numConsVar)
CONSFIXEDRHS = range(numConsFixed)

OBJ = c1_coefs
MAT = {(0, i):c2_coefs[i] for i in range(numIntVars)}
RHS = {i:1 for i in range(numConst)}

MATFixed = {}
for j in range(numConst):
    for i in range(numVar + numConst):
        MATFixed[(j, i)] = 0

k = 0
for j in range(numConst):
    for i in listIndexes[j]:
        MATFixed[(j, i)] = 1
    listIndexes[j].append(numVar + k)
    MATFixed[(j, numVar + k)] = 1
    k += 1
UB_I = 1


# In[50]:


eps = 0.5

M = {}

for i in CONSVARRHS:
    M[i] = sum(-MAT[(i, j)] for j in range(numVar)) + 1


# In[51]:


timeLimit = 86400
debug_print = False
ipForU = True


# In[52]:


def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value


# In[53]:


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


# In[54]:


start = time.time() 
U = generatePointForU()  


# In[55]:


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
   
    temp = {}
    for k, v in intVarsInit.items():
        temp[k] = changeValue(v.X) 
    
    return temp

intVarsInit = generateInitPoint()


# In[56]:


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
        with open("Results_IP_Linear_details_idea_5_{}.txt".format(instance.split('.')[0]), "a") as _filed:
            _filed.write('Solution in iteration 0' + '\n\n') 
            _filed.write("Integer Variables:\n")
            for k, v in intVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write('\n' + "Slack Variables:\n")  
            for k, v in slackVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write('-------------------------------------' + '\n')
    
    temp = {}
    for k, v in intVarsStrong.items():
        temp[k] = changeValue(v.X)
    
    return temp
    
intVarsInitStrong = convertWeakToStrongNDP(intVarsInit, _print=True)


# In[57]:


intPartList = []
intPartList.append(intVarsInitStrong)
intPartListOrig = []
intPartListOrig.append(intVarsInitStrong)


# In[58]:


EF = []

temp_ndp = () 
temp_ndp = temp_ndp + (round(sum(OBJ[j]*intVarsInitStrong[j] for j in INTVARS)),)

for k in CONSVARRHS:
    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*intVarsInitStrong[l] for l in INTVARS)),) 

EF.append(temp_ndp)


# In[59]:


def genSubPrimalConst():
    distinct_i_values = list(set(i for i in RHS.keys()))
    random.shuffle(distinct_i_values)
    numSubPrimal = round(primalRatio * numConst)
    selected_i_values = distinct_i_values[:numSubPrimal]
    tempCONSFIXEDRHS = selected_i_values 

    return tempCONSFIXEDRHS


# In[60]:


def findIntPart(tempRHS):
    m = Model()
    m.setParam("LogToConsole", 0);
    
    intVarsInit = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsInit[i] for i in INTVARS), GRB.MINIMIZE)
    
    for j in CONSVARRHS:
        m.addConstr(sum(MAT[(j, i)] * intVarsInit[i] for i in INTVARS) <= tempRHS[j])
    
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsInit[i] for i in INTVARS) +
                    sum(MATFixed[(j, i)] * slackVarsInit[i] for i in SLACKVARS) == RHS[j])
    m.optimize()
    
    if m.getAttr('Status') == 2:
        temp = {}
        for k, v in intVarsInit.items():
            temp[k] = round(changeValue(v.X)) 
        return temp
    else:
        return


# In[61]:


idxIntPartList = 1
thetaList = [] 

# consider all the primal for the subsequent iterations when it is True
allPrimalAfterThisIter = False

primalRatio = 0.1

while True:
#     print('allPrimalAfterThisIter', allPrimalAfterThisIter)
    
    if (allPrimalAfterThisIter):
        tempCONSFIXEDRHS = CONSFIXEDRHS
    
    else:
        random.seed(20)
        tempCONSFIXEDRHS = genSubPrimalConst()
     
    m = Model()
    m.setParam("LogToConsole", 0);
    m.params.NonConvex = 2
    
    thetaVar = m.addVar(vtype = GRB.CONTINUOUS, name = "theta")
    intVars = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "int_var")
    slackVars = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack_var")
    alphaVars = m.addVars(len(CONSVARRHS), len(intPartList), vtype = GRB.BINARY, name = "alpha_var")
    betaVars = m.addVars(len(intPartList), vtype = GRB.BINARY, name = "beta_var")

    m.setObjective(thetaVar, GRB.MAXIMIZE)

    for k in range(len(intPartList)):
        m.addConstr(thetaVar <= - sum(OBJ[i]*intVars[i] for i in INTVARS) +   
            (1- betaVars[k])*sum(OBJ[j]*intPartList[k][j] for j in INTVARS) + betaVars[k]*(U+1))

    for k in range(len(intPartList)):
        m.addConstr(thetaVar <= - sum(OBJ[i]*intVars[i] for i in INTVARS) +   
            (1- betaVars[k])*sum(OBJ[j]*intPartList[k][j] for j in INTVARS) + betaVars[k]*(U+1))
    
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(sum(MAT[(i, j)]*(intVars[j] - intPartList[k][j]) for j in INTVARS) + eps 
                                            <= M[i]*(1 - alphaVars[(i, k)]))
    
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(sum(MAT[(i, j)]*(intPartList[k][j] - intVars[j]) for j in INTVARS) 
                    <= M[i]*alphaVars[(i, k)])
    
    for k in range(len(intPartList)):
        for i in CONSVARRHS:
            m.addConstr(betaVars[k] >= alphaVars[(i, k)])
    
    for k in range(len(intPartList)):
        m.addConstr(betaVars[k] <= sum(alphaVars[(i, k)] for i in CONSVARRHS))
    
    for k in tempCONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(k, j)]*intVars[j] for j in INTVARS) + 
                sum(MATFixed[(k, j)]*slackVars[j] for j in SLACKVARS) == RHS[k])
        
    m.optimize()
    
    if debug_print:
        with open("Results_IP_Linear_details_idea_5_{}.txt".format(instance.split('.')[0]), "a") as _filed:
            _filed.write('Solution in iteration' + ' ' + str(idxIntPartList) + '\n\n') 
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
            _filed.write('\n' + "Theta: " + str(round(thetaVar.X, 2))  + '\n')
            _filed.write('-------------------------------------' + '\n')
        
    end = time.time()
    elapsedTime = end - start
    
#     print('status', m.getAttr('Status'))
    
    thetaList.append(round(thetaVar.X, 2))
    
    if ((thetaVar.X < 0.1) and (allPrimalAfterThisIter)) or elapsedTime > timeLimit:
        if thetaVar.X < 0.1:
            print("Finished!")
        else:
            print("Finished due to time limit!")
        
        if thetaVar.X < 0.1:
            with open("Results_IP_Linear_idea_5_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Theta list: ' + str(thetaList) + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations:' + str(idxIntPartList))
        else:
            with open("Results_IP_Linear_idea_5_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Theta list: ' + str(thetaList) + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations:' + str(idxIntPartList))
        
        break
    
    int_part = dict((k, round(changeValue(v.X))) for k,v in intVars.items())
    
    if allPrimalAfterThisIter == False:
        intPartList.append(int_part)
        
        tempRHS = []    
        for j in CONSVARRHS:
            tempRHS.append(sum(MAT[(j, i)] * int_part[i] for i in INTVARS)) 

        resFindIntPart = findIntPart(tempRHS)
        if resFindIntPart != None:
#             print('Found a feasible int part for the original problem!')
            int_part_org = convertWeakToStrongNDP(resFindIntPart, _print=False)
            
            temp_ndp = () 
            temp_ndp = temp_ndp + (round(sum(OBJ[j]*int_part_org[j] for j in INTVARS)),)

            for k in CONSVARRHS:
                temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*int_part_org[l] for l in INTVARS)),) 
#             print('NDP', temp_ndp)
            
            if temp_ndp not in EF:
                EF.append(temp_ndp)
                intPartListOrig.append(int_part_org)
                
    if allPrimalAfterThisIter == True:
        int_part_converted = convertWeakToStrongNDP(int_part, _print=False)
        intPartList.append(int_part_converted)
        temp_ndp = () 
        temp_ndp = temp_ndp + (round(sum(OBJ[j]*int_part_converted[j] for j in INTVARS)),)

        for k in CONSVARRHS:
            temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*int_part_converted[l] for l in INTVARS)),) 
            
#         print('NDP', temp_ndp)
        EF.append(temp_ndp)
    
    if thetaList[-1] <= 1000 and not allPrimalAfterThisIter:
        allPrimalAfterThisIter = True
        intPartList = intPartListOrig
    
    idxIntPartList += 1  
        
#     print('---------------------------------')


# In[ ]:


# idxIntPartList


# In[ ]:


# EF


# In[ ]:


# len(EF)


# In[ ]:


# 1192.000 862.000
# 1050.000 1234.000
# 1162.000 1138.000
# 1176.000 1026.000
# 1184.000 900.000
# 1172.000 1062.000
# 1168.000 1064.000
# 1126.000 1192.000
# 1140.000 1146.000
# 1138.000 1148.000
# 1058.000 1196.000


# In[ ]:


#     all_equal = True  
#     if allPrimalAfterThisIter == False:
#         for k in CONSFIXEDRHS:
#             lhs = sum(MATFixed[(k, j)] * int_part[j] for j in INTVARS) + sum(MATFixed[(k, j)] * slack_part[j] for j in SLACKVARS)
#             if lhs != RHS[k]:
#                 all_equal = False 
#                 print("Not feasible for the original problem.")
#                 break  

#         if all_equal:
#             intPartListOrig.append(int_part) 
#             print("Feasible for the original problem.")


# In[ ]:


# intPartList


# In[ ]:


# elapsedTime


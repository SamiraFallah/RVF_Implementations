#!/usr/bin/env python
# coding: utf-8

'''
Created on Oct 01, 2022

@author: Samira Fallah (saf418@lehigh.edu)
'''

import time
import argparse
from gurobipy import *
import sys
import math
import numpy as np

# adding Folder to the system path
sys.path.insert(0, '/home/saf418/ValueFunctionCode/RVFCodes/Gurobi/final_MILP')

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--instance", help = "specify the instance")
flags = parser.parse_args()
instanceName = flags.instance.split('.py')[0]
for key, val in vars(__import__(instanceName)).items():
    if key.startswith('__') and key.endswith('__'):
        continue
    vars()[key] = val

timeLimit = 14400
debug_print = False
mipForU = True

def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value

# Generate U - Upper bound on the whole VF
def generatePointForU():
    m = Model()
    m.setParam("LogToConsole", 0);
    
    if mipForU:
        intVarsInit = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    else:
        intVarsInit = m.addVars(list(INTVARS), vtype = GRB.CONTINUOUS, lb = 0, ub = UB_I, name = "integer variable")

    contVarsInit = m.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "continuous variable")
    slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsInit[i] for i in INTVARS) + sum(OBJ[i] * contVarsInit[i] for i in CONVARS),
                                                           GRB.MAXIMIZE)
    
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsInit[i] for i in INTVARS) +
                    sum(MATFixed[(j, i)] * contVarsInit[i] for i in CONVARS) +
                    sum(MATFixed[(j, i)] * slackVarsInit[i] for i in SLACKVARS) == RHS[j])
    m.optimize()
    
    return math.ceil(m.objVal)

U = generatePointForU()

# Generate a feasible solution to start with
def generateInitPoint():
    m = Model()
    m.setParam("LogToConsole", 0);
    
    intVarsInit = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    contVarsInit = m.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "continuous variable")
    slackVarsInit = m.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsInit[i] for i in INTVARS) + sum(OBJ[i] * contVarsInit[i] for i in CONVARS),
                                                           GRB.MINIMIZE)
    
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsInit[i] for i in INTVARS) +
                    sum(MATFixed[(j, i)] * contVarsInit[i] for i in CONVARS) +
                    sum(MATFixed[(j, i)] * slackVarsInit[i] for i in SLACKVARS) == RHS[j])
    m.optimize()
#     print(m.getAttr('Status'))
    
    temp = ()
    for k, v in intVarsInit.items():
        temp = temp + (round(changeValue(v.X)),)
        
    for k, v in contVarsInit.items():
        temp = temp + (changeValue(v.X),)
    
    return temp

totalVarsInit = generateInitPoint()

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP(_totalVarsInit, _print=False):
    m = Model()
    m.setParam("LogToConsole", 0);
    
    intVarsStrong = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    contVarsStrong = m.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "continuous variable")
    slackVarsStrong = m.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
    
    m.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i] for i in CONVARS) 
                   + sum(MAT[(i, j)] * intVarsStrong[j] for j in INTVARS for i in CONSVARRHS)
                   + sum(MAT[(i, j)] * contVarsStrong[j] for j in CONVARS for i in CONSVARRHS), GRB.MINIMIZE)
    
    RHSFirstObj = 0
    for i in INTVARS:
        RHSFirstObj += OBJ[i] * _totalVarsInit[i]
    for i in CONVARS:
        RHSFirstObj += OBJ[i] * _totalVarsInit[i]
      
    m.addConstr(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i] for i in CONVARS)
                                                            <= RHSFirstObj)
    
    for j in CONSVARRHS:
        m.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) + 
                    sum(MAT[(j, i)] * contVarsStrong[i] for i in CONVARS) <= 
                    sum(MAT[(j, i)] * _totalVarsInit[i] for i in INTVARS) +
                    sum(MAT[(j, i)] * _totalVarsInit[i] for i in CONVARS))
        
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS) 
                    + sum(MATFixed[(j, i)] * contVarsStrong[i] for i in CONVARS) 
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m.optimize()
    
    if m.getAttr('Status') in [3, 4]:
        return _totalVarsInit
    
    if _print and debug_print:
        with open("Results_MILP_details_idea_4_{}.txt".format(instanceName), "a") as _filed:
            _filed.write('Solution in iteration 0' + '\n\n') 
            _filed.write("Integer Variables:\n")
            for k, v in intVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write("Continuous Variables:\n")
            for k, v in contVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
            _filed.write('\n' + "Slack Variables:\n")  
            for k, v in slackVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
            _filed.write('-------------------------------------' + '\n')
    
    temp = {}
    for k, v in intVarsStrong.items():
        temp[k] = round(changeValue(v.X))
    for k, v in contVarsStrong.items():
        temp[k] = changeValue(v.X)
    
    return temp
    
totalVarsInitStrong = convertWeakToStrongNDP(totalVarsInit, _print=True)

initPoint = tuple([totalVarsInitStrong[i] for i in INTVARS]) 

initTotalPoint = initPoint

intPartList = [] 

allIntPartDict = {}
allIntPartDict[0] = initPoint

EF = {}

temp_ndp = ()
temp_ndp = temp_ndp + (round(sum(OBJ[j]*totalVarsInitStrong[j] for j in INTVARS)) +
                        round(sum(OBJ[j]*totalVarsInitStrong[j] for j in CONVARS),2),)

for k in CONSVARRHS: 
    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*totalVarsInitStrong[l] for l in INTVARS)) +
                            round(sum(MAT[(k, l)]*totalVarsInitStrong[l] for l in CONVARS),2),)

EF[0] = temp_ndp


def calculate_distance(point_a_key, point_b_key): 
    values_dict_a = np.array(list(EF[point_a_key]))
    values_dict_b = np.array(list(EF[point_b_key]))
    
    return np.linalg.norm(values_dict_a - values_dict_b)

def generateSubIntLst(allIntPartDict, initPoint, initPointIdx, _totalIter):    
    if _totalIter <= 5:
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


shouldCallSublist = True
allIntPart = False
initPointIdx = 1
_iter = 0
counterRep = 0
totalIter = 1
numSubRatio = 0.5
theta_thresh = 100

# set it to True if you want to use presolve option of gurobi
presolve = False

if presolve:
    dualLB = -GRB.INFINITY
else:
    dualLB = -1e5
    
start = time.time()     
while True:
    if shouldCallSublist:
        intPartList = generateSubIntLst(allIntPartDict, initPoint, initPointIdx, totalIter)
        allIntPart = False
    else:
        intPartList = allIntPartDict
        shouldCallSublist = True
        allIntPart = True
    
    m = Model()
    m.setParam("LogToConsole", 0);
    m.params.NonConvex = 2
    m.Params.Presolve = 2
    
    thetaVar = m.addVar(vtype = GRB.CONTINUOUS, name = "theta")
    intVars = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "int_var")
    conVars = m.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "cont_var")
    slackVars = m.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack_var")
    dualVarsVarRHS = m.addVars(len(CONSVARRHS), len(intPartList), vtype = GRB.CONTINUOUS, lb = dualLB, ub = 0,
                               name = "dual_varying_RHS")
    dualVarsFixedRHS = m.addVars(len(CONSFIXEDRHS), len(intPartList), vtype = GRB.CONTINUOUS, lb = dualLB,
                               name = "dual_fixed_RHS")

    m.setObjective(thetaVar, GRB.MAXIMIZE)
    
    # theta constraint 
    for k in range(len(intPartList)):
        m.addConstr(thetaVar <=
          sum(OBJ[i]*(intPartList[k][i] - intVars[i]) for i in INTVARS)  
        - sum(OBJ[j]*conVars[j] for j in CONVARS)
        + sum(MAT[(i, j)]*dualVarsVarRHS[(i, k)]*(intVars[j] - intPartList[k][j]) for j in INTVARS for i in CONSVARRHS)
        + sum(MAT[(i, j)]*dualVarsVarRHS[(i, k)]*(conVars[j]) for j in CONVARS for i in CONSVARRHS)
        - sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, k)]*(intPartList[k][j]) for j in INTVARS for i in CONSFIXEDRHS)
        + sum(RHS[i]*dualVarsFixedRHS[(i, k)] for i in CONSFIXEDRHS))
        
    m.addConstr(thetaVar + sum(OBJ[i]*intVars[i] for i in INTVARS) + sum(OBJ[i]*conVars[i] for i in CONVARS) <= U + 1)
    
    for k in range(len(intPartList)):
        for j in CONVARS:
            m.addConstr(sum(MAT[(i, j)]*dualVarsVarRHS[(i, k)] for i in CONSVARRHS)
                      + sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, k)] for i in CONSFIXEDRHS) <= OBJ[j])
     
    for k in range(len(intPartList)):
        for j in SLACKVARS:
            m.addConstr(sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, k)] for i in CONSFIXEDRHS) <= 0)
            
    for j in CONSFIXEDRHS:
        m.addConstr(sum(MATFixed[(j, i)] * intVars[i] for i in INTVARS)
                  + sum(MATFixed[(j, i)] * conVars[i] for i in CONVARS)
                  + sum(MATFixed[(j, i)] * slackVars[i] for i in SLACKVARS) == RHS[j])
        
    m.optimize()
    
    end = time.time()
    elapsedTime = end - start
    print('time', round(elapsedTime,2))
    
    thetaVarValue = thetaVar.X
    print('thetaVarValue', round(thetaVarValue,2))
    
    totalIter += 1
    
    total_part = convertWeakToStrongNDP({**dict((k, round(changeValue(v.X))) for k,v in intVars.items()),
                            **dict((k, round(changeValue(v.X), 10)) for k,v in conVars.items())}, _print=False)
    
    int_part = tuple([total_part[i] for i in INTVARS]) 
    
    if (int_part in allIntPartDict.values()) and (not allIntPart):
        shouldCallSublist = False
        print('--------------------')
        continue
    
    if int_part not in allIntPartDict.values():
        _iter += 1  
        temp_ndp = ()
        temp_ndp = temp_ndp + (round(sum(OBJ[j]*total_part[j] for j in INTVARS)) +
                                round(sum(OBJ[j]*total_part[j] for j in CONVARS),2),)

        for k in CONSVARRHS: 
            temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*total_part[l] for l in INTVARS)) +
                                       round(sum(MAT[(k, l)]*total_part[l] for l in CONVARS),2),)
        EF[_iter] = temp_ndp 
        allIntPartDict[_iter] = int_part
        initPoint = int_part
        initPointIdx = _iter

        status = m.getAttr('Status')
        if debug_print and status == 2:
            with open("Results_MILP_details_idea_4_{}.txt".format(instanceName), "a") as _filed:
                _filed.write('Solution in iteration' + ' ' + str(_iter) + '\n\n') 
                _filed.write("Integer Variables:\n")
                for k, v in intVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
                _filed.write("Continuous Variables:\n")
                for k, v in conVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
                _filed.write('\n' + "Slack Variables:\n")  
                for k, v in slackVars.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
                _filed.write('\n' + "Dual Variables (varying RHSs):\n")  
                for k, v in dualVarsVarRHS.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
                _filed.write('\n' + "Dual Variables (fixed RHSs):\n")  
                for k, v in dualVarsFixedRHS.items():
                    _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
                _filed.write('-------------------------------------' + '\n')
    
    if (thetaVarValue < theta_thresh and allIntPart) or elapsedTime > timeLimit:
        if thetaVar.X < theta_thresh:
            print("Finished!")
        else:
            print("Finished due to time limit!")
            
        if thetaVarValue < theta_thresh:
            with open("Results_MILP_idea_4_{}.txt".format(instanceName), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(totalIter))
        else:
            with open("Results_MILP_idea_4_{}.txt".format(instanceName), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(totalIter))
        
        break
        
    print('--------------------')

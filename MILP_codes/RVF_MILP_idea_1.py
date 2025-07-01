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

# Set it to False for the two-stage converion
conversionOld = True

timeLimit = 14400
debug_print = False
mipForU = True

def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value

# Generate a feasible solution to start with
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
    
    temp = {}
    for k, v in intVarsInit.items():
        temp[k] = round(changeValue(v.X)) 
    for k, v in contVarsInit.items():
        temp[k] = changeValue(v.X) 
    
    return temp

totalVarsInit = generateInitPoint()

# one-stage conversion

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP_one_stage(_totalVarsInit, _print=False):
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
    
    if _print and debug_print:
        with open("Results_MILP_details_idea_1_{}.txt".format(instanceName), "a") as _filed:
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
    
    if m.getAttr('Status') in [3, 4]:
        return _totalVarsInit
    temp = {}
    for k, v in intVarsStrong.items():
        temp[k] = round(changeValue(v.X))
    for k, v in contVarsStrong.items():
        temp[k] = changeValue(v.X)
    
    return temp


# Two-stage conversion

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP_two_stage(_totalVarsInit, _print=False):
#     first stage problem
    m_1 = Model()
    m_1.setParam("LogToConsole", 0);
    
    intVarsStrong = m_1.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    contVarsStrong = m_1.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "continuous variable")
    slackVarsStrong = m_1.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
    
    m_1.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i] for i in CONVARS) 
                   , GRB.MINIMIZE)
    
    for j in CONSVARRHS:
        m_1.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) + 
                    sum(MAT[(j, i)] * contVarsStrong[i] for i in CONVARS) <= 
                    sum(MAT[(j, i)] * _totalVarsInit[i] for i in INTVARS) +
                    sum(MAT[(j, i)] * _totalVarsInit[i] for i in CONVARS))
        
    for j in CONSFIXEDRHS:
        m_1.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS) 
                    + sum(MATFixed[(j, i)] * contVarsStrong[i] for i in CONVARS) 
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m_1.optimize()
    
    if m_1.getAttr('Status') in [3, 4]:
        return _totalVarsInit
        
    var_m_1 = {}
    for k, v in intVarsStrong.items():
        var_m_1[k] = round(changeValue(v.X))
    for k, v in contVarsStrong.items():
        var_m_1[k] = changeValue(v.X)

    zVal = sum(OBJ[i] * intVarsStrong[i].X for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i].X for i in CONVARS)
    
#     second stage problem
    m_2 = Model()
    m_2.setParam("LogToConsole", 0);
    
    intVarsStrong = m_2.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    contVarsStrong = m_2.addVars(list(CONVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "continuous variable")
    slackVarsStrong = m_2.addVars(list(SLACKVARS), vtype = GRB.CONTINUOUS, lb = 0, name = "slack variable")
    
    m_2.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i] for i in CONVARS)
                   + sum(MAT[(i, j)] * intVarsStrong[j] for j in INTVARS for i in CONSVARRHS)
                   + sum(MAT[(i, j)] * contVarsStrong[j] for j in CONVARS for i in CONSVARRHS), GRB.MINIMIZE)
      
    m_2.addConstr(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(OBJ[i] * contVarsStrong[i] for i in CONVARS)
                                                            <= zVal)
    
    for j in CONSVARRHS:
        m_2.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) + 
                    sum(MAT[(j, i)] * contVarsStrong[i] for i in CONVARS) <= 
                    sum(MAT[(j, i)] * var_m_1[i] for i in INTVARS) +
                    sum(MAT[(j, i)] * var_m_1[i] for i in CONVARS))
        
    for j in CONSFIXEDRHS:
        m_2.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS) 
                    + sum(MATFixed[(j, i)] * contVarsStrong[i] for i in CONVARS) 
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m_2.optimize() 
    
    if m_2.getAttr('Status') in [3, 4]:
        return _totalVarsInit   
    
    if _print and debug_print:
        with open("Results_MILP_details_idea_1_{}.txt".format(instanceName), "a") as _filed:
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

if conversionOld:
    totalVarsInitStrong = convertWeakToStrongNDP_one_stage(totalVarsInit, _print=True)
else:
    totalVarsInitStrong = convertWeakToStrongNDP_two_stage(totalVarsInit, _print=True)

intPartList = []
totalPartList = []
intPartList.append({i: round(totalVarsInitStrong[i]) for i in INTVARS})
totalPartList.append(totalVarsInitStrong)

idxIntPartList = 1
thetaList = [] 
counterRep = 0
theta_thresh = 100
# set it to True if you want to use presolve option of gurobi
presolve = False

EF = []

if presolve:
    dualLB = -GRB.INFINITY
else:
    dualLB = -1e20

start = time.time() 
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

# theta constraint only for first int part index
m.addConstr(thetaVar <=
  sum(OBJ[i]*(intPartList[0][i] - intVars[i]) for i in INTVARS)  
- sum(OBJ[j]*conVars[j] for j in CONVARS)
+ sum(MAT[(i, j)]*dualVarsVarRHS[(i,0)]*(intVars[j] - intPartList[0][j]) for j in INTVARS for i in CONSVARRHS)
+ sum(MAT[(i, j)]*dualVarsVarRHS[(i, 0)]*(conVars[j]) for j in CONVARS for i in CONSVARRHS)
- sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, 0)]*(intPartList[0][j]) 
        for j in INTVARS for i in CONSFIXEDRHS)
+ sum(RHS[i]*dualVarsFixedRHS[(i, 0)] for i in CONSFIXEDRHS))

m.addConstr(thetaVar + sum(OBJ[i]*intVars[i] for i in INTVARS) + sum(OBJ[i]*conVars[i] for i in CONVARS) <= U + 1)

for j in CONVARS:
    m.addConstr(sum(MAT[(i, j)]*dualVarsVarRHS[(i, 0)] for i in CONSVARRHS)
                  + sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, 0)] for i in CONSFIXEDRHS) <= OBJ[j])

for j in SLACKVARS:
    m.addConstr(sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, 0)] for i in CONSFIXEDRHS) <= 0)

for j in CONSFIXEDRHS:
    m.addConstr(sum(MATFixed[(j, i)] * intVars[i] for i in INTVARS)
              + sum(MATFixed[(j, i)] * conVars[i] for i in CONVARS)
              + sum(MATFixed[(j, i)] * slackVars[i] for i in SLACKVARS) == RHS[j])

while True:
    m.optimize()
    status = m.getAttr('Status') 
    if debug_print and status == 2:
        with open("Results_MILP_details_idea_1_{}.txt".format(instanceName), "a") as _filed:
            _filed.write('Solution in iteration' + ' ' + str(idxIntPartList) + '\n\n') 
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
            _filed.write('\n' + "Theta: " + str(round(thetaVar.X, 2))  + '\n')
            _filed.write('-------------------------------------' + '\n')

    end = time.time()
    elapsedTime = end - start
    print('time', round(elapsedTime,2))
    
    if status == 2:
        thetaList.append(round(thetaVar.X, 2))
        thetaVarValue = thetaVar.X
    else:
        thetaVarValue = 0
    
    print('thetaVarValue', round(thetaVarValue,2))
    
    if thetaVarValue < theta_thresh or elapsedTime > timeLimit:
        if thetaVarValue < theta_thresh:
            print("Finished!")
        else:
            print("Finished due to time limit!")
        
        for efftotalSol in totalPartList:
            temp_ndp = () 
            temp_ndp = temp_ndp + (round(sum(OBJ[j]*efftotalSol[j] for j in INTVARS)) +
                           round(sum(OBJ[j]*efftotalSol[j] for j in CONVARS),2),)
    
            for k in CONSVARRHS:
                temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*efftotalSol[l] for l in INTVARS)) +
                               round(sum(MAT[(k, l)]*efftotalSol[l] for l in CONVARS),2),)
            
            EF.append(temp_ndp)
        
        if thetaVarValue < theta_thresh:
            with open("Results_MILP_idea_1_{}.txt".format(instanceName), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Theta list: ' + str(thetaList) + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(idxIntPartList))
        else:
            with open("Results_MILP_idea_1_{}.txt".format(instanceName), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Theta list: ' + str(thetaList) + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(idxIntPartList))
        
        break
    
    total_part = convertWeakToStrongNDP_one_stage({**dict((k, round(changeValue(v.X))) for k,v in intVars.items()),
                            **dict((k, round(changeValue(v.X), 10)) for k,v in conVars.items())}, _print=False)
        
    if {i: total_part[i] for i in INTVARS} in intPartList:
        counterRep += 1
        if counterRep >= 2:
            with open("Results_MILP_idea_1_{}.txt".format(instanceName), "w") as _file:
                _file.write('Algorithm stopped. Solution is already exist!' + '\n')
                _file.write('Efficient Frontier: ' + str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Theta list: ' + str(thetaList) + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(idxIntPartList))
                break
    
    intPartList.append({i: round(total_part[i]) for i in INTVARS})  
    totalPartList.append(total_part)

    # updating the model
    lastIntPartIndex = len(intPartList) - 1

    for i in CONSVARRHS:
        dualVarsVarRHS[(i,lastIntPartIndex)] = m.addVar(name = "dual_varying_RHS[{},{}]".format(i,lastIntPartIndex),
                                                    vtype = GRB.CONTINUOUS, lb = dualLB, ub = 0)
    for i in CONSFIXEDRHS:
        dualVarsFixedRHS[(i,lastIntPartIndex)] = m.addVar(name = "dual_fixed_RHS[{},{}]".format(i,lastIntPartIndex),
                                                    vtype = GRB.CONTINUOUS, lb = dualLB)

    # theta constraint only for last int part index
    m.addConstr(thetaVar <=
      sum(OBJ[i]*(intPartList[lastIntPartIndex][i] - intVars[i]) for i in INTVARS)  
    - sum(OBJ[j]*conVars[j] for j in CONVARS)
    + sum(MAT[(i, j)]*dualVarsVarRHS[(i,lastIntPartIndex)]*(intVars[j] - intPartList[lastIntPartIndex][j]) for j in INTVARS for i in CONSVARRHS)
    + sum(MAT[(i, j)]*dualVarsVarRHS[(i, lastIntPartIndex)]*(conVars[j]) for j in CONVARS for i in CONSVARRHS)
    - sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)]*(intPartList[lastIntPartIndex][j]) 
            for j in INTVARS for i in CONSFIXEDRHS) + sum(RHS[i]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS))

    for j in CONVARS:
        m.addConstr(sum(MAT[(i, j)]*dualVarsVarRHS[(i, lastIntPartIndex)] for i in CONSVARRHS)
                  + sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS) <= OBJ[j]) 

    for j in SLACKVARS:
        m.addConstr(sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS) <= 0)    
          
    for k in range(1, m.SolCount):
        m.Params.SolutionNumber = k 
        
        if conversionOld:
            _total_part = convertWeakToStrongNDP_one_stage({**dict((k, round(changeValue(v.Xn))) for k,v in intVars.items()),
                            **dict((k, round(changeValue(v.X), 10)) for k,v in conVars.items())}, _print=False)
        else:
            _total_part = convertWeakToStrongNDP_two_stage({**dict((k, round(changeValue(v.Xn))) for k,v in intVars.items()),
                            **dict((k, round(changeValue(v.X), 10)) for k,v in conVars.items())}, _print=False)
            
        if {i: _total_part[i] for i in INTVARS} not in intPartList:
            intPartList.append({i: round(_total_part[i]) for i in INTVARS})
            totalPartList.append(_total_part)
            
            # updating the model
            lastIntPartIndex = len(intPartList) - 1

            for i in CONSVARRHS:
                dualVarsVarRHS[(i,lastIntPartIndex)] = m.addVar(name = "dual_varying_RHS[{},{}]".format(i,lastIntPartIndex),
                                                            vtype = GRB.CONTINUOUS, lb = dualLB, ub = 0)
            for i in CONSFIXEDRHS:
                dualVarsFixedRHS[(i,lastIntPartIndex)] = m.addVar(name = "dual_fixed_RHS[{},{}]".format(i,lastIntPartIndex),
                                                            vtype = GRB.CONTINUOUS, lb = dualLB)

            # theta constraint only for last int part index
            m.addConstr(thetaVar <=
              sum(OBJ[i]*(intPartList[lastIntPartIndex][i] - intVars[i]) for i in INTVARS)  
            - sum(OBJ[j]*conVars[j] for j in CONVARS)
            + sum(MAT[(i, j)]*dualVarsVarRHS[(i,lastIntPartIndex)]*(intVars[j] - intPartList[lastIntPartIndex][j]) for j in INTVARS for i in CONSVARRHS)
            + sum(MAT[(i, j)]*dualVarsVarRHS[(i, lastIntPartIndex)]*(conVars[j]) for j in CONVARS for i in CONSVARRHS)
            - sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)]*(intPartList[lastIntPartIndex][j]) 
                    for j in INTVARS for i in CONSFIXEDRHS) + sum(RHS[i]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS))

            for j in CONVARS:
                m.addConstr(sum(MAT[(i, j)]*dualVarsVarRHS[(i, lastIntPartIndex)] for i in CONSVARRHS)
                          + sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS) <= OBJ[j])
            
            for j in SLACKVARS:
                m.addConstr(sum(MATFixed[(i, j)]*dualVarsFixedRHS[(i, lastIntPartIndex)] for i in CONSFIXEDRHS) <= 0)
            
    idxIntPartList += 1
    print('--------------------')
#!/usr/bin/env python
# coding: utf-8

'''
Created on Oct 01, 2022

@author: Samira Fallah (saf418@lehigh.edu)
'''

import time
import argparse
from gurobipy import *
import math

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--instance", help = "specify the instance")
flags = parser.parse_args()
instance = flags.instance 
f = open("InstancesTest_SPP/{}".format(instance), "r")
# f = open("InstancesTest_Knapsack/{}".format(instance), "r")

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

eps = 0.5

M = {}

for i in CONSVARRHS:
    M[i] = sum(-MAT[(i, j)] for j in range(numVar)) + 1

# Set it to False for the two-stage converion
conversionOld = True 

timeLimit = 86400
debug_print = False
ipForU = True

def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value

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

start = time.time() 
U = generatePointForU() 

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

# one-stage conversion

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP_one_stage(_intVarsInit, _print=False):
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
        with open("Results_IP_Linear_details_idea_3_{}.txt".format(instance.split('.')[0]), "a") as _filed:
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

# Two-stage conversion

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP_two_stage(_intVarsInit, _print=False):
#     first stage problem
    m_1 = Model()
    m_1.setParam("LogToConsole", 0);
    
    intVarsStrong = m_1.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    slackVarsStrong = m_1.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    
    m_1.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS), GRB.MINIMIZE)
    
    for j in CONSVARRHS:
        m_1.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) <= 
                    sum(MAT[(j, i)] * _intVarsInit[i] for i in INTVARS))
        
    for j in CONSFIXEDRHS:
        m_1.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS) 
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m_1.optimize()
    
    if m_1.getAttr('Status') in [3, 4]:
        return _intVarsInit
    
    var_m_1 = {}
    for k, v in intVarsStrong.items():
        var_m_1[k] = changeValue(v.X)
    
    zVal = sum(OBJ[i] * intVarsStrong[i].X for i in INTVARS)
    
#     second stage problem
    m_2 = Model()
    m_2.setParam("LogToConsole", 0);
    
    intVarsStrong = m_2.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "integer variable")
    slackVarsStrong = m_2.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack variable")
    
    m_2.setObjective(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) + sum(MAT[(i, j)] * intVarsStrong[j] for j in INTVARS for i in CONSVARRHS), GRB.MINIMIZE)
      
    m_2.addConstr(sum(OBJ[i] * intVarsStrong[i] for i in INTVARS) <= zVal)
    
    
    for j in CONSVARRHS:
        m_2.addConstr(sum(MAT[(j, i)] * intVarsStrong[i] for i in INTVARS) <= 
                      sum(MAT[(j, i)] * var_m_1[i] for i in INTVARS))
        
    for j in CONSFIXEDRHS:
        m_2.addConstr(sum(MATFixed[(j, i)] * intVarsStrong[i] for i in INTVARS) 
                    + sum(MATFixed[(j, i)] * slackVarsStrong[i] for i in SLACKVARS)  == RHS[j])
    
    m_2.optimize()    
    
    if m_2.getAttr('Status') in [3, 4]:
        return _intVarsInit
    
    if _print and debug_print:
        with open("Results_IP_Linear_details_idea_3_{}.txt".format(instance.split('.')[0]), "a") as _filed:
            _filed.write('Solution in iteration 0' + '\n\n') 
            _filed.write("Integer Variables:\n")
            for k, v in intVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X))) + '\n')
            _filed.write('\n' + "Slack Variables:\n")  
            for k, v in slackVarsStrong.items():
                _filed.write(str(k) + ' ' + str(round(changeValue(v.X), 5)) + '\n')
            _filed.write('-------------------------------------' + '\n')
    
    temp = {}
    for k, v in intVarsStrong.items():
        temp[k] = changeValue(v.X)

    return temp

if conversionOld:
    intVarsInitStrong = convertWeakToStrongNDP_one_stage(intVarsInit, _print=True)
else:
    intVarsInitStrong = convertWeakToStrongNDP_two_stage(intVarsInit, _print=True)

intPartList = []
intPartList.append(intVarsInitStrong)

EF = []

temp_ndp = ()
temp_ndp = temp_ndp + (sum(OBJ[j]*intVarsInitStrong[j] for j in INTVARS),)

for k in CONSVARRHS: 
    temp_ndp = temp_ndp + (sum(MAT[(k, l)]*intVarsInitStrong[l] for l in INTVARS),)

EF.append(temp_ndp)

idxIntPartList = 1

m = Model()
m.setParam("LogToConsole", 0);

thetaVar = m.addVar(vtype = GRB.CONTINUOUS, name = "theta")
intVars = m.addVars(list(INTVARS), vtype = GRB.INTEGER, lb = 0, ub = UB_I, name = "int_var")
slackVars = m.addVars(list(SLACKVARS), vtype = GRB.INTEGER, lb = 0, name = "slack_var")
alphaVars = m.addVars(len(CONSVARRHS), len(intPartList), vtype = GRB.BINARY, name = "alpha_var")
betaVars = m.addVars(len(intPartList), vtype = GRB.BINARY, name = "beta_var")

m.setObjective(thetaVar, GRB.MAXIMIZE)

# theta constraint -- const 1
m.addConstr(thetaVar <= - sum(OBJ[i]*intVars[i] for i in INTVARS) +   
    (1- betaVars[0])*sum(OBJ[j]*intPartList[0][j] for j in INTVARS) + betaVars[0]*(U+1))

# const 2
for i in CONSVARRHS:
    m.addConstr(sum(MAT[(i, j)]*(intVars[j] - intPartList[0][j]) for j in INTVARS) + eps 
                                            <= M[i]*(1 - alphaVars[(i, 0)]))

# const 3
for i in CONSVARRHS:
    m.addConstr(sum(MAT[(i, j)]*(intPartList[0][j] - intVars[j]) for j in INTVARS) 
                    <= M[i]*alphaVars[(i, 0)])

# const 4
for i in CONSVARRHS:
    m.addConstr(betaVars[0] >= alphaVars[(i, 0)])

# const 5
m.addConstr(betaVars[0] <= sum(alphaVars[(i, 0)] for i in CONSVARRHS))

# const 6
for k in CONSFIXEDRHS:
    m.addConstr(sum(MATFixed[(k, j)]*intVars[j] for j in INTVARS) + 
                sum(MATFixed[(k, j)]*slackVars[j] for j in SLACKVARS) == RHS[k])
        
m.addConstr(thetaVar >= 0.001)

m.params.SolutionLimit = 3

while True:
    m.optimize()
    status = m.getAttr('Status')
    
    if debug_print and status == 2:
        with open("Results_IP_Linear_details_idea_3_{}.txt".format(instance.split('.')[0]), "a") as _filed:
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
            _filed.write('-------------------------------------' + '\n')
        
    end = time.time()
    elapsedTime = end - start
    
    if status in [3, 4] or elapsedTime > timeLimit:
        if status in [3, 4]:
            print("Finished!")
        else:
            print("Finished due to time limit!")
        
        if status in [3, 4]:
            with open("Results_IP_Linear_idea_3_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations:' + str(idxIntPartList))
        else:
            with open("Results_IP_Linear_idea_3_{}.txt".format(instance.split('.')[0]), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Elapsed Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations:' + str(idxIntPartList))
        
        break
    
    for k in range(m.SolCount):
        thisNDP = ()
        m.Params.SolutionNumber = k 
        if conversionOld:
            int_part = convertWeakToStrongNDP_one_stage(dict((k, round(changeValue(v.Xn))) for k,v in intVars.items()), _print=False)
        else:
            int_part = convertWeakToStrongNDP_two_stage(dict((k, round(changeValue(v.Xn))) for k,v in intVars.items()), _print=False)
        
        thisNDP = thisNDP + (sum(OBJ[j]*int_part[j] for j in INTVARS),)
        
        for k in CONSVARRHS:
            thisNDP = thisNDP + (sum(MAT[(k, l)]*int_part[l] for l in INTVARS),)
            
        if thisNDP not in EF:
            intPartList.append(int_part)
            EF.append(thisNDP)
            
            # updating the model
            lastIntPartIndex = len(intPartList) - 1
            for i in CONSVARRHS:
                alphaVars[(i,lastIntPartIndex)] = m.addVar(name = "alpha_var[{},{}]".format(i,lastIntPartIndex),
                                                                        vtype = GRB.BINARY)

            betaVars[lastIntPartIndex] = m.addVar(name = "beta_var[{}]".format(lastIntPartIndex), vtype = GRB.BINARY)   

            # theta constraint only for last int part index
            m.addConstr(thetaVar <= - sum(OBJ[i]*intVars[i] for i in INTVARS) +   
                (1 - betaVars[lastIntPartIndex])*sum(OBJ[j]*intPartList[lastIntPartIndex][j] for j in INTVARS) + betaVars[lastIntPartIndex]*(U+1))

            # const 2 only for last int part index
            for i in CONSVARRHS:
                m.addConstr(sum(MAT[(i, j)]*(intVars[j] - intPartList[lastIntPartIndex][j]) for j in INTVARS) + eps 
                                                        <= M[i]*(1 - alphaVars[(i, lastIntPartIndex)]))

            # const 3 only for last int part index
            for i in CONSVARRHS:
                m.addConstr(sum(MAT[(i, j)]*(intPartList[lastIntPartIndex][j] - intVars[j]) for j in INTVARS) 
                                <= M[i]*alphaVars[(i, lastIntPartIndex)])

            # const 4 only for last int part index
            for i in CONSVARRHS:
                m.addConstr(betaVars[lastIntPartIndex] >= alphaVars[(i, lastIntPartIndex)])

            # const 5 only for last int part index
            m.addConstr(betaVars[lastIntPartIndex] <= sum(alphaVars[(i, lastIntPartIndex)] for i in CONSVARRHS))
    
    idxIntPartList += 1  
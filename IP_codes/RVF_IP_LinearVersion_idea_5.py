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
import random

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

intPartList = []
intPartList.append(intVarsInitStrong)
intPartListOrig = []
intPartListOrig.append(intVarsInitStrong)

EF = []

temp_ndp = () 
temp_ndp = temp_ndp + (round(sum(OBJ[j]*intVarsInitStrong[j] for j in INTVARS)),)

for k in CONSVARRHS:
    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*intVarsInitStrong[l] for l in INTVARS)),) 

EF.append(temp_ndp)

def genSubPrimalConst():
    distinct_i_values = list(set(i for i in RHS.keys()))
    random.shuffle(distinct_i_values)
    numSubPrimal = round(primalRatio * numConst)
    selected_i_values = distinct_i_values[:numSubPrimal]
    tempCONSFIXEDRHS = selected_i_values 

    return tempCONSFIXEDRHS

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

idxIntPartList = 1
thetaList = [] 

# consider all the primal for the subsequent iterations when it is True
allPrimalAfterThisIter = False

primalRatio = 0.1

while True:
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
            int_part_org = convertWeakToStrongNDP(resFindIntPart, _print=False)
            
            temp_ndp = () 
            temp_ndp = temp_ndp + (round(sum(OBJ[j]*int_part_org[j] for j in INTVARS)),)

            for k in CONSVARRHS:
                temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*int_part_org[l] for l in INTVARS)),) 

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
            
        EF.append(temp_ndp)
    
    if thetaList[-1] <= 1000 and not allPrimalAfterThisIter:
        allPrimalAfterThisIter = True
        intPartList = intPartListOrig
    
    idxIntPartList += 1  
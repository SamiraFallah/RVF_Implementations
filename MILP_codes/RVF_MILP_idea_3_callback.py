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
    m.Params.Threads = 1
    
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
    #m.display()
    
    return math.ceil(m.objVal)


U = generatePointForU()

list_num_nodes_rvf = []

# Generate a feasible solution to start with
def generateInitPoint():
    m = Model()
    m.setParam("LogToConsole", 0);
    m.Params.Threads = 1
    
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
    list_num_nodes_rvf.append(m.NodeCount)
   
    temp = {}
    for k, v in intVarsInit.items():
        temp[k] = round(changeValue(v.X)) 
    for k, v in contVarsInit.items():
        temp[k] = changeValue(v.X) 
    
    return temp
start = time.time() 
totalVarsInit = generateInitPoint()


# one-stage conversion

# Convert the feasible solution to an efficient solution
def convertWeakToStrongNDP_one_stage(_totalVarsInit, _print=False):
    m = Model()
    m.setParam("LogToConsole", 0);
    m.Params.Threads = 1
    
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
        with open("Results_MILP_details_idea_3_callback_{}.txt".format(instanceName), "a") as _filed:
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
        with open("Results_MILP_details_idea_3_callback_{}.txt".format(instanceName), "a") as _filed:
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
contPartList = []
intPartList.append({i: round(totalVarsInitStrong[i]) for i in INTVARS})
contPartList.append({i: round(totalVarsInitStrong[i]) for i in CONVARS})

temp_ndp = () 
temp_ndp = temp_ndp + (round(sum(OBJ[j]*totalVarsInitStrong[j] for j in INTVARS)) +
                       round(sum(OBJ[j]*totalVarsInitStrong[j] for j in CONVARS), 5),)

for k in CONSVARRHS:
    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*totalVarsInitStrong[l] for l in INTVARS)) +
                           round(sum(MAT[(k, l)]*totalVarsInitStrong[l] for l in CONVARS),5),) 

EF = []
EF.append(temp_ndp) 
len_ef_before = len(EF)

# set it to True if you want to use presolve option of gurobi
presolve = False

if presolve:
    dualLB = -GRB.INFINITY
else:
    dualLB = -1e20

m = Model()
m.setParam("LogToConsole", 0);
m.Params.Threads = 1
m.params.NonConvex = 2
# m.setParam('TimeLimit', 60)

if presolve:
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
    
m.addConstr(thetaVar >= 1, name = "theta_threshold")

while True:
    
    def my_callback(m, where):
        if where == GRB.Callback.MIPSOL:
            
            sol_list = m.cbGetSolution(intVars)
            current_solution = {k: round(sol_list[k]) for k in INTVARS}
            sol_list_con = m.cbGetSolution(conVars)
            current_solution_con = {k: round(sol_list_con[k]) for k in CONVARS}

            # convert and overwrite current solutions
            total_part = convertWeakToStrongNDP_one_stage({**current_solution, **current_solution_con}, _print=False)
            current_solution = {k: round(total_part[k]) for k in INTVARS}
            current_solution_con = {k:total_part[k] for k in CONVARS}
            
            if current_solution not in intPartList:
                temp_ndp = () 
                temp_ndp = temp_ndp + (round(sum(OBJ[j]*total_part[j] for j in INTVARS)) +
                                       round(sum(OBJ[j]*total_part[j] for j in CONVARS), 5),)
    
                for k in CONSVARRHS:
                    temp_ndp = temp_ndp + (round(sum(MAT[(k, l)]*total_part[l] for l in INTVARS)) +
                                           round(sum(MAT[(k, l)]*total_part[l] for l in CONVARS),5),) 
                
                EF.append(temp_ndp)    
                intPartList.append({k: round(total_part[k]) for k in INTVARS})
                contPartList.append({k:total_part[k] for k in CONVARS})
                
                m.terminate()
    
    m.optimize(my_callback)
   
    list_num_nodes_rvf.append(m.NodeCount)
    status = m.getAttr('Status')
    
    
    end = time.time()
    elapsedTime = end - start
    
    # log to new file
    with open("Results_MILP_idea_3_callback_new_{}.txt".format(instanceName), "a") as _file_logs:
        _file_logs.write('Efficient Frontier: '+ str(EF) + '\n')
        _file_logs.write('Total Running Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
        _file_logs.write('Number of NDPs: ' + str(len(EF)) + '\n')
        _file_logs.write('Number of Total Nodes of the BB: ' + str(sum(list_num_nodes_rvf)) + '\n')
        _file_logs.write('Avg Running time Per Node in RVF: ' + str(elapsedTime/sum(list_num_nodes_rvf)) + '\n')
        _file_logs.write('-'*30 + '\n')
        _file_logs.flush()
    
    len_ef_after = len(EF) 
    ef_not_changed = (len_ef_after == len_ef_before)
    len_ef_before = len_ef_after
    
    if status in [3, 4, 9] or elapsedTime > timeLimit or thetaVar.X < 0.1 or ef_not_changed:
        if status in [3, 4, 9] or thetaVar.X < 0.1 or ef_not_changed:
            print("Finished!")
        else:
            print("Finished due to time limit!")

        with open("Results_MILP_idea_3_callback_{}.txt".format(instanceName), "w") as _file:
            _file.write('Efficient Frontier: '+ str(EF) + '\n')
            _file.write('Total Running Time: ' + str(round(elapsedTime, 2)) + ' sec' + '\n')
            _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
            _file.write('Number of Total Nodes of the BB: ' + str(sum(list_num_nodes_rvf)) + '\n')
            _file.write('Avg Running time Per Node in RVF: ' + str(elapsedTime/sum(list_num_nodes_rvf)))
          
        break  
     
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
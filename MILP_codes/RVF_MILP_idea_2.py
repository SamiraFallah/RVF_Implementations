#!/usr/bin/env python
# coding: utf-8

# In[ ]:


'''
Created on Oct 01, 2022

@author: Samira Fallah (saf418@lehigh.edu)
'''


# In[ ]:


import time
import argparse
from gurobipy import *
import sys
import math


# # Insert the instance manually

# In[ ]:


# numVars = 11
# numIntVars = 9
# numContVars = 1
# numConsFixed = 1
# numConsVar = 1
# INTVARS = range(numIntVars)
# CONVARS = range(numIntVars, numIntVars+numContVars)
# SLACKVARS = range(numIntVars+numContVars, numVars)
# CONSVARRHS = range(numConsVar)
# CONSFIXEDRHS = range(numConsFixed)
# UB_I = 1
# instanceName = ''

# Ins 1
# OBJ = [-566, -611, -506, -180, -817, -184, -585, -423, -26, -317, 0]
# MAT = {(0, 0):-62, (0, 1):-84, (0, 2):-977, (0, 3):-979, (0, 4):-874, (0, 5):-54, (0, 6):-269, (0, 7):-93, (0, 8):-881, (0, 9):-563, (0, 10):0}
# MATFixed = {(0, 0):557, (0, 1):898, (0, 2):148, (0, 3):63, (0, 4):78, (0, 5):964, (0, 6):246, (0, 7):662, (0, 8):386, (0, 9):272, (0, 10):1}
# RHS = {0:2137}

# Ins 2
# OBJ = [-149, -741, -532, -380, -656, -391, -694, -371, -579, -662, 0]
# MAT = {(0, 0):-154, (0, 1):-201, (0, 2):-489, (0, 3):-993, (0, 4):-788, (0, 5):-148, (0, 6):-82, (0, 7):-232, (0, 8):-312, (0, 9):-178, (0, 10):0}
# MATFixed = {(0, 0):215, (0, 1):402, (0, 2):408, (0, 3):746, (0, 4):782, (0, 5):63, (0, 6):137, (0, 7):476, (0, 8):434, (0, 9):715, (0, 10):1}
# RHS = {0:2189}

# Ins 3
# OBJ = [-230, -870, -555, -578, -995, -596, -800, -819, -630, -504, 0]
# MAT = {(0, 0):-747, (0, 1):-816, (0, 2):-996, (0, 3):-509, (0, 4):-700, (0, 5):-238, (0, 6):-395, (0, 7):-870, (0, 8):-236, (0, 9):-292, (0, 10):0}
# MATFixed = {(0, 0):860, (0, 1):900, (0, 2):169, (0, 3):415, (0, 4):477, (0, 5):163, (0, 6):10, (0, 7):277, (0, 8):981, (0, 9):640, (0, 10):1}
# RHS = {0:2446}

# UB_I = 1


# In[ ]:


# numVars = 9
# numIntVars = 2
# numContVars = 5
# numConsFixed = 3
# numConsVar = 1
# INTVARS = range(numIntVars)
# CONVARS = range(numIntVars, numIntVars+numContVars)
# SLACKVARS = range(numIntVars+numContVars, numVars)
# CONSVARRHS = range(numConsVar)
# CONSFIXEDRHS = range(numConsFixed)

# # Ins 4
# OBJ = [2, 5, 0, 7, 10, 2, 10, 0, 0]
# MAT = {(0, 0):-1, (0, 1):-10, (0, 2):10, (0, 3):-8, (0, 4):1, (0, 5):-7, (0, 6):6, (0, 7):0, (0, 8):0}
# MATFixed = {(0, 0):-1, (0, 1):4, (0, 2):9, (0, 3):3, (0, 4):2, (0, 5):6, (0, 6):-10, (0, 7):0, (0, 8):0,
#            (1, 0):0, (1, 1):5, (1, 2):0, (1, 3):1, (1, 4):0, (1, 5):0, (1, 6):0, (1, 7):1, (1, 8):0,
#            (2, 0):0, (2, 1):5, (2, 2):0, (2, 3):0, (2, 4):0, (2, 5):0, (2, 6):1, (2, 7):0, (2, 8):1}

# RHS = {0:4, 1:5, 2:5}
# UB_I = 1


# In[ ]:


# adding Folder to the system path
sys.path.insert(0, '/home/saf418/ValueFunctionCode/RVFCodes/Gurobi/milp_1')

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--instance", help = "specify the instance")
flags = parser.parse_args()
instanceName = flags.instance.split('.py')[0]
for key, val in vars(__import__(instanceName)).items():
    if key.startswith('__') and key.endswith('__'):
        continue
    vars()[key] = val


# In[ ]:


# from InstanceTest.obj_3_var_10_nzr_4_intR_6_U_1_ins_2 import *
# instanceName = 'obj_3_var_10_nzr_4_intR_6_U_1_ins_2'


# In[ ]:


# from InstanceTest.MILP_obj_2_var_10_nzr_4_intR_6_u_1_ins_0 import *
# instanceName = 'MILP_obj_2_var_10_nzr_4_intR_6_u_1_ins_0'


# In[ ]:


# Set it to False for the two-stage converion
conversionOld = True 


# In[ ]:


timeLimit = 14400
debug_print = False
mipForU = True


# In[ ]:


def changeValue(value):
    if str(value) == 'None':
        return 0.0
    return value


# In[ ]:


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
    #m.display()
    
    return math.ceil(m.objVal)


# In[ ]:


U = generatePointForU()


# In[ ]:


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


# # one-stage conversion

# In[ ]:


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
        with open("Results_MILP_details_idea_2_{}.txt".format(instanceName), "a") as _filed:
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


# # Two-stage conversion

# In[ ]:


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
        with open("Results_MILP_details_idea_2_{}.txt".format(instanceName), "a") as _filed:
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


# In[ ]:


if conversionOld:
    totalVarsInitStrong = convertWeakToStrongNDP_one_stage(totalVarsInit, _print=True)
else:
    totalVarsInitStrong = convertWeakToStrongNDP_two_stage(totalVarsInit, _print=True)


# In[ ]:


intPartList = []
totalPartList = []
intPartList.append({i: round(totalVarsInitStrong[i]) for i in INTVARS})
totalPartList.append(totalVarsInitStrong)


# In[ ]:


idxIntPartList = 1
iter_num = 1
defaultTheta = 10
currentTheta = 10
total_time = 0
EF = []

# set it to True if you want to use presolve option of gurobi
presolve = False

if presolve:
    dualLB = -GRB.INFINITY
else:
    dualLB = -1e20

start = time.time() 
m = Model()
m.setParam("LogToConsole", 0);
m.params.SolutionLimit = 1
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

m.setObjective(1, GRB.MAXIMIZE)

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

m.addConstr(thetaVar >= currentTheta, name = "theta_threshold")

while True:
    start_it = time.time() 
    
    m.optimize() 
    status = m.getAttr('Status')
    
    if debug_print and status == 2:
        with open("Results_MILP_details_idea_2_{}.txt".format(instanceName), "a") as _filed:
            _filed.write('Solution in iteration' + ' ' + str(iter_num) + '\n\n') 
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

    end_it = time.time()
    iter_time = end_it - start_it
    
    end = time.time()
    elapsedTime = end - start
    print('time', round(elapsedTime,2))
    
    print('status', status)
    
    if status in [3, 4] or elapsedTime > timeLimit:
        if status in [3, 4]:
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

        if status in [3, 4]:
            with open("Results_MILP_idea_2_{}.txt".format(instanceName), "w") as _file:
                _file.write('Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Total Running Time: ' + str(round(total_time, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(idxIntPartList))
        else:
            with open("Results_MILP_idea_2_{}.txt".format(instanceName), "w") as _file:
                _file.write('Approximate Efficient Frontier: '+ str(EF) + '\n')
                _file.write('Total Running Time: ' + str(round(total_time, 2)) + ' sec' + '\n')
                _file.write('Number of NDPs: ' + str(len(EF)) + '\n')
                _file.write('Number of iterations: ' + str(idxIntPartList))
                
        break  
    
#     temp_dualVarsVarRHS = {}
#     for k, v in dualVarsVarRHS.items():
#         temp_dualVarsVarRHS[k] = round(changeValue(v.X))
#     print('temp_dualVarsVarRHS', temp_dualVarsVarRHS)
    
#     temp_dualVarsFixedRHS = {}
#     for k, v in dualVarsFixedRHS.items():
#         temp_dualVarsFixedRHS[k] = round(changeValue(v.X))
#     print('temp_dualVarsFixedRHS', temp_dualVarsFixedRHS)
    
    if conversionOld:
        total_part = convertWeakToStrongNDP_one_stage({**dict((k, round(changeValue(v.X))) for k,v in intVars.items()), **dict((k, round(changeValue(v.X))) for k,v in conVars.items())}, _print=False)
    else:
        total_part = convertWeakToStrongNDP_one_stage({**dict((k, round(changeValue(v.X))) for k,v in intVars.items()), **dict((k, round(changeValue(v.X))) for k,v in conVars.items())}, _print=False)
    
    if {i: total_part[i] for i in INTVARS} in intPartList:
        print('found repetitive')
        currentTheta = currentTheta * 10
        print('currentTheta', currentTheta)

    else:
        currentTheta = defaultTheta  
        intPartList.append({k: round(total_part[k]) for k in INTVARS})
        totalPartList.append(total_part)
        total_time += iter_time
        idxIntPartList += 1
        
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
            
    theta_threshold_c = m.getConstrByName('theta_threshold')
    m.remove(theta_threshold_c)
    m.addConstr(thetaVar >= currentTheta, name = "theta_threshold")
    
    iter_num += 1
    print('--------------------')


# In[ ]:


# EF


# In[ ]:


# len(EF)


# In[ ]:


# intPartList


# In[ ]:





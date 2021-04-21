import pyomo.environ as pyo
import numpy as np

def read_instance(file_name):
    opening_cost = {}
    demand = {}
    capacity = {}
    travel_cost = {}
    try:
        file = open("Instances/{}".format(file_name),'r')
        info = file.readline().split(" ")
        I = int(info[0])
        J = int(info[1])
        info = file.readline().split(" ")
        for j in range(J):
            opening_cost[j] = int(info[j])
        info = file.readline().split(" ")
        for i in range(I):
            demand[i] = int(info[i])
        info = file.readline().split(" ")
        for j in range(J):
            capacity[j] = int(info[j])
        for i in range(I):
            info = file.readline().split(" ")
            for j in range(J):
                travel_cost[(i,j)] = int(info[j])
    except:
        print("Error reading file.")
    return opening_cost,demand,capacity,travel_cost


def obj_function(model):
    return sum(model.f[j]*model.y[j] for j in model.J)+sum(model.t[(i,j)]*model.x[(i,j)] for i in model.I for j in model.J)

def first_constraint_rule(model, j):
    return sum(model.x[(i,j)] for i in model.I) <= model.c[j]*model.y[j]

def sec_constraint_rule(model, i):
    return sum(model.x[(i,j)] for j in model.J) >= model.d[i]



def solve_flp(instance_name,linear):
    opening_cost, demand, capacity, travel_cost = read_instance(instance_name)

    #model creation
    model = pyo.ConcreteModel()
    model.I = pyo.RangeSet(0,len(demand)-1)
    model.J = pyo.RangeSet(0,len(capacity)-1)


    model.f = pyo.Param(model.J,initialize=opening_cost,default=0)#f
    model.c = pyo.Param(model.J,initialize=capacity,default=0)#u??
    model.d = pyo.Param(model.I,initialize=demand,default=0)#d
    model.t = pyo.Param(model.I,model.J,initialize=travel_cost,default=0)#t

    #instance = model.create_instance('abs_data.dat')
    #instance.pprint()

    model.x = pyo.Var(model.I, model.J, domain=pyo.NonNegativeIntegers)#A discuter
    model.y = pyo.Var(model.J, domain=pyo.Binary)

    #Add objectives and constraints

    model.obj = pyo.Objective(rule=obj_function)

    model.con_1 = pyo.Constraint(model.J,rule=first_constraint_rule)
    model.con_2 = pyo.Constraint(model.I,rule=sec_constraint_rule)

    opt = pyo.SolverFactory("glpk")
    opt.solve(model,tee=False)
    print(pyo.value(model.obj))

    #return (obj,x,y)
    
def initial_solution_flp(instance_name):
    pass
    #return (obj,x,y)
    
def local_search_flp(x,y):
    pass
    #return (obj,x,y)


if __name__ == '__main__':
    solve_flp("FLP-100-20-0.txt", False)
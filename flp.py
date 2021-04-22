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
#WHY NOT MERGE BOTH FOR ?
#AGAIN WHY are there parenthesis between the brackets?
def first_constraint_rule(model, j):
    return sum(model.x[(i,j)] for i in model.I) <= model.c[j]*model.y[j]
#WHY are there parenthesis between the brackets?
def sec_constraint_rule(model, i):
    return sum(model.x[(i,j)] for j in model.J) >= model.d[i]
#WHY are there parenthesis between the brackets?

def sec_function(x,y,t,f):
    return sum(f[j]*y[j] for j in range(len(y)))+sum(t[(i,j)]*x[(i,j)] for i in range(len(x)) for j in range(len(x[0])))


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

    if (linear==False):
        model.x = pyo.Var(model.I, model.J, domain=pyo.NonNegativeIntegers)#A discuter
        model.y = pyo.Var(model.J, domain=pyo.Binary)
    elif (linear==True):
        model.x = pyo.Var(model.I, model.J, domain=pyo.NonNegativeReals)
        model.y = pyo.Var(model.J, domain=pyo.NonNegativeReals, bounds=(0,1))

    #Add objectives and constraints
    model.obj = pyo.Objective(rule=obj_function)
    model.con_1 = pyo.Constraint(model.J,rule=first_constraint_rule)
    model.con_2 = pyo.Constraint(model.I,rule=sec_constraint_rule)

    #solve
    opt = pyo.SolverFactory("glpk")
    opt.solve(model,tee=False)
    #print(pyo.value(model.obj))

    x = np.zeros(shape=(np.asarray(model.I)[-1]+1,np.asarray(model.J)[-1]+1))
    y = np.zeros(shape=(np.asarray(model.J)[-1]+1))
    for i in np.asarray(model.I):
        for j in np.asarray(model.J):
            x[i,j]=model.x[i,j].value

    for i in np.asarray(model.J):
        y[i] = model.y[i].value

    obj = pyo.value(model.obj)

    c = np.zeros(shape=y.shape)
    d = np.zeros(shape=x.shape[0])

    for k in np.asarray(model.J):
        c[k] = model.c[k]
    for k in np.asarray(model.I):
        d[k] = model.d[k]

    t = np.zeros(shape=x.shape)
    f = np.zeros(shape=y.shape)

    for k in np.asarray(model.J):
        f[k] = model.f[k]
    for k in np.asarray(model.I):
        for l in np.asarray(model.J):
            t[k,l]=model.t[k,l]

    print(obj)


    return (obj,x,y,model,c,d,t,f)

def initial_solution_flp(instance_name):
    #GREEDY ALGORITHM
    obj, x, y, model,c,d,t,f = solve_flp(instance_name, True)


#----------------Algo commence

    sort_y = np.zeros(shape=(y.shape[0], 2))

    for j in range(len(y)):
        sort_y[j,0] = y[j]
        sort_y[j,1] = j

    sort_y = sort_y[np.argsort(sort_y[:, 0])[::-1]]

    x_bar = np.zeros(shape=x.shape)
    y_bar = np.zeros(shape=y.shape)


    temp = np.zeros(shape=(x.shape[0], 2))


    for j_p in range(len(sort_y)):#len(sort_y)

        j=int(sort_y[j_p,1])

        temp[:,0] = x[:,j]
        temp[:,1] = [i for i in range(len(x))]

        temp = temp[np.argsort(temp[:, 0])[::-1]]
        for i_p in range(len(temp)):
            i=int(temp[i_p,1])

            if sum(x_bar[:,j])<c[j] and sum(x_bar[i,:])<d[i]:
                x_bar[i,j] = min(c[j]-sum(x_bar[:,j]), d[i]-sum(x_bar[i,:]))

        
        if satisfying_cond(x_bar, d):
            obj = sec_function(x_bar,y_bar,t,f)
            print(obj)
            return(obj,x_bar,y_bar)
    #return (obj,x,y)

def satisfying_cond(x_bar, d):
    j=0
    for i in range(len(d)):
        #print(d[i], sum(x_bar[i,:]))
        if sum(x_bar[i,:])<d[i]:
            j+=1
    if j!=0:
        return False
    else:
        return True

def local_search_flp(x,y):
    pass
    #return (obj,x,y)


if __name__ == '__main__':
    #solve_flp("FLP-100-20-0.txt", False)
    #initial_solution_flp("FLP-100-20-0.txt")
    initial_solution_flp("FLP-200-40-0.txt")
    #solve_flp("FLP-100-20-0.txt", False)

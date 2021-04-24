import pyomo.environ as pyo
import numpy as np
import random
import time
import copy

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

def convertToNpArray(model): #x,y,obj,c,d,t,f,
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
    return x,y,obj,c,d,t,f

def facility_movement(y_bar, x_bar,c, seed):
    j1_p, j2_p, j1_m, j2_m = random_assignement(x_bar,c, seed)

    #--- CORRECTION: BEGIN ---#
    #y_bar[j1_p], y_bar[j2_p] = 0, 0 #ERROR? should be 1, 1
    #missing y_bar[j1_m] and y_bar[j2_m] = 0, 0
    y_bar[j1_p], y_bar[j2_p] = 1, 1
    y_bar[j1_m], y_bar[j2_m] = 0, 0

    #x[:,j1_m] = [0 for i in range(x_bar.shape[0])] #ERROR, should be x_bar
    #x[:,j2_m] = [0 for i in range(x_bar.shape[0])] #ERROR, should be x_bar
    x_bar[:,j1_m] = 0 #no need for the for, the ":" takes care of all the i elements
    x_bar[:,j2_m] = 0 #no need for the for, the ":" takes care of all the i elements

    #--- CORRECTION: END ---#
    return y_bar, x_bar

#TODO: CHANGE FOR LOOPS
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

    x,y,obj,c,d,t,f = convertToNpArray(model)
    #print(obj)
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
            #print(obj)
            return(obj,x_bar,y_bar,c,d,t,f)
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

def sort_function(vector, order):
    sort_v = np.zeros(shape=(vector.shape[0], 2)).astype('uint16')
    for i in range(len(vector)):
        sort_v[i,0] = vector[i]
        sort_v[i,1] = i
    if (order=="decreasing"):
        sort_v = sort_v[np.argsort(sort_v[:, 0])[::-1]]
    elif (order=="increasing"):
        sort_v = sort_v[np.argsort(sort_v[:, 0])]
    else:
        print("Please choose either 'decreasing' or 'increasing' for the parameter 'order' ")

    return sort_v
#y_bar, x_bar = greedy_reassign(y_bar, x_bar, sort_d, t, c, d)
def greedy_reassign(y_bar, x_bar, sort_d, t, c, d):
    temp = np.zeros(shape=(t.shape[1], 2)).astype('uint16')

    for i_p in range(x_bar.shape[0]):
        i=sort_d[i_p,1]
        #Sorting the travel cost by increasing order
        temp[:,0] = t[i,:]
        temp[:,1] = [j for j in range(t.shape[1])]
        temp = temp[np.argsort(temp[:, 0])]

        for j_p in range(x_bar.shape[1]): #ambiguity with previous j_p (which standed for j_plus not j_prime)
            j = temp[j_p,1]

            if y_bar[j] ==1 and sum(x_bar[:,j])<c[j] and sum(x_bar[i,:])<d[i]:
                x_bar[i,j] = min(c[j]-sum(x_bar[:,j]), d[i]-sum(x_bar[i,:]))
    return y_bar, x_bar

def local_search_flp(instance_name):

    t_end = 0.1*60#stopping criterion
    t_1 = time.time()
    obj,x,y,c,d,t,f = initial_solution_flp(instance_name)

    #Sorting d in decreasing order
    sort_d = sort_function(d, "decreasing")
    #Initialization
    y_bar, x_bar = np.copy(y), np.copy(x)

    continue_search = True
    seed_original = 0
    while(continue_search):
        #Perturbation
        random.seed(seed_original)
        seed= random.randrange(100000)
        y_new, x_new = facility_movement(y_bar, x_bar, c, seed)
        y_new, x_new = greedy_reassign(y_new, x_new, sort_d, t, c, d)
        obj_new = sec_function(x_new,y_new,t,f)
        if (obj_new<obj):
            obj=copy.deepcopy(obj_new)
            y_bar, x_bar = copy.deepcopy(y_new),copy.deepcopy(x_new)
        t_2 = time.time()
        print("-------------->", obj, obj_new)
        #also add constraint: max number of iteration without improvement
        if t_end<t_2-t_1:
            continue_search=False
        seed_original+=1

    #print(t_end, t_2-t_1)
    obj = sec_function(x_bar,y_bar,t,f) #redundant but let's keep it for the readibility
    print("-->", y_bar, y_new)
    print(obj)
    return(obj, x_bar,y_bar)
    #return (obj,x,y)

def random_assignement(x_bar, c, seed):
    i=seed
    random.seed(i)
    j1_p = random.randrange(x_bar.shape[1])
    j2_p = random.randrange(x_bar.shape[1])
    j1_m = random.randrange(x_bar.shape[1])
    j2_m = random.randrange(x_bar.shape[1])

    while sum(x_bar[:,j1_m]+x_bar[:,j2_m])>c[j1_p]+c[j2_p] and i<10000:
        i+=1
        random.seed(i)
        j1_p = random.randrange(x_bar.shape[1])
        j2_p = random.randrange(x_bar.shape[1])
        j1_m = random.randrange(x_bar.shape[1])
        j2_m = random.randrange(x_bar.shape[1])


    return j1_m,j1_p,j2_m,j2_p


if __name__ == '__main__':
    #solve_flp("FLP-100-20-0.txt", False)
    #initial_solution_flp("FLP-100-20-0.txt")
    #initial_solution_flp("FLP-200-40-0.txt")
    local_search_flp("FLP-100-20-0.txt")
    #solve_flp("FLP-100-20-0.txt", False)

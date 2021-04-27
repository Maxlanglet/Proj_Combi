import pyomo.environ as pyo
import numpy as np
import random
import time
import copy
import sys

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

def facility_movement(y, x,c, seed):
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)

    j1_p,j2_p, j1_m, j2_m = random_facility(x_bar, y_bar,c, seed)

    y_bar[j1_p], y_bar[j2_p] = 1, 1
    y_bar[j1_m], y_bar[j2_m] = 0, 0

    x_bar[:,j1_m] = 0
    x_bar[:,j2_m] = 0

    return y_bar, x_bar

def assignment_movement(y,x,c,seed):
    random.seed(seed)
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)
    j = random_assignment(x_bar, seed) #j contains i1 (and may be i2) and 2 facilities associated to him

    if j == []:
        return y_bar, x_bar
    # A close look inside the vector j = [[i1,j11,j12], [i2,j21,j22]]
    #print("---> Look at j: ",j)
    #TODO : Reassign randomly each demand to up to 2 facilities each.
    for k in range (j.shape[0]):

        sum_xij_for_2_facilities=0
        sum_xij_for_2_facilities = x_bar[j[k,0],j[k,1]] + x_bar[j[k,0],j[k,2]]
        # print("y.shape[0] : ", j.shape[0])
        # print("x_bar[j[k,0],j[k,1]] : ",x_bar[j[k,0],j[k,1]] )
        # print("x_bar[j[k,0],j[k,2]]", x_bar[j[k,0],j[k,2]])
        # print("sum_xij_for_2_facilities :", sum_xij_for_2_facilities)
        first_new_xij = random.randrange(0, sum_xij_for_2_facilities)
        second_new_xij= sum_xij_for_2_facilities - first_new_xij
        x_bar[j[k,0],j[k,1]] = first_new_xij
        x_bar[j[k,0],j[k,2]] = second_new_xij
        # print("first_new_xij: ", first_new_xij)
        # print("second_new_xij : ", second_new_xij)
        # print("x_bar[j[k,0],j[k,1]] : ", x_bar[j[k,0],j[k,1]])
        # print("x_bar[j[k,0],j[k,2]] : ", x_bar[j[k,0],j[k,2]])
        # print("coucou")


    #rand_reass =
    return y_bar, x_bar



#TODO: CHANGE FOR LOOPS
def solve_flp(instance_name,linear):
    start = time.time()
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

    end = time.time()
    x,y,obj,c,d,t,f = convertToNpArray(model)
    #print(obj, ", ", end-start, ", ", instance_name)
    return (obj,x,y,model,c,d,t,f)

def initial_solution_flp(instance_name):
    #GREEDY ALGORITHM
    start = time.time()
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
    for j_p in range(len(sort_y)):
        j=int(sort_y[j_p,1])
        y_bar[j] = 1
        temp[:,0] = x[:,j]
        temp[:,1] = [i for i in range(len(x))]
        temp = temp[np.argsort(temp[:, 0])[::-1]]
        for i_p in range(len(temp)):
            i=int(temp[i_p,1])

            if sum(x_bar[:,j])<c[j] and sum(x_bar[i,:])<d[i]:
                x_bar[i,j] = min(c[j]-sum(x_bar[:,j]), d[i]-sum(x_bar[i,:]))

        if satisfying_cond(x_bar, d):
            obj = sec_function(x_bar,y_bar,t,f)
            end = time.time()
            #print(obj, ", ", end-start, ", ", instance_name)
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

def greedy_reassign(y, x, sort_d, t, c, d):
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)
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

    t_end = 30*60#stopping criterion
    t_1 = time.time()
    obj,x,y,c,d,t,f = initial_solution_flp(instance_name)

    #Sorting d in decreasing order
    sort_d = sort_function(d, "decreasing")
    #Initialization
    y_bar, x_bar = copy.deepcopy(y), copy.deepcopy(x)
    #print("########## BEGIN LOCAL SEARCH ##########")

    y_best, x_best = copy.deepcopy(y_bar), copy.deepcopy(x_bar)
    obj_best = copy.deepcopy(obj)

    counter_no_improvement=0
    max_no_improvement = 10
    count_local_moves=0
    max_local_moves_no_improvement = 300
    #since the seed changes at each iteration of the while
    #it makes sense to suppose a better move even
    #after 3 different local moves without improvement"
    seed_original = 0
    move_facility=False
    move_assignment=True
    continue_search=True
    # print("move_facility: ", move_facility)
    # print("move_assignment: ", move_assignment)

    while(continue_search and count_local_moves<max_local_moves_no_improvement):
        random.seed(seed_original)
        seed= random.randrange(100000)

        #Perturbation (local move)
        if counter_no_improvement>max_no_improvement:
            counter_no_improvement=0
            count_local_moves+=1
            move_facility = not move_facility
            move_assignment = not move_assignment
            # print("move_facility: ", move_facility)
            # print("move_assignment: ", move_assignment)
            if count_local_moves==max_local_moves_no_improvement:
                print("No more improvement with both movements, the search stops")

        if move_facility==True:
            y_new, x_new = facility_movement(y_best, x_best, c, seed)
            if np.sum(y_new) < np.sum(y_bar)-x_bar.shape[1]//10:
                y_new, x_new = y_best, x_best
                #y_new, x_new = y_bar, x_bar
                move_facility = not move_facility
                move_assignment = not move_assignment

        if move_assignment==True:
            y_new, x_new = assignment_movement(y_best, x_best, c, seed)
            if y_new.all() == y_best.all() and x_new.all() == x_best.all():
                move_facility = not move_facility
                move_assignment = not move_assignment

        
        y_new, x_new = greedy_reassign(y_new, x_new, sort_d, t, c, d)
        obj_new = sec_function(x_new,y_new,t,f)
        if (obj_new<obj_best):
            count_local_moves=0
            counter_no_improvement=0
            if respect_constraints(x_new, y_new, c, d):
                y_best, x_best = copy.deepcopy(y_new),copy.deepcopy(x_new)
                obj_best= sec_function(x_best,y_best,t,f)
                #print(obj_best)
        else:
            counter_no_improvement+=1

        #Stop criterion
        t_2 = time.time()
        if t_end<t_2-t_1:
            continue_search=False
        seed_original+=1

    #print(sec_function(x_best,y_best,t,f))
    print(obj_best, ", ", t_2-t_1, ", ", instance_name)
    #print(y_best)
    #print(obj)
    return(obj, x_best,y_best)
    #return (obj,x,y)

def respect_constraints(x,y,c,d):
    const1 = [sum(x[i,:])>=d[i] for i in range(x.shape[0])]
    const2 = [sum(x[:,j])<=c[j]*y[j] for j in range(x.shape[1])]

    if np.all(const1) and np.all(const2):
        return True
    else:
        return False


def random_facility(x_bar,y_bar, c, seed):
    i=seed
    random.seed(i)
    j1_p = random.randrange(x_bar.shape[1])
    j2_p = random.randrange(x_bar.shape[1])
    j1_m = random.randrange(x_bar.shape[1])
    j2_m = random.randrange(x_bar.shape[1])
    #although not explicit, it will randomly choose between 0 and 2 facilities to open or and to close
    while sum(x_bar[:,j1_m]+x_bar[:,j2_m])>c[j1_p]+c[j2_p]: # and y_bar[j1_p]==0 and y_bar[j2_p]==0 and y_bar[j1_m]==1 and y_bar[j2_m]==1
        i+=1
        random.seed(i)
        j1_p = random.randrange(x_bar.shape[1])
        j2_p = random.randrange(x_bar.shape[1])
        j1_m = random.randrange(x_bar.shape[1])
        j2_m = random.randrange(x_bar.shape[1])

    return j1_p, j2_p, j1_m, j2_m

def random_assignment(x_bar, seed):
    s=seed
    np.random.seed(s)
    random.seed(s)
    nb_customers = random.randrange(1,3)
    #condition=0
    nb_facilities = 2
    valid_i = np.count_nonzero(x_bar>0, axis = 1)
    valid_i = np.flatnonzero(valid_i>=2)
    if len(valid_i) >1:
        j = np.zeros(shape=(nb_customers,3)).astype('uint8')
        j[:,0] = np.random.choice(valid_i, nb_customers, replace=False)
    elif len(valid_i) == 1:
        j = np.zeros(shape=(1,3)).astype('uint8')
        j[:,0] = np.random.choice(valid_i, 1, replace=False)
    else:
        return []
    line = 0
    for p in j[:,0]:
        valid_j = np.flatnonzero(x_bar[p,:])
        #j_list = np.random.choice(valid_j, 4, replace=False)
        j[line,1:] = np.random.choice(valid_j, 2, replace=False)
        line+=1
    #not useful to test the condition anymore because we looked into i,j values
    #that respected this condition

    # condition=1
    # for i in range(nb_customers):
    #     condition*= x_bar[j[i,0],j[i,1]]*x_bar[j[i,0],j[i,2]]

    return j


if __name__ == '__main__':
    #instance = "FLP-150-30-0.txt"
    instance = sys.argv[1]
    #print("best solution:")
    #solve_flp(instance, False)
    #initial_solution_flp(instance)
    local_search_flp(instance)
    #solve_flp("FLP-100-20-0.txt", False)

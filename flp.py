import pyomo.environ as pyo
import numpy as np
import random
import time
import copy
import sys
from matplotlib import pyplot as plt

c = np.empty(1)#placeholder to define them as global
d = np.empty(1)
t = np.empty(1)
f = np.empty(1)
model = pyo.ConcreteModel()

# This function reads the instance file and extracts all the parameters from it.
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


# Objective function for our Pyomo model
def obj_function(model):
    return sum(model.f[j]*model.y[j] for j in model.J)+sum(model.t[(i,j)]*model.x[(i,j)] for i in model.I for j in model.J)

#First constraint of the FLP for our Pyomo model
def first_constraint_rule(model, j):
    return sum(model.x[(i,j)] for i in model.I) <= model.c[j]*model.y[j]
#Second constraint of the FLP for our Pyomo model
def sec_constraint_rule(model, i):
    return sum(model.x[(i,j)] for j in model.J) >= model.d[i]
#WHY are there parenthesis between the brackets?

# Objective function of the FLP for np arrays
def sec_function(x,y,t,f):
    return sum(f[j]*y[j] for j in range(len(y)))+sum(t[(i,j)]*x[(i,j)] for i in range(len(x)) for j in range(len(x[0])))


#This function converts the variables and parameters of the Pyomo environment to numpy arrays in order to later work
#more easily with them
def convertToNpArray(model): #x,y,obj,c,d,t,f,
    global c
    global d 
    global t 
    global f 
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
    return x,y,obj#,c,d,t,f


#This function solves the FLP with the GLPK solver.
#If linear==True, then it solves the LP relaxation of the problem.
def solve_flp(instance_name,linear):
    start = time.time()
    opening_cost, demand, capacity, travel_cost = read_instance(instance_name)

    #model creation
    global model
    model = pyo.ConcreteModel()
    model.I = pyo.RangeSet(0,len(demand)-1)
    model.J = pyo.RangeSet(0,len(capacity)-1)

    model.f = pyo.Param(model.J,initialize=opening_cost,default=0)#f
    model.c = pyo.Param(model.J,initialize=capacity,default=0)#u??
    model.d = pyo.Param(model.I,initialize=demand,default=0)#d
    model.t = pyo.Param(model.I,model.J,initialize=travel_cost,default=0)#t

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
    x,y,obj = convertToNpArray(model)
    #print(obj, ", ", end-start, ", ", instance_name)
    return (obj,x,y)


# This function performs the greedy rounding algorithm
# to get a feasible initial solution from the LP relaxation solutions
def initial_solution_flp(instance_name):
    #GREEDY ALGORITHM
    start = time.time()
    obj, x, y = solve_flp(instance_name, True)
#------------ Start of the greedy rounding algorithm
    #Sorting  y in decreasing order
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
        # Sorting x_ij over the clients in decreasing order (with j fixed)
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
            #return(obj,x_bar,y_bar,c,d,t,f)
            return(obj,x_bar,y_bar)
    #return (obj,x,y) #we'll always find a solution inside the for so we'll never get to that point. Useless to keep the return here


# This function checks that the condition at the end of the greedy rounding algorithm is respected.
def satisfying_cond(x_bar, d):
    j=0
    #We check wheter there is one customer i for which the condition is not respected.
    for i in range(len(d)):
        if sum(x_bar[i,:])<d[i]: #opposite condition of the algorithm
            j+=1 #the condition of the algorithm is not respected, we increase this counter
    if j!=0: #the condition is not respected for at least one customer
        return False
    else: #the condition of the algorithm was respected for all the customers
        return True

#This function allows to sort a vector in increasing or decreasing order
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


#This function performs the greedy reassignment. Notably, it takes as input the y & x
# that were modified by the local move
def greedy_reassign(y, x, sort_d, t, c, d):
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)
    temp = np.zeros(shape=(t.shape[1], 2)).astype('uint16')

    for i_p in range(x_bar.shape[0]):
        i=sort_d[i_p,1] #fixing i
        #Sorting the travel cost by increasing order
        temp[:,0] = t[i,:]
        temp[:,1] = [j for j in range(t.shape[1])]
        temp = temp[np.argsort(temp[:, 0])]

        for j_p in range(x_bar.shape[1]):
            j = temp[j_p,1] #fixing j

            if y_bar[j] ==1 and sum(x_bar[:,j])<c[j] and sum(x_bar[i,:])<d[i]:
                x_bar[i,j] = min(c[j]-sum(x_bar[:,j]), d[i]-sum(x_bar[i,:]))
    return y_bar, x_bar


# This algorithm takes as input the instance on which it has to find an initial solutions
# over which to perform a local search
def local_search_flp(x,y):

    t_end = 30*60#stopping time criterion
    t_1 = time.time()

    #Sorting d in decreasing order
    sort_d = sort_function(d, "decreasing")
    #Initialization of y_bar, x_bar
    y_bar, x_bar = copy.deepcopy(y), copy.deepcopy(x)

    y_best, x_best = copy.deepcopy(y_bar), copy.deepcopy(x_bar)# the best solutions y,x so far is the intiial solution
    obj_best = sec_function(x,y,t,f) # the best objective value we have so far is the one associated to the initial solution
    # The following counter will be used to count how many times we perform a search with a local move without observing any improvement
    counter_no_improvement=0
    max_no_improvement = 10# sets the limits of how many times to perfom a search with the same local move
    # The following counter will be used to count how many times we have changed the local move to perform in the local search without observing any improvement

    count_local_moves=0
    max_local_moves_no_improvement = 300# sets the limits of how many times change the local move to perform in the local search without observing any improvement
    #We set the original random seed to 0

    seed_original = 0
    #We begin the search with the assignment_movement
    move_facility=False
    move_assignment=True
    # We use a boolean value to assess if we need to continue the local search
    continue_search=True


    while(continue_search and count_local_moves<max_local_moves_no_improvement):
        random.seed(seed_original)
        seed= random.randrange(100000)

         #If it looks like we get stuck in a local minima, we change the local move in the hope to escape from it
        if counter_no_improvement>max_no_improvement:
            counter_no_improvement=0
            count_local_moves+=1
            move_facility = not move_facility
            move_assignment = not move_assignment

            if count_local_moves==max_local_moves_no_improvement:#if we really get stuck too many times in the local minima, we stop the search. However, we set the limit really high
                print("No more improvement with both movements, the search stops")

        if move_facility==True: # The local move to perform is the facility movement
            y_new, x_new = facility_movement(y_best, x_best, c, seed)
            if np.sum(y_new) < np.sum(y_bar)-x_bar.shape[1]//10:
                y_new, x_new = y_best, x_best
                move_facility = not move_facility
                move_assignment = not move_assignment

        if move_assignment==True: # The local move to perform is the assignment movement
            y_new, x_new = assignment_movement(y_best, x_best, c, seed)
            if y_new.all() == y_best.all() and x_new.all() == x_best.all(): # if we get exactly the same vectors, we change the local move to perform (not necessary)
                move_facility = not move_facility
                move_assignment = not move_assignment

        # We now have the local move performed on y & x. We then perform the greedy reassign algorithm
        y_new, x_new = greedy_reassign(y_new, x_new, sort_d, t, c, d)
        obj_new = sec_function(x_new,y_new,t,f)

        if (obj_new<obj_best): # The new solution seems to be better than the best one so far
            count_local_moves=0
            counter_no_improvement=0
            # The new solution must respect the constraints of the problem in order to be feasible.
            if respect_constraints(x_new, y_new, c, d):
                # the new solution respects the constraints, we update y_best, x_best and obj_best
                y_best, x_best = copy.deepcopy(y_new),copy.deepcopy(x_new)
                obj_best= sec_function(x_best,y_best,t,f)
                #print(obj_best)
        else:   # the new solution proposed by our algorithm doesn't respect the constraints of the problem
            counter_no_improvement+=1

        #Stop criterion
        t_2 = time.time()
        if t_end<t_2-t_1:
            continue_search=False# The time elapsed is above the threshold set for our search. We stop the search

        # We change the random seed for the next iteration of the local search
        seed_original+=1
    return(obj_best, x_best,y_best)

#This function applies the local move "facility movement" to the x,y solutions
def facility_movement(y, x,c, seed):
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)

    j1_p,j2_p, j1_m, j2_m = random_facility(x_bar, y_bar,c, seed)

    y_bar[j1_p], y_bar[j2_p] = 1, 1 #opens the facilities j1_p and j_2p
    y_bar[j1_m], y_bar[j2_m] = 0, 0 #closes the facilities j1_m and j_2m

    x_bar[:,j1_m] = 0 #j1_m is closed, so its corresponding values in x must be set to 0
    x_bar[:,j2_m] = 0#j2_m is closed, so its corresponding values in x must be set to 0

    return y_bar, x_bar

def assignment_movement(y,x,c,seed):
    random.seed(seed)
    y_bar = copy.deepcopy(y)
    x_bar = copy.deepcopy(x)
    j = random_assignment(x_bar, seed) #j contains i1 (and may be i2) and 2 facilities associated to each client
    # j is of the form: [[i1,j11,j12], [i2,j21,j22]]
    if j == []:
        return y_bar, x_bar

    for k in range (j.shape[0]): #for each client

        sum_xij_for_2_facilities=0
        sum_xij_for_2_facilities = x_bar[j[k,0],j[k,1]] + x_bar[j[k,0],j[k,2]]
        # We redistribute randomly the sum_xij_for_2_facilities over the 2 facilities
        first_new_xij = random.randrange(0, sum_xij_for_2_facilities)
        second_new_xij= sum_xij_for_2_facilities - first_new_xij
        x_bar[j[k,0],j[k,1]] = first_new_xij
        x_bar[j[k,0],j[k,2]] = second_new_xij
    return y_bar, x_bar

# This function checks that the new solutions y, x proposed respect the constraints of the FLP
def respect_constraints(x,y,c,d):
    const1 = [sum(x[i,:])>=d[i] for i in range(x.shape[0])]
    const2 = [sum(x[:,j])<=c[j]*y[j] for j in range(x.shape[1])]
    if np.all(const1) and np.all(const2):
        return True
    else:
        return False

# This function finds up to 2 facilities to open and up to 2 facilities to close (for facility_movement)
def random_facility(x_bar,y_bar, c, seed):
    i=seed
    random.seed(i)
    j1_p = random.randrange(x_bar.shape[1])
    j2_p = random.randrange(x_bar.shape[1])
    j1_m = random.randrange(x_bar.shape[1])
    j2_m = random.randrange(x_bar.shape[1])
    #although not explicit, due to the random behaviour, it will randomly choose between 0 and 2 facilities to open and to close
    while sum(x_bar[:,j1_m]+x_bar[:,j2_m])>c[j1_p]+c[j2_p]:
        i+=1
        random.seed(i)
        j1_p = random.randrange(x_bar.shape[1])
        j2_p = random.randrange(x_bar.shape[1])
        j1_m = random.randrange(x_bar.shape[1])
        j2_m = random.randrange(x_bar.shape[1])
    return j1_p, j2_p, j1_m, j2_m

# This function finds between 1 and 2 clients and associates 2 facilities to each one of them (for the assignment_movement)
def random_assignment(x_bar, seed):
    s=seed
    np.random.seed(s)
    random.seed(s)
    nb_customers = random.randrange(1,3) #randomly chooses 1 or 2 clients
    nb_facilities = 2 # always takes 2 facilities (if we take 0 or 1 there would later be nothing to reassign)

    # /!\ In order to respect the condition of the assignement movement, we make sure to only select non-zero x_ij
    # For that, we will list all the valid i values (and later all the valid j values)
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
    # Given how we selected the customers and the facilities, we are sure that all their x_ij associated are non-zero
    return j


if __name__ == '__main__':
    instance = sys.argv[1]
    local_search_flp(instance)

from datetime import datetime # delete in future versions
import math
import numpy as np
from scipy.integrate import quad

def initTrajectory(mode):
    ## Waypoints
    if mode:
        fileName = 'trajectories/' + datetime.now().strftime('%y%m%d_%H%M') + '.traj' # change when GUI is implemented
        dataStart, dataWaypoints = userInputTrajectory()
        ## Waypoints: ID x, y, v, a, theta, kappa
        waypoints = createWaypoints(dataStart, dataWaypoints)
        # saves trajectory
        np.savetxt(fileName, waypoints, delimiter=',') 
    else:
        fileName = 'trajectories/'+ str(np.loadtxt('init/load.traj_conf', dtype=str))
        waypoints = np.loadtxt(fileName, delimiter=',', dtype=float)

    return waypoints, fileName

def userInputTrajectory():
    finishFlag = False
    # index of last input
    i_k = 0

    # start point and orientation
    dataStart = np.zeros(3)
    print('Wählen Sie den gewünschten Anfangspunkt und Kurswinkel')
    printMsg = 'Geben Sie die x-Koordinate in m: '
    dataStart[0] = checkInputValue(printMsg, 'float')
    printMsg = 'Geben Sie die y-Koordinate in m: '
    dataStart[1] = checkInputValue(printMsg, 'float')
    printMsg = 'Geben Sie den Kurswinkel in Grad: '
    dataStart[2] = math.radians(checkInputValue(printMsg, 'float'))
    print('\n')
    # ID, length, width, velocity, acceleration
    dataWaypoints = np.array([0, 0.0, 0.0, 0.0, 0.0])
    # loop for creating the trajectory 
    while not finishFlag:
        # gets previous velocity 
        if i_k == 0:
            v_k = dataWaypoints[3]
        else:
            v_k =dataWaypoints[i_k,3]
        print('Geschwindigkeit im vorherigen Abschnitt: v=', round(v_k*3.6, 4), 'in km/h\nMögliche Trajektorienabschnitt:\n1: Gerade mit Beschleunigung\n2: Gerade mit konstanter Geschwindigkeit\n3: Spurwechsel')
        # verify that input is of type integer
        printMsg = 'Wählen Sie den gewünschten Trajektorienabschnitt in den Sie die entsprechende Nummer angeben: '
        inputID = checkInputValue(printMsg, 'int')
        # section selection
        match inputID:
            case 1:
                print('Gewählter Trajektorienabschnitt: Gerade mit Beschleunigung')
                # verify that input is of type integer
                printMsg = 'Geben Sie die gewünschte Beschleunigung (a) in m/s als absoluten Wert: '
                a = abs(checkInputValue(printMsg, 'float'))
                printMsg = 'Geben Sie die gewünschte Endgeschwindigkeit (v) in km/h als absoluten Wert: '
                v = abs(checkInputValue(printMsg, 'float'))
                newSection = [1, 0.0, 0.0, v/3.6, a]
            case 2:
                if abs(v_k) < 0.001:
                    print('Fügen Sie ein Abschnitt mit Beschleunigung')
                    newSection = [0, 0, 0, 0, 0]
                else: 
                    print('Gewählter Trajektorienabschnitt: Gerade mit konstanter Geschwindigkeit')
                    printMsg = 'Geben Sie die gewünschte Länge (l) in m als absoluten Wert: '
                    l = abs(checkInputValue(printMsg, 'float'))
                    newSection = [2, l, 0.0, v_k, 0.0]
            case 3:
                if abs(v_k) < 0.001:
                    print('Fügen Sie ein Abschnitt mit Beschleunigung')
                    newSection = [0, 0, 0, 0, 0]
                else:    
                    print('Gewählter Trajektorienabschnitt: Spurwechsel')
                    printMsg = 'Geben Sie die gewünschte Länge (l) in m als absoluten Wert: '
                    l = abs(checkInputValue(printMsg, 'float'))
                    printMsg = 'Geben Sie die gewünschte Breite (b) in m als absoluten Wert: '
                    b = abs(checkInputValue(printMsg, 'float'))
                    newSection = [3, l, b, v_k, 0.0]
            case _:
                print('Die angegebene Nummer ist nicht korrekt!')
                newSection = [0, 0, 0, 0, 0]
            
        dataWaypoints, i_k = checkTrajectory(dataWaypoints, newSection, i_k)
        print('Wollen Sie weitere Abschnitte hinzufügen?\nj: für weiter  || n: für  fertig')
        printMsg = 'Eingabe:'
        inputStr = checkInputValue(printMsg, 'str')
        if not inputStr == 'j':
            finishFlag = True

    # return trajectory sections
    return dataStart, dataWaypoints

def checkInputValue(printMsg, valueType):
    breakCondition = False
    if valueType == 'str':
        breakCounter = 1
    else:
        breakCounter = 0
    # checks format of input    
    while not breakCondition: 
        try:
            if valueType == 'int':
                inputValue = int(input(printMsg))
            elif valueType == 'float':
                inputValue = float(input(printMsg))
            elif valueType == 'str':
                inputValue = str(input(printMsg))
            breakCondition = True
        except ValueError:
            print('Eingabe nicht korrekt!')
            if breakCounter < 5:
                breakCounter += 1
            else:
                inputValue = int(0)
                breakCondition = True
        # checks if string contains the correct data        
        if valueType == 'str' and not (inputValue == 'j' or inputValue == 'n'):
            print('Eingabe nicht korrekt!')
            if breakCounter < 5:
                breakCounter += 1
                breakCondition = False
            else:
                inputValue = 'n'
                
    return inputValue

def checkTrajectory(traj, newSection, i):
    printMsg = 'Abschnitt wird nicht hinzugefügt! Eingaben nicht korrekt'
    addNewSection = True
    match newSection[0]:
        case 1:
            if newSection[4] == 0.0:
                print(printMsg)
                addNewSection = False
        case 2:
            if newSection[1] == 0.0:
                print(printMsg)
                addNewSection = False
        case 3:
            if newSection[1] == 0.0 or newSection[2] == 0.0:
                print(printMsg)
                addNewSection = False
    if addNewSection:
        traj = np.vstack([traj, newSection])
        i += 1
    return traj, i

def createWaypoints(dataStart, dataWaypoints):
    dx, dy = transformPoints(dataStart[2], -1, 0)
    # waypoint definition: Index, time, x, y, v, a, theta, kappa
    waypoints = np.array([1, 0, dataStart[0] + round(dx,5), dataStart[1] + round(dy,5), 0.01, 0.01, dataStart[2], 0])
    waypoints = np.vstack((waypoints,np.array([2, 0, dataStart[0], dataStart[1], 0.01, 0.01, dataStart[2], 0])))
    index_i = waypoints[-1,0]; t_i = waypoints[-1,1]; x_i = waypoints[-1,2]; y_i = waypoints[-1,3]; v_i = waypoints[-1,4]; theta_i = waypoints[-1,6]

    for i in range(1, len(dataWaypoints)):
        match dataWaypoints[i,0]:
            case 1:
                if v_i <= 0.01:
                    v_i = 0
                dv = (dataWaypoints[i,3] - v_i)
                if dv < 0:
                    a = -dataWaypoints[i,4]
                else:
                    a = dataWaypoints[i,4]
                t = dv/a
                s_f = 0.5*a*t**2 + v_i*t
                x = np.append(np.arange(0, s_f + 1, 1), s_f)
                # removes penultimum number of vector if distance is to small
                if np.diff(x[-2:]) < 0.7:
                    x = np.delete(x, -2)    
                # generates the time profile
                time = -v_i/a + np.sqrt(v_i**2 - 2*a*-x)/a
                # generates the velocity profile
                v = a*time + v_i
                # creates x, y points with right direction
                x, y = pointsForStraight(theta_i, x[1:], x_i, y_i)
                # stores data in waypoints array
                waypoints = createWaypointsForStraight(waypoints, time, x, y, v[1:], index_i, t_i, a, theta_i)
            case 2:
                s_f = dataWaypoints[i,1]
                x = np.append(np.arange(0, s_f + 1, 1), s_f)
                if np.diff(x[-2:]) < 0.7:
                    x = np.delete(x, -2)
                # generates the time profile
                time = x/v_i  
                x, y = pointsForStraight(theta_i, x[1:], x_i, y_i)
                v = np.ones(len(x))*v_i
                # stores data in waypoints array
                waypoints = createWaypointsForStraight(waypoints, time, x, y, v, index_i, t_i, 0, theta_i)
            case 3:
                stateInital = np.array([0, 0, 0.0, 0.0]); stateGoal = np.array([dataWaypoints[i,1], dataWaypoints[i,2], 0.0, 0.0])
                time, x, y, theta, kappa, v = caclulateTrajectory(stateInital, stateGoal, v_i)
                # transforms calculated values
                x = np.delete(x, 0); y = np.delete(y, 0)
                for i in range(len(x)):
                    x[i], y[i] = np.round(transformPoints(theta_i, x[i], y[i]),5)
                x = x + x_i;  y = y + y_i    
                theta = theta[1:] + theta_i
                # stores the data in the waypoint array
                index = np.arange(1, len(x) + 1) + index_i; a = np.zeros(len(x))
                waypoints = np.vstack((waypoints,np.stack((index, time[1:] + t_i, x, y, v[1:], a, theta, kappa[1:]), axis=1)))
        # update values
        index_i = waypoints[-1,0]; t_i = waypoints[-1,1]; x_i = waypoints[-1,2]; y_i = waypoints[-1,3]; v_i = waypoints[-1,4]; theta_i = waypoints[-1,6]

    return waypoints    

def pointsForStraight(alpha, x, x_i, y_i):
    x_t = np.zeros(len(x))
    y_t  = np.zeros(len(x)) 
    for i in range(len(x)):
        x_t[i], y_t[i] = transformPoints(alpha, x[i], 0)
    # moves points to the right coordinates
    x = x_t + x_i;  y = y_t + y_i
    return x, y

def transformPoints(alpha, x, y):
    R_z = np.array([[math.cos(alpha),-math.sin(alpha)],
                    [math.sin(alpha), math.cos(alpha)]])
    x, y = np.round(R_z.dot([x, y]),5)
    
    return x, y

def createWaypointsForStraight(waypoints, time, x, y, v, index_i, t_i, a_i, theta_i):
    # creates the parameters of the waypoint of the section
    index = np.arange(1, len(x) + 1) + index_i; time = time[1:] + t_i
    a = np.ones(len(x))*a_i; theta = np.ones(len(x))*theta_i; kappa = np.zeros(len(x))
    
    # stores the data in the waypoint array
    waypoints = np.vstack((waypoints,np.stack((index, time, x, y, v, a, theta, kappa), axis=1)))
    return waypoints
  
# calculate path for constains points
def caclulateTrajectory(stateInital, stateGoal, v):
    # move points to the origin 
    stateGoal = stateGoal - stateInital

    # iteration needed for high angles due to liniarization
    stateGoalEnd = np.copy(stateGoal)
    if stateGoal[2] >= math.pi/5:
        stateGoal[2] = math.pi/5

    # heuristic initial values for iteration
    a = stateInital[3]
    s = math.sqrt(stateGoal[0]**2 + stateGoal[1]**2)*(stateGoal[2]**2/5 + 1) + 2/5*abs(stateGoal[2])
    b = 6*stateGoal[2]/s**2 + 2*(2*stateGoal[3] - stateInital[3])/s
    c = 6*stateGoal[2]/s**3 + 3*(2*stateInital[3] - stateGoal[3])/s**2
    d = 0

    # error toleranze between goal and calculeted values
    tolerIter = np.array([0.1, 0.1, 0.001, 0.1, 2000])
    while stateGoalEnd[2] >= stateGoal[2]:
        deltaP_hat = np.array((b, c, d, s))
        # initial goal state for heuristic values
        stateP = calcGoalWithParam(a, b, c, d, s)
        # error expected between state with iterated values and goal state
        deltaX = stateGoal - stateP
        
        #number of iterations
        numIte = 1
        while any(abs(deltaX) > tolerIter[0:4]) & bool(numIte <  tolerIter[4]):
            # Jacobian Matrix
            H = calcJacobian(a, b, c, d, s, stateGoal[2])
            deltaP_til = np.linalg.inv(H).dot(deltaX.transpose())
            deltaP_hat += deltaP_til
            # update state
            b = deltaP_hat[0]; c = deltaP_hat[1]; d = deltaP_hat[2]; s = deltaP_hat[3]
            stateP = calcGoalWithParam(a, b, c, d, s)
            # error expected between state with iterated values and goal state
            deltaX = stateGoal - stateP
            # increases iteration counter
            numIte += 1
        # increases yaw angle
        stateGoal[2] += 1E-4
    
    # create waypoints
    x = np.zeros(math.ceil(s/min(max((v/3.6*0.3),0.7),4))); y = np.copy(x); theta = np.copy(x); dtheta = np.copy(x)
    s_points = np.linspace(0, s, num=len(x))

    for i in range(len(x)):
        integral = quad(f_cos, 0, s_points[i], args=(a, b, c, d, 0))
        x[i] = integral[0]
        integral = quad(f_sin, 0, s_points[i], args=(a, b, c, d, 0))
        y[i] = integral[0]
        theta[i] = a*s_points[i] + 1/2*b*s_points[i]**2 + 1/3*c*s_points[i]**3 + 1/4*d*s_points[i]**4
        dtheta[i] = a + b*s_points[i] + c*s_points[i]**2 + d*s_points[i]**3
    # move points to right coordinates
    x = stateInital[0] + x; y = stateInital[1] + y
    # generates time profile
    time = s_points/v
    # generates velocity profile
    v = np.ones(len(x))*v 

    #plot to visualize the path
    #plt.scatter(x,y, marker='o')
    #plt.show()

    return  time, x, y, theta, dtheta, v

def calcGoalWithParam(a, b, c, d, s):
    x_p = quad(f_cos, 0, s, args=(a, b, c, d, 0))
    y_p = quad(f_sin, 0, s, args=(a, b, c, d, 0))
    theta_p = a*s + 1/2*b*s**2 + 1/3*c*s**3 + 1/4*d*s**4
    kappa_p = a + b*s + c*s**2 + d*s**3
    sate_p = np.array(([x_p[0], y_p[0], theta_p, kappa_p]))

    return sate_p

# calculation of the Jacobian matrix
def calcJacobian(a, b, c, d, s, theta):
    # derivates of integral (az + 1/2bz + ...)*cos(az + 1/2bz + ...)
    S_2 = quad(f_sin, 0, s, args=(a, b, c, d, 2))
    S_3 = quad(f_sin, 0, s, args=(a, b, c, d, 3))
    S_4 = quad(f_sin, 0, s, args=(a, b, c, d, 4))
    # derivates of sin(az + 1/2bz + ...)
    C_2 = quad(f_cos, 0, s, args=(a, b, c, d, 2))
    C_3 = quad(f_cos, 0, s, args=(a, b, c, d, 3))
    C_4 = quad(f_cos, 0, s, args=(a, b, c, d, 4))
    # Jacobian Matrix
    H = np.array(([-1/2*S_2[0], -1/3*S_3[0], -1/4*S_4[0],    math.cos(theta)],
                  [ 1/2*C_2[0],  1/3*C_3[0],  1/4*C_4[0],    math.sin(theta)],
                  [ 1/2*s**2,    1/3*s**3,    1/4*s**4, a + b*s + c*s**2 + d*s**3],
                  [    s,           s**2,        s**3,    b + 2*c*s + 3*d*s**2]))

    return H

# integral for z^i*cos(az + 1/2bz + ...) function
def f_cos(z, a, b, c, d, i):

    return z**i*math.cos(a*z + 1/2*b*z**2 + 1/3*c*z**3 + 1/4*d*z**4)

# integral for z^i*sin(az + 1/2bz + ...) function
def f_sin(z, a, b, c, d, i):

    return z**i*math.sin(a*z + 1/2*b*z**2 + 1/3*c*z**3 + 1/4*d*z**4)


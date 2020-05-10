import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import rospy
import numpy as np

def pidTuner(time, position, velocity, acceleration, thrust, ratesMagn, Tf, fmin, fmax, wmax):
    idx = time>0
    time         = time[idx]
    position     = position[idx, :] 
    velocity     = velocity[idx, :] 
    acceleration = acceleration[idx, :]  
    thrust       = thrust[idx] 
    ratesMagn    =  ratesMagn[idx]  
    flag         = True

    figStates, axes = plt.subplots(3,1,sharex=True)
    gs = gridspec.GridSpec(6, 2)
    axPos = plt.subplot(gs[0:2, 0])
    axVel = plt.subplot(gs[2:4, 0])
    axAcc = plt.subplot(gs[4:6, 0])

    for ax,yvals in zip([axPos, axVel, axAcc], [position,velocity,acceleration]):
        cols = ['r','g','b']
        labs = ['x','y','z']
        for i in range(3):
            ax.plot(time,yvals[:,i],cols[i],label=labs[i])

        axPos.set_ylabel('Pos [m]')
        axVel.set_ylabel('Vel [m/s]')
        axAcc.set_ylabel('Acc [m/s^2]')
        axAcc.set_xlabel('Time [s]')
        axPos.legend()
        axPos.set_title('States')

        infeasibleAreaColour = [1,0.5,0.5]
        axThrust = plt.subplot(gs[0:3, 1])
        axOmega  = plt.subplot(gs[3:6, 1])
        axThrust.plot(time,thrust,'k', label='command')
        axThrust.plot([0,Tf],[fmin,fmin],'r--', label='fmin')
        axThrust.fill_between([0,Tf],[fmin,fmin],-1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
        axThrust.fill_between([0,Tf],[fmax,fmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
        axThrust.plot([0,Tf],[fmax,fmax],'r-.', label='fmax')

        axThrust.set_ylabel('Thrust [m/s^2]')
        axThrust.legend()

        axOmega.plot(time, ratesMagn,'k',label='command magnitude')
        axOmega.plot([0,Tf],[wmax,wmax],'r--', label='wmax')
        axOmega.fill_between([0,Tf],[wmax,wmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
        axOmega.set_xlabel('Time [s]')
        axOmega.set_ylabel('Body rates [rad/s]')
        axOmega.legend()

        axThrust.set_title('Inputs')

        axThrust.set_ylim([min(fmin-1,min(thrust)), max(fmax+1,max(thrust))])
        axOmega.set_ylim([0, max(wmax+1,max(ratesMagn))])

        rospy.sleep(5)
        flag = False
        # flag.data = False
        # flag_publisher.publish(flag)
        plt.savefig('pidTuner.png')
        # plt.show()
    return flag

def timeVsDist(time, dist):
    time = np.array(time)
    dist = np.array(dist)
    time = time - time[0]
    plt.plot(time, dist)
    plt.xlabel('time')
    plt.ylabel('Distance from ground(m)')
    plt.savefig('timeVsDist.png')
    print('exiting time dist graph')
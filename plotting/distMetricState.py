import numpy as np
import numpy.linalg

def distState_old(s1,s2):
    if s1[0]==s2[0]:
        pNorm = np.linalg.norm(np.array(s1[1])-np.array(s2[1]))
        vNorm = np.linalg.norm(np.array(s1[2])-np.array(s2[2]))
        return np.linalg.norm(np.array([pNorm,vNorm]))
    else:
        raise Exception("Can't return distance between states of " 
        "different model types")


def distState(s1,s2):
    if s1[0]==s2[0]:
        error = 0
        if s1[0] == 3:
            # the angle should be moded.
            # the distance should take that into account
            # ignore the first two parts of this state
            th1 = s1[1][2] % np.pi # put in range of 0 to pi
            th2 = s2[1][2] % np.pi # put in range of 0 to pi
            error = min(np.pi - np.abs(th1-th2),np.abs(th1-th2))
        elif s1[0] == 2:
            error = np.linalg.norm(np.array(s1[1])-np.array(s2[1]))
        elif s1[0] == 4:
            error = np.sqrt(s1[1][3]**2-s2[1][3]**2)
        else:
            pNorm = np.linalg.norm(np.array(s1[1])-np.array(s2[1]))
            vNorm = np.linalg.norm(np.array(s1[2])-np.array(s2[2]))
            error = np.linalg.norm(np.array([pNorm,vNorm]))
        return error
    else:
        raise Exception("Can't return distance between states of " 
        "different model types")

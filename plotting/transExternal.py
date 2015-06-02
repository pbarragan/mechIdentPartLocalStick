import subprocess

def simulate(state,action):

    # initialize the string
    callString = './transExternal.o '

    # add the actions
    for a in action:
        callString += str(a)+' '

    # add model
    callString += str(state[0])

    # add parameters
    for p in state[1]:
        callString += ' '+str(p)

    # add variables
    for v in state[2]:
        callString += ' '+str(v)
    
    proc = subprocess.Popen([callString],stdout=subprocess.PIPE,shell=True)
    (out, err) = proc.communicate()
    #print "program output:", out
    data = out.split('\n')

    m = int(data[0])
    if data[1] == '':
        ps = []
    else:
        ps = [float(s) for s in data[1].split(',')]
    if data[2] == '':
        vs = []
    else:
        vs = [float(s) for s in data[2].split(',')]
    o = [float(s) for s in data[3].split(',')]

    return o, [m, ps, vs]


import math

def angleDiff(a1,a2):
    diff = a1-a2
    if(diff>math.pi):
        diff -= 2*math.pi
    elif(diff<-math.pi):
        diff += 2*math.pi
    return diff    

def stateDist(s1,s2):
    if s1[0]!=s2[0]:
        return 100000000
    else:
        if s1[0] == 0 or s1[0] == 1:
            dist = 0
            count = 0.0
            for p1,p2 in zip(s1[1],s2[1]):
                dist += (p1-p2)**2
                count += 1.0
            for v1,v2 in zip(s1[2],s2[2]):
                dist += (v1-v2)**2
                count += 1.0
            return math.sqrt(dist/count)
        elif s1[0]==2:
            dist = (s1[1][0]-s2[1][0])**2 + (s1[1][1]-s2[1][1])**2 \
                    + (s1[1][2]-s2[1][2])**2 + (angleDiff(s1[2][0],s2[2][0]))**2
            #print angleDiff(s1[2][0],s2[2][0])
            return math.sqrt(dist/4.0)
        elif s1[0]==3:

            lengthPs = math.sqrt((s1[1][0]-s2[1][0])**2
                                 + (s1[1][1]-s2[1][1])**2)
            dist1 = (s1[1][0]-s2[1][0])**2 + (s1[1][1]-s2[1][1])**2 \
                    + (angleDiff(s1[1][2],s2[1][2]))**2 + (s1[2][0]-s2[2][0])**2
            dist2 = (s1[1][0]+s2[1][0])**2 + (s1[1][1]+s2[1][1])**2 \
                    + (angleDiff(s1[1][2],(s2[1][2]+math.pi)))**2 \
                    + (s1[2][0]-(lengthPs-s2[2][0]))**2
            dist3 = (s1[1][0]+s2[1][0])**2 + (s1[1][1]+s2[1][1])**2 \
                    + (angleDiff(s1[1][2],(s2[1][2]-math.pi)))**2 \
                    + (s1[2][0]-(lengthPs-s2[2][0]))**2

            dist = min([dist1,dist2,dist3])
            return math.sqrt(dist/4.0)

def stateDist2(s1,s2):
    if s1[0]!=s2[0]:
        return 100000000
    else:
        if s1[0] == 0 or s1[0] == 1:
            dist = 0
            count = 0.0
            for p1,p2 in zip(s1[1],s2[1]):
                dist += abs(p1-p2)
                count += 1.0
            for v1,v2 in zip(s1[2],s2[2]):
                dist += abs(v1-v2)
                count += 1.0
            return dist/count
        elif s1[0]==2:
            dist = abs(s1[1][0]-s2[1][0]) + abs(s1[1][1]-s2[1][1]) \
                    + abs(s1[1][2]-s2[1][2]) \
                    + abs(angleDiff(s1[2][0],s2[2][0]))/(math.pi)
            #print angleDiff(s1[2][0],s2[2][0])
            return dist/4.0
        
        ## elif s1[0]==4:
        ##     lengthPs = math.sqrt((s1[1][0]-s2[1][0])**2
        ##                          + (s1[1][1]-s2[1][1])**2)
        ##     dist1 = abs(angleDiff(s1[1][2],s2[1][2]))\
        ##             + abs(s1[2][0]-s2[2][0])
        ##     dist2 = abs(angleDiff(s1[1][2],(s2[1][2]+math.pi))) \
        ##             + abs(s1[2][0]-(lengthPs-s2[2][0]))
        ##     dist3 = abs(angleDiff(s1[1][2],(s2[1][2]-math.pi))) \
        ##             + abs(s1[2][0]-(lengthPs-s2[2][0]))

        ##     dist = min([dist1,dist2,dist3])
        ##     return dist/2.0
        
        elif s1[0]==3:
            aE = abs(angleDiff(s1[1][2]%math.pi,s2[1][2]%math.pi))
            x1 = s1[1][0]+s1[2][0]*math.cos(s1[1][2])
            y1 = s1[1][1]+s1[2][0]*math.sin(s1[1][2])            
            x2 = s2[1][0]+s2[2][0]*math.cos(s2[1][2])
            y2 = s2[1][1]+s2[2][0]*math.sin(s2[1][2])
            dE = math.hypot((x1-x2),(y1-y2))
            #print aE
            #print dE
            return (aE/math.pi+dE)/2.0

        elif s1[0]==4:

            # xp, yp, r, thL, dL
            # th, d
            dist = abs(s1[1][0]-s2[1][0]) + abs(s1[1][1]-s2[1][1]) \
                   + abs(s1[1][2]-s2[1][2]) + abs(s1[1][3]-s2[1][3]) \
                   + abs(s1[1][4]-s2[1][4]) \
                   + abs(angleDiff(s1[2][0],s2[2][0]))/(math.pi) \
                   + abs(s1[2][1]-s2[2][1])
            #print angleDiff(s1[2][0],s2[2][0])
            return dist/7.0        
        

def printTable(top,side,e,sd):
    row_format ="{:^15}"+"{:^21}" * len(top)
    row_format2 ="{:^15}"+"{:^10.3f}|{:^10.3f}" * (len(top))
    print row_format.format("", *top)
    for ty, row,rowSD in zip(side, e,sd):
        print row_format2.format(ty,\
                                 *[val for pair in \
                                   zip(row,rowSD) for val in pair])

def printTableNoSD(top,side,e):
    row_format ="{:^15}"+"{:^10}" * len(top)
    row_format2 ="{:^15}"+"{:^10}" * (len(top))
    print row_format.format("", *top)
    for ty, row in zip(side, e):
        print row_format2.format(ty,*row)

def printFlatTable(top,e,sd):
    row_format ="{:^21}" * len(top)
    row_format2 ="{:^10.6f}|{:^10.6f}" * (len(e))
    print row_format.format(*top)
    print row_format2.format(*[val for pair in \
                               zip(e,sd) for val in pair])


def printMisClassTable(top,side,m,mM):
    row_format ="{:^15}"+"{:^23}" * len(top)
    innerString=",".join(["{:^2}"]*len(top))
    row_format2 ="{:^15}"+("  {:^2} ("+innerString+")  ") * (len(top))
    print row_format.format("", *top)
    for ty, r,rM in zip(side,m,mM):
        print row_format2.format(ty,\
                                 *[val for pair in \
                                   [[x]+y for x,y in zip(r,rM)]\
                                   for val in pair])

#Matlab adaptation of code in python
#Blanchon Marc
# December 11, 2017
#
#Creation of switch case to accomplish detection algorithm of tags

import numpy as np
from operator import add


#---------------------------------------------------------------------------------------------------------------------------
# FUNCTION
#---------------------------------------------------------------------------------------------------------------------------

#define function to check ratio (See theory)
def check_ratio(b1, w1, b2, w2, b3):
    boolean_result = False

    input_val = np.array([b1, w1, b2, w2, b3])
    desired_val = np.array([1, 1, 3, 1, 1])

    minima = int(min(float(s) for s in input_val))

    input_val = input_val/minima
    tolerance = 0.7

    if ( np.linalg.norm(input_val - desired_val, 2) < tolerance):
        boolean_result = True
    else:
        boolean_result = False;

    return boolean_result


#---------------------------------------------------------------------------------------------------------------------------
# CASES
#---------------------------------------------------------------------------------------------------------------------------


#no black encountered
def case_zero(img , nrow , ncol , b1 , currState):
    if img[nrow, ncol] == 0:
        currState = 1
        b1 = b1+1
    return currState , b1


#inside first black
def case_one(img , nrow , ncol , b1 , w1 , currState):
    if img[nrow, ncol] == 1:
        currState = 2
        w1 = w1 + 1
    else:
        b1 = b1 + 1
    return currState , b1 , w1


#inside first white
def case_two(img , nrow , ncol , w1 , b2 , r0 , currState):
    if img[nrow, ncol] == 0:
        b2 = b2 + 1;
        currState = 3;
        r0 = np.array([nrow, ncol])
    else:
        w1 = w1 + 1
    return currState , w1 , b2 , r0

#inside second black
def case_three(img , nrow , ncol , b2 , w2 , currState):
    if img[nrow, ncol] == 1:
        w2 = w2 + 1
        currState = 4
    else:
        b2 = b2 + 1
    return currState, b2, w2

#inside second white
def case_four(img , nrow , ncol , w2 , b3 , r1 , r0 , currState , pastFirstTest):
    if img[nrow, ncol]== 0:
        b3 = b3 + 1
        currState = 5
        if not pastFirstTest:
            r1 = np.array([nrow, ncol])
        else:
            temp = r1
            r1 = np.array([nrow, ncol])
            r0 = temp

    else:
        w2 = w2 + 1
    return currState , pastFirstTest , w2 , b3 , r1 , r0


def case_five(img , nrow , ncol , b1 , w1 , b2 , w2 , b3 , r0 , currState , pastFirstTest , detection , endloc , startloc):
    if img[nrow, ncol] == 1:
        startloc = r0
        endloc = np.array([nrow, ncol])
        if check_ratio(b1, w1, b2, w2, b3):
            addition = map(add, startloc, endloc)
            addition = [addition[0]/2 , addition[1]/2]
            detection = np.vstack([detection, addition])

        b1 = b2
        w1 = w2
        b2 = b3
        w2 = 1
        b3 = 0
        currState = 4
        if not pastFirstTest:
            pastFirstTest = True

        endloc = np.array([0, 0])
    else:
        b3 = b3 + 1
    return currState , b1 , w1 , b2 , w2 , b3 , r0 , pastFirstTest , detection , endloc , startloc

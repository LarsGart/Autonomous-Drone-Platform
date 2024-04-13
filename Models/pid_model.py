'''
Author: Jerin Abraham

This class acts as a generic PID loop
It can be used for any PID purpose
'''
import numpy as np


def resetError():
    for member in PID.members:
        member.err, member.deltaErr, member.errSum, member.prevErr = 0, 0, 0, 0

class PID:
    members = []
    '''
    Initializes pid coefficients and errors

    PARAMETERS:
        kP (float): Proportional coefficient of the PID loop
        kI (float): Integral coefficient of the PID loop
        kD (float): Derivative coefficient of the PID loop
        limit (float): Absolute value of the maximum output from the PID loop
    '''
    def __init__(self, kP=0.0, kI=0.0, kD=0.0, limit=100.0):
        self.ctrlInput, self.worldInput, self.output = 0, 0, 0
        self.err, self.deltaErr, self.errSum, self.prevErr = 0, 0, 0, 0
        self.kP, self.kD, self.kI, self.limit = kP, kI, kD, limit
        PID.members.append(self)


    def __calcErr(self):
        self.err = self.worldInput - self.ctrlInput
        if self.kI > 0:
            self.errSum = np.clip(self.errSum + self.err, -self.limit/self.kI, self.limit/self.kI)
        self.deltaErr, self.prevErr = self.err - self.prevErr, self.err
        

    def __calcPID(self):
        self.output = np.clip(self.kP*self.err + self.kI*self.errSum + self.kD*self.deltaErr, -self.limit, self.limit)


    '''
    Calculate PID output from desired value and measured world value

    PARAMETERS:
        ctrlInput (float): Desired world value
        worldInput (float): Measured world value

    RETURNS:
        (float): pid output
    '''
    def calc(self, ctrlInput, worldInput):
        self.ctrlInput, self.worldInput = ctrlInput, worldInput
        self.__calcErr()
        self.__calcPID()
        return self.output
    
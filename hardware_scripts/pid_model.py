import numpy as np

'''
Class to instantiate a PID object
'''
class PID():
    '''
    Constructor for PID class
    Initializes the pid coefficients and errors

    PARAMETERS:
        kP (float): Proportional coefficient of the PID loop
        kI (float): Integral coefficient of the PID loop
        kD (float): Derivative coefficient of the PID loop
        limit (float): Absolute value of the maximum output from the PID loop
    '''
    def __init__(self, kP = 0, kI = 0, kD = 0, limit = 100):
        self.ctrlInput = 0
        self.worldInput = 0
        self.output = 0

        self.err = 0
        self.deltaErr = 0
        self.errSum = 0
        self.prevErr = 0

        self.kP = kP
        self.kD = kI
        self.kI = kD

        self.limit = limit

    def __calcErr(self):
        # Calculate P error
        self.err = self.worldInput - self.ctrlInput

        # Calculate I error if kI is greater than 0
        if (self.kI > 0):
            errSum = np.clip(
                self.errSum + self.err,
                -self.limit / self.kI,
                self.limit / self.kI
            )

        # Calculate D error and update previous error
        self.deltaErr = self.err - self.prevErr
        self.prevErr = self.err
        
    def __calcPID(self):
        # 
        self.output = np.clip(
            self.kP * self.err + self.kI * self.errSum + self.kD * self.deltaErr,
            -self.limit,
            self.limit
        )
    
    '''
    Reset errors
    '''
    def resetError(self):
        self.err = 0
        self.deltaErr = 0
        self.errSum = 0
        self.prevErr = 0

    '''
    Calculate PID output from desired value and measured world value

    PARAMETERS:
        ctrlInput (float): Desired world value
        worldInput (float): Measured world value

    RETURNS:
        (float) pid output
    '''
    def calc(self, ctrlInput, worldInput):
        self.ctrlInput = ctrlInput
        self.worldInput = worldInput

        self.__calcErr()
        self.__calcPID()

        return self.output
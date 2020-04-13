import servoControl
import numpy as np

class ballController:
    def __init__(self):

        self.err_max = 400.0

        self.kP_x = 1.1*.5
        self.kI_x = 1.5*.1
        self.kD_x = 1.5*.15

        self.kP_y = 1.5*.35
        self.kI_y = 1.5*.1
        self.kD_y = 1.5*.2

        self.integ_x = 0.
        self.deriv_x = 0.
        self.erkm1_x = 0.

        self.integ_y = 0.
        self.deriv_y = 0.
        self.erkm1_y = 0.

        self.sigma = .1
        self.vbar = .001
        self.sat_lim = .3

    def update(self, position, waypoint, Ts):

        # calculate error and have it be on the order of 10^0
        err_x = (float(waypoint[0]) - position[0])/self.err_max
        err_x = self.sat(err_x, .2)
        err_y = (float(waypoint[1]) - position[1])/self.err_max
        err_y = self.sat(err_y, .2)

        print("err_x = {}, erry_y = {}".format(err_x, err_y))
        
        a1 = (2*self.sigma-Ts)/(2*self.sigma+Ts)
        a2 = 2/(2*self.sigma+Ts)

        self.deriv_x = a1*self.deriv_x + a2*(err_x - self.erkm1_x)
        self.deriv_y = a1*self.deriv_y + a2*(err_y - self.erkm1_y)

        if abs(self.deriv_x) < self.vbar:
            self.integ_x += (Ts/2)*(err_x + self.erkm1_x)
        else:
            self.integ_x = 0.0

        if abs(self.deriv_y) < self.vbar:
            self.integ_y += (Ts/2)*(err_y + self.erkm1_y)
        else:
            self.integ_y = 0.0

        u_unsat_x = self.kP_x*err_x + self.kI_x*self.integ_x + self.kD_x*self.deriv_x
        print("p_x = {},\t\ti_x = {},\t\td_x = {}".format(self.kP_x*err_x, self.kI_x*self.integ_x, self.kD_x*self.deriv_x))
        u_unsat_y = self.kP_y*err_y + self.kI_y*self.integ_y + self.kD_y*self.deriv_y
        print("p_y = {},\t\ti_y = {},\t\td_y = {}".format(self.kP_y*err_y, self.kI_y*self.integ_y, self.kD_y*self.deriv_y))

        u_sat_x = self.sat(u_unsat_x, self.sat_lim)
        u_sat_y = self.sat(u_unsat_y, self.sat_lim)

        if not self.kI_x == 0.0:
            self.integ_x = self.integ_x + (1/self.kI_x) * (u_sat_x - u_unsat_x)

        if not self.kI_y == 0.0:
            self.integ_y = self.integ_y + (1/self.kI_y) * (u_sat_y - u_unsat_y)

        self.erkm1_x = err_x
        self.erkm1_y = err_y

        return u_sat_x, u_sat_y

    def sat(self, u, lim):
        if u > lim:
            return lim
        if u < -lim:
            return -lim 
        return u


            
        




#! /usr/bin/env python3
import serial
import time

RANGE_MAX = 1650
RANGE_MIN = 1350

TOP_RANGE_OFFSET = -150
BOT_RANGE_OFFSET = 0

class ServoControl:
    def __init__(self, top_center=0.0, bot_center=0.0, port='/dev/ttyUSB0', baud=115200):
        try:
            self.ser = serial.Serial(port, baud)
            self.ser_valid = True
        except serial.SerialException:
            print("ServoControl: Didn't find serial device, commands will be printed rather than sent")
            self.ser_valid = False
            
        self.top_center = top_center
        self.bot_center = bot_center
        
    def _writeCommand_us(self, top_cmd_us, bot_cmd_us):
        # top_cmd_us and bot_cmd_us are in microseconds as the arduino writes to the servos
        if(top_cmd_us < RANGE_MIN + TOP_RANGE_OFFSET or top_cmd_us > RANGE_MAX + TOP_RANGE_OFFSET
                or bot_cmd_us < RANGE_MIN + BOT_RANGE_OFFSET or bot_cmd_us > RANGE_MAX + BOT_RANGE_OFFSET):
            print("ServoControl._writeCommand_us Error: command out of range")
            return
        else:
            cmd_string = "{},{}\n".format(int(top_cmd_us), int(bot_cmd_us))
            if self.ser_valid:
                self.ser.write(cmd_string.encode())
            else:
                print("SerialControl writing: ", cmd_string)
            return

    def writeCommand(self, top_cmd, bot_cmd):
        # top_cmd and bot_cmd are both between -1.0 and 1.0
        # convert top command to us
        top_cmd_us = 1500 + (top_cmd + self.top_center)*(RANGE_MAX - RANGE_MIN)/2 + TOP_RANGE_OFFSET
        bot_cmd_us = 1500 + (bot_cmd + self.bot_center)*(RANGE_MAX - RANGE_MIN)/2 + BOT_RANGE_OFFSET
        self._writeCommand_us(top_cmd_us, bot_cmd_us)
        return


if __name__=="__main__":
    import numpy as np
    print("Testing ServoControl")
    sc = ServoControl()

    # from 0 to 1 to -1 to 0
    steps = np.concatenate((np.linspace(0, -1, 20), np.linspace(-1, 1, 40), np.linspace(1, 0, 20)))
    for i in steps:
        sc.writeCommand(i, 0.0) 
        time.sleep(.2)
        
    for i in steps:
        sc.writeCommand(0.0, i) 
        time.sleep(.2)


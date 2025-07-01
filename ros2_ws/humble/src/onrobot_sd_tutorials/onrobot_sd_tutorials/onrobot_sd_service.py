#!/usr/bin/env python3

import time
import xmlrpc.client

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from onrobot_interfaces.srv import SetCommand

CONN_ERR = -2
RET_OK = 0
RET_FAIL = -1

SCREW_DRIVER_ID = 0
BIT_EXTENDER = 100
IP_ADDRESS = "192.168.1.1"


class OnRobotSDServer(Node):

    def __init__(self):
        super().__init__('onrobot_sd_command_server')

        self.err_h = [True, True, True]

        try:
            self.cb = xmlrpc.client.ServerProxy("http://" + IP_ADDRESS + ":41414/")
        except TimeoutError:
            self.get_logger().info("Connection to ComputeBox failed!")

        self.sd_id = SCREW_DRIVER_ID
        self.bit_extender_length = BIT_EXTENDER

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/onrobot_sd/joint_states', qos_profile)

        self.move_shank_srv = self.create_service(
            SetCommand,
            "/onrobot_sd/move_shank",
            self.onrobot_sd_move_shank)

        self.min_shank_pos = 0 + self.bit_extender_length
        self.max_shank_pos = 55 + self.bit_extender_length

    def isconn(self, t_index):
        '''
        Returns with True if Screw driver is connected, False otherwise

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: True if connected, False otherwise
        @rtype: bool
        '''
        isSDConn = self.cb.cb_is_device_connected(t_index, 0x80)
        if not isSDConn:
            self.get_logger().info("No Screw driver connected")
            return False
        else:
            return True

    def setErrhON(self, t_index):
        '''
        Turns ON error handling for all screwdriver commands
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        '''
        self.err_h[t_index] = True

    def setErrhOFF(self, t_index):
        '''
        Turns OFF error handling for all screwdriver commands
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        '''
        self.err_h[t_index] = False

    def getErrh(self, t_index):
        '''
        Gets if error handling is turned ON or OFF for the given instance

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: True if error handling is turned on for this instance, False if turned off
        @rtype: bool
        '''
        return self.err_h[t_index]

    def _err_handler(self, t_index):
        '''
        Checks ant interprets command result and erro code

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @return: True if error, False otherwise
        @rtype: bool
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        err_code = self.cb.sd_get_error_code(t_index)
        cmd_result = self.cb.sd_get_command_results(t_index)

        init_err_mask = 0xF0
        retval = True

        # Check the error code
        if err_code != 0:
            if err_code & 0x04 != 0:
                self.get_logger().info("Screw driver saftey circuit triggered")
            if err_code & 0x08 != 0:
                self.get_logger().info("Screw driver not calibrated")

            init_err = err_code & init_err_mask

            # Check init errors
            if init_err == 0x10:
                self.get_logger().info("Screw driver init error: Shank stall current not reached")
            elif init_err == 0x20:
                self.get_logger().info("Screw driver init error: No shank index mark found")
            elif init_err == 0x30:
                self.get_logger().info("Screw driver init error: Unable to home shank")
            elif init_err == 0x40:
                self.get_logger().info("Screw driver init error: Invalid shank index placement")
            elif init_err == 0x50:
                self.get_logger().info("Screw driver init error: No torque index mark found")
            elif init_err == 0x60:
                self.get_logger().info("Screw driver init error: Torque difference overflow")
            elif init_err == 0x70:
                self.get_logger().info("Screw driver init error: Index mark value has changed (clean encoder disk)")

            if err_code & 0x100:
                self.get_logger().info("Wrong Quick changer type for the Screw driver")
            if err_code & 0x200:
                self.get_logger().info("Wrong Power Supply Type for the screw driver")
        else:
            # No error
            retval = False

        # Check command result
        cmd_res_msg = None
        if cmd_result != 0:
            retval = True
            # Interpret command result
            if cmd_result == 1:
                cmd_res_msg = "Unknown command"
            elif cmd_result == 2:
                cmd_res_msg = "Not screwing in"
            elif cmd_result == 3:
                cmd_res_msg = "Timeout waiting for torque"
            elif cmd_result == 4:
                cmd_res_msg = "Torque exceeded prematurely"
            elif cmd_result == 5:
                cmd_res_msg = "Unable to loosen screw"
            elif cmd_result == 6:
                cmd_res_msg = "Shank reached the end"
            elif cmd_result == 7:
                cmd_res_msg = "Shank obstructed during move"
            else:
                cmd_res_msg = "Unknown command result"

            self.get_logger().info("Screwdriver command result: " + cmd_res_msg)
        else:
            retval = False

        return retval

    def isBusy(self, t_index):
        '''
        Gets if the screw driver is busy or not

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: bool
        @return: True if busy, False otherwise
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        shank_busy = self.cb.sd_get_shank_busy(t_index)
        dev_busy = self.cb.sd_get_screwdriver_busy(t_index)

        if ((not shank_busy) and (not dev_busy)):
            return False
        else:
            return True

    def get_torque_grad(self, t_index):
        '''
        Gets the torque gradient result

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: float
        @return: Torque gradient
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        return self.cb.sd_get_torque_gradient(t_index)

    def get_shank_pos(self, t_index):
        '''
        Gets the current shank position

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: float
        @return: Shank position in mm
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        return self.cb.sd_get_shank_position(t_index)

    def get_force(self, t_index):
        '''
        Gets the current force

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: float
        @return: Current force in N
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        return self.cb.sd_get_force(t_index)

    def get_ach_torq(self, t_index):
        '''
        Gets the achieved torque

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: float
        @return: Achieved torque in Nm
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        return self.cb.sd_get_achieved_torque(t_index)

    def get_curr_torq(self, t_index):
        '''
        Gets the current torque

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: float
        @return: Current torque in Nm
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

        return self.cb.sd_get_current_torque(t_index)

    def tighten(self, t_index, force, screw_len, torq, f_wait):
        '''
        Starts a screw tighten command

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param force:  Screw in force in N (18-30)
        @type force: int
        @param screw_len: Screwing lenght in mm (0-35)
        @type screw_len: float
        @param torq: Screw in torque in Nm (0-5)
        @type torq: float
        @param f_wait: Wait for command to finish or not?
        @type f_wait: bool
        '''

        if self.isconn(t_index) is False:
            return CONN_ERR

        # Sanity check
        if force < 18 or force > 30:
            self.get_logger().info("Invalid force parameter for tighten command, valid range: 18-30")
            return RET_FAIL

        if screw_len < 0.0 or screw_len > 35.0:
            self.get_logger().info("Invalid screw length for tighten command, valid range: 0-35")
            return RET_FAIL

        if torq < 0.0 or torq > 5.0:
            self.get_logger().info("Invalid torque parameter for tighten command, valid range: 0-5")
            return RET_FAIL

        self.cb.sd_tighten(t_index, int(force), float(screw_len), float(torq))

        timeout = False
        if f_wait:
            busy_cnt = 0
            f_busy = self.isBusy(t_index)
            while (f_busy):
                time.sleep(0.1)
                f_busy = self.isBusy(t_index)
                busy_cnt += 1
                if busy_cnt > 300:
                    self.get_logger().info("Screw driver tighten command timeout")
                    timeout = True
                    break
            else:
                timeout = False

        # Check for error
        if self.err_h[t_index]:
            err_state = self._err_handler(t_index)
            # There was no error and no timeout
            if (err_state == False) and (timeout == False):
                return RET_OK
            else:
                return RET_FAIL
        # There was no error handling only check timeout
        else:
            if timeout:
                return RET_FAIL
            else:
                return RET_OK

    def loosen(self, t_index, force, screw_len, f_wait):
        '''
        Starts a screw loosening command

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param force:  Screw loosen force in N (18-30)
        @type force: int
        @param screw_len: Length of the screw in mm (0-35)
        @type screw_len: float
        @param f_wait: Wait for command to finish or not?
        @type f_wait: bool
        '''

        if self.isconn(t_index) is False:
            return CONN_ERR

        # Sanity check
        if force < 18 or force > 30:
            self.get_logger().info("Invalid force parameter for loosen command, valid range: 18-30")
            return RET_FAIL

        if screw_len < 0.0 or screw_len > 35.0:
            self.get_logger().info("Invalid screw length for loosen command, valid range: 0-35")
            return RET_FAIL

        self.cb.sd_loosen(t_index, int(force), float(screw_len))

        timeout = False
        if f_wait:
            busy_cnt = 0
            f_busy = self.isBusy(t_index)
            while (f_busy):
                time.sleep(0.1)
                f_busy = self.isBusy(t_index)
                busy_cnt += 1
                if busy_cnt > 100:
                    self.get_logger().info("Screw driver loosen command timeout")
                    timeout = True
                    break
            else:
                timeout = False

        # Check for error
        if self.err_h[t_index]:
            err_state = self._err_handler(t_index)
            # There was no error and no timeout
            if (err_state == False) and (timeout == False):
                return RET_OK
            else:
                return RET_FAIL
        # There was no error handling only check timeout
        else:
            if timeout:
                return RET_FAIL
            else:
                return RET_OK

    def pickup_screw(self, t_index, zforce, screw_len, f_wait):
        '''
        Starts a screw pickup command

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param zforce:  Screw pickup force in N (18-30)
        @type zforce: int
        @param screw_len: Length of the screw in mm (0-35)
        @type screw_len: float
        @param f_wait: Wait for command to finish or not?
        @type f_wait: bool
        '''

        # Sanity check
        if zforce < 18 or zforce > 30:
            self.get_logger().info("Invalid zforce parameter for pickup screw command, valid range: 18-30")
            return RET_FAIL

        if screw_len < 0.0 or screw_len > 35.0:
            self.get_logger().info("Invalid screw length for pickup screw command, valid range: 0-35")
            return RET_FAIL

        self.cb.sd_pickup_screw(t_index, int(zforce), float(screw_len))

        timeout = False
        if f_wait:
            busy_cnt = 0
            f_busy = self.isBusy(t_index)
            while (f_busy):
                time.sleep(0.1)
                f_busy = self.isBusy(t_index)
                busy_cnt += 1
                if busy_cnt > 100:
                    self.get_logger().info("Screw driver tighten command timeout")
                    timeout = True
                    break
            else:
                timeout = False

        # Check for error
        if self.err_h[t_index]:
            err_state = self._err_handler(t_index)
            # There was no error and no timeout
            if (err_state == False) and (timeout == False):
                return RET_OK
            else:
                return RET_FAIL
        # There was no error handling only check timeout
        else:
            if timeout:
                return RET_FAIL
            else:
                return RET_OK

    def move_shank(self, t_index, shank_pos, f_wait):
        '''
        Moves the shank to the given position

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param shank_pos:  Shank position in mm (0-55)
        @type shank_pos: int
        @param f_wait: Wait for command to finish or not?
        @type f_wait: bool
        '''

        if self.isconn(t_index) is False:
            return CONN_ERR

        # Sanity check
        if shank_pos < self.min_shank_pos or shank_pos > self.max_shank_pos:
            self.get_logger().info("Invalid shank position parameter for move shank command, valid range: " + str(self.min_shank_pos) + "-" + str(self.max_shank_pos))
            return RET_FAIL

        self.cb.sd_move_shank(t_index, int(shank_pos))

        timeout = False
        if f_wait:
            busy_cnt = 0
            f_busy = self.isBusy(t_index)
            while (f_busy):
                time.sleep(0.1)
                f_busy = self.isBusy(t_index)
                busy_cnt += 1
                if busy_cnt > 30:
                    self.get_logger().info("Screw driver move shank command timeout!")
                    timeout = True
                    break
            else:
                timeout = False

        #  Check for error
        if self.err_h[t_index]:
            err_state = self._err_handler(t_index)
            # There was no error and no timeout
            if (err_state == False) and (timeout == False):
                return RET_OK
            else:
                return RET_FAIL
        # There was no error handling only check timeout
        else:
            if timeout:
                return RET_FAIL
            else:
                return RET_OK

    def halt(self, t_index):
        '''
        Stops the Screw driver

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        '''

        self.cb.sd_stop(t_index)

    def resetpower(self, t_index):
        '''
        Resets the power of the grippers\n
        Needs to be issued after saftey event

        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        '''
        if self.isconn(t_index) is False:
            return CONN_ERR

    def onrobot_sd_move_shank(self, request, response):
        """To handle sending commands via socket connection."""
        self.get_logger().info(request.command)
        pos_val_real = self.genCommand(request.command)

        while True:
            if not self.isBusy(self.sd_id):  # not busy
                self.get_logger().info("Shank pose (before): " + str(self.get_shank_pos(self.sd_id)) + "mm")
                break

        while True:
            if not self.isBusy(self.sd_id):  # not busy
                self.move_shank(t_index=self.sd_id, shank_pos=pos_val_real, f_wait=True)
                break

        while True:
            if not self.isBusy(self.sd_id):  # not busy
                self.get_logger().info("Shank pose (after): " + str(self.get_shank_pos(self.sd_id)) + "mm")
                break

        return response

    def genCommand(self, command):
        """Updates the command according to the character entered by the user."""    

        pos_val = int(float(command) * 55.0 + self.bit_extender_length)
        return pos_val


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotSDServer()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_service(node.set_command_srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

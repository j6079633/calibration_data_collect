from . import communicate


class Controller:
    def __init__(self):
        self.robot_communicate = communicate.Communicate()
        self.step_dic = {'GRIPPER_RESET': 0,
                         'Get_GRIPPER_STATUS': 1,
                         'GET_ROBOT_CART_POSITION': 2,
                         'GET_ROBOT_PULES_POSITION': 15,
                         'MOVE_POSITION': 3,
                         'MOVE_JOINT': 4,
                         'GRIPPER_OFF': 5,
                         'GRIPPER_ON': 6,
                         'CONVEYOR_A_OFF': 7,
                         'CONVEYOR_A_ON': 8,
                         'CONVEYOR_B_OFF': 9,
                         'CONVEYOR_B_ON': 10,
                         'CONVEYOR_C_OFF': 11,
                         'CONVEYOR_C_ON': 12,
                         'CONVEYOR_D_OFF': 13,
                         'CONVEYOR_D_ON': 14,
                         'GET_ROBOT_CART_POSITION_TOOL4': 16, # triangle P1
                         'GET_ROBOT_CART_POSITION_TOOL5': 17, # triangle P2
                         'GET_ROBOT_CART_POSITION_TOOL6': 18, # triangle P3
                         'GET_ROBOT_CART_POSITION_TOOL7': 19, # rectangle P1
                         'GET_ROBOT_CART_POSITION_TOOL8': 20, # rectangle P2
                         'GET_ROBOT_CART_POSITION_TOOL9': 21, # rectangle P3
                         'GET_ROBOT_CART_POSITION_TOOL10': 22, # circle P1
                         'GET_ROBOT_CART_POSITION_TOOL11': 23, # circle P2
                         'GET_ROBOT_CART_POSITION_TOOL12': 24, # circle P3
                         }
        self.robot_info = None
        self.state = -1

    def power_on(self):
        state, msg = self.robot_communicate.sock_setting()
        if state == 0:
            print('robot power on')
            self.state = state
        return state, msg

    def power_off(self):
        state, msg = self.robot_communicate.send_data_to_robot_close()
        if state == 0:
            print('robot power off')
        return state, msg

    #receive ' n ' : can't move
    def move_robot_pos(self, Tx, Ty, Tz, Rx, Ry, Rz, speed):
        #print('move robot position to: %d,%d,%d,%d,%d,%d (speed:%d)' %(Tx, Ty, Tz,Rx, Ry, Rz, speed))
        self.robot_communicate.send_data_to_robot(Tx, Ty, Tz, Rx, Ry, Rz, speed,
                                              self.step_dic['MOVE_POSITION'])
        receive = self.robot_communicate.recv_data_from_robot()
        return receive

    # receive ' n ' : can't move
    def move_robot_joint(self, joint1, joint2, joint3, joint4, joint5, joint6, speed):
        #print('move robot joint to: %d,%d,%d,%d,%d,%d (speed:%d)' % (joint1, joint2, joint3, joint4, joint5, joint6, speed))
        self.robot_communicate.send_data_to_robot(joint1,joint2,joint3, joint4, joint5,joint6,speed,
                                              self.step_dic['MOVE_JOINT'])
        receive = self.robot_communicate.recv_data_from_robot()
        return receive

    def gripper_on(self):
        #print('gripper on')
        self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                             self.step_dic['GRIPPER_ON'])
        self.robot_communicate.recv_data_from_robot()

    def gripper_off(self):
        #print('gripper off')
        self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                              self.step_dic['GRIPPER_OFF'])
        self.robot_communicate.recv_data_from_robot()

    def gripper_reset(self):
        self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                                  self.step_dic['GRIPPER_RESET'])
        self.robot_communicate.recv_data_from_robot()

    def get_gripper_status(self):
        self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                                  self.step_dic['Get_GRIPPER_STATUS'])
        receive = self.robot_communicate.recv_data_from_robot()
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive] #busy 1 0 0 0 0 0
        # print(self.robot_info[0])
        # print(self.robot_info[1])
        # print(self.robot_info[2])
        # print(self.robot_info[3])
        # print(self.robot_info[4])
        # print(self.robot_info[5])
        return receive

    def get_robot_pos(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive
        #print('get robot cart pos')

    def get_robot_pos_tool4(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL4'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool5(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL5'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool6(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL6'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool7(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL7'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool8(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL8'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool9(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL9'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool10(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL10'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool11(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL11'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pos_tool12(self):
        send_ok = self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_CART_POSITION_TOOL12'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive

        self.robot_info = [int(i) for i in receive]
        self.robot_info[0] = self.robot_info[0]/1000
        self.robot_info[1] = self.robot_info[1]/1000
        self.robot_info[2] = self.robot_info[2] / 1000
        self.robot_info[3] = self.robot_info[3] / 10000
        self.robot_info[4] = self.robot_info[4] / 10000
        self.robot_info[5] = self.robot_info[5] / 10000
        Tx = self.robot_info[0]; Ty = self.robot_info[1]; Tz = self.robot_info[2]
        Rx = self.robot_info[3]; Ry = self.robot_info[4]; Rz = self.robot_info[5]
        #print('get robot cart pos: %f,%f,%f,%f,%f,%f ' % (Tx, Ty, Tz, Rx, Ry, Rz))
        return receive

    def get_robot_pulse(self):
        self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
                                            self.step_dic['GET_ROBOT_PULES_POSITION'])
        receive = self.robot_communicate.recv_data_from_robot()
        if receive is None:
            print('not receive data from robot')
            return
        for i in receive:
            if str(i).find('s') != -1 or str(i).find('n') != -1:
                return receive
        self.robot_info = [int(i) for i in receive]
        joint1 = self.robot_info[0];joint2 = self.robot_info[1];joint3 = self.robot_info[2]
        joint4 = self.robot_info[3];joint5 = self.robot_info[4];joint6 = self.robot_info[5]
        #print('get robot joint pulse: %d,%d,%d,%d,%d,%d' % (joint1, joint2, joint3, joint4, joint5, joint6))
        return receive
        #print('get robot joint pulse')

    # def get_gripper_status(self):
    #
    #     self.robot_communicate.send_data_to_robot(0, 0, 0, 0, 0, 0, 0,
    #                                         self.step_dic['Get_GRIPPER_STATUS'])
    #     receive = self.robot_communicate.recv_data_from_robot()
    #     if receive is None:
    #         print('not receive data from robot')
    #         return
    #     self.robot_info = [int(i) for i in receive]
    #     B1 = self.robot_info[0];B2 = self.robot_info[1];B3 = self.robot_info[2]
    #     B4 = self.robot_info[3];B5 = self.robot_info[4];B6 = self.robot_info[5]
    #     #print('get gripper status: %d,%d,%d,%d,%d,%d' % (B1, B2, B3, B4, B5, B6))
    #     #print('get gripper status')

    def check_move_end(self, mode, input):
        #print(mode,'=input:', input)
        #print(mode,'=robot:', self.robot_info)
        if mode == 'pulse':
            self.get_robot_pulse()
        elif mode == 'position':
            self.get_robot_pos()
        else:
            print('controller/check_move_end/mode error: 0 or 1')
            return False
        if self.robot_info[0] == input[0] and\
           self.robot_info[1] == input[1] and \
           self.robot_info[2] == input[2] and \
           self.robot_info[3] == input[3] and\
           self.robot_info[4] == input[4] and\
           self.robot_info[5] == input[5]:
            return True
        else:
            return False
import sys
import re
import rclpy
import threading

from PySide2 import QtCore
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from MainFormUI import Ui_MainWindow
from tpm_msgs.srv import AxisOperation as axisOP
from tpm_msgs.srv import RobotOperation as robOP
from tpm_msgs.srv import JogPose
from sample_client_py import libRobotOP
from sample_client_py.libRobotOP import Axis_Parameter as axisParam
from sample_client_py.libRobotOP import Robot_Parameter as robotParam
from ProcessRunner import ProcessRunner

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.nowPoseId = 0
        self.nowAxisId = 0
        self.axisNum = 6

        self._opLib = libRobotOP.Lib()
        self._opLib.initROS()
        threading.Thread(target = lambda: rclpy.spin(self._opLib)).start()

        self.setup_ui()
        pass

    def closeEvent(self, event):
        self._opLib.destroy_node()
        rclpy.shutdown()

    def setup_ui(self):
        self.setFixedSize(980, 510)

        self.mainForm = Ui_MainWindow()
        self.mainForm.setupUi(self)
        
        self.axisId_buttons = [
            self.mainForm.radBtn_axis0,
            self.mainForm.radBtn_axis1,
            self.mainForm.radBtn_axis2,
            self.mainForm.radBtn_axis3,
            self.mainForm.radBtn_axis4,
            self.mainForm.radBtn_axis5,
            self.mainForm.radBtn_axisAll
        ]
        self.poseId_buttons = [
            self.mainForm.radBtn_poseX,
            self.mainForm.radBtn_poseY,
            self.mainForm.radBtn_poseZ,
            self.mainForm.radBtn_poseA,
            self.mainForm.radBtn_poseB,
            self.mainForm.radBtn_poseC
        ]
        self.poseTexts = [
            self.mainForm.txt_poseX,
            self.mainForm.txt_poseY,
            self.mainForm.txt_poseZ,
            self.mainForm.txt_poseA,
            self.mainForm.txt_poseB,
            self.mainForm.txt_poseC
            ]

        self.read_paras()
        self.set_slot()

    def read_paras(self):
        hmOffset = self._opLib.get_parameters('home_offsets').double_array_value
        psl = self._opLib.get_parameters('pos_limit').double_array_value
        nsl = self._opLib.get_parameters('neg_limit').double_array_value
        for axisId in range(self.axisNum):
            txt_hmOffset = getattr(self.mainForm, 'txt_hmOff' + str(axisId))
            txt_hmOffset.setText(f"{hmOffset[axisId]:.2f}")

            txt_psl = getattr(self.mainForm, 'txt_psl' + str(axisId))
            txt_psl.setText(f"{psl[axisId]:.2f}")

            txt_nsl = getattr(self.mainForm, 'txt_nsl' + str(axisId))
            txt_nsl.setText(f"{nsl[axisId]:.2f}")
            pass

    def set_slot(self):
        # update status timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.Handle_UpdateStatus)
        self.timer.start(200)

        # select axis event
        for btn in self.axisId_buttons:
            btn.clicked.connect(self.Handle_AxisSelectedChanged)
        
        # select jog dist event
        self.jogDist_buttons = [
            self.mainForm.radBtn_jogDist_0,
            self.mainForm.radBtn_jogDist_1,
            self.mainForm.radBtn_jogDist_2,
            self.mainForm.radBtn_jogDist_3
        ]
        for btn in self.jogDist_buttons:
            btn.clicked.connect(self.Handle_JogDistSelectedChanged)

        # speed slider event //valueChanged
        self.mainForm.sld_speed.sliderReleased.connect(self.Handle_speed_slider)
        self.mainForm.sld_speed.setValue(50)

        # axis operation event
        self.mainForm.btnOp_jogPos.pressed.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.JOG_POS))
        self.mainForm.btnOp_jogPos.released.connect(lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.STOP))
        self.mainForm.btnOp_jogNeg.pressed.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.JOG_NEG))
        self.mainForm.btnOp_jogNeg.released.connect(lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.STOP))

        
        self.mainForm.btnOp_svon    .clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.SERVO_ON))
        self.mainForm.btnOp_svoff   .clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.SERVO_OFF))
        self.mainForm.btnOp_home    .clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.HOME))
        self.mainForm.btnOp_clrAlm  .clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.CLEAR_ALM))
        self.mainForm.btnOp_stop    .clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.STOP))
        self.mainForm.btnOp_find_ORG.clicked.connect (lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.SEARCH_ORG))
        self.mainForm.btnOp_setToOffset .clicked.connect(lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.SET_AS_OFFSET))
        self.mainForm.btnOp_setToZero   .clicked.connect(lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.SET_AS_ZERO))
        self.mainForm.btnOp_mvToZero    .clicked.connect(lambda: self._opLib.axis_action(self.nowAxisId, axisOP.Request.MV_TO_ZERO))

        # pose jog event
        self.mainForm.btnJog_pos.pressed.connect (lambda: self._opLib.jog_pose(self.nowPoseId, JogPose.Request.DIR_POS))
        self.mainForm.btnJog_pos.released.connect(lambda: self._opLib.rob_action(robOP.Request.STOP, 0.0))
        self.mainForm.btnJog_neg.pressed.connect (lambda: self._opLib.jog_pose(self.nowPoseId, JogPose.Request.DIR_NEG))
        self.mainForm.btnJog_neg.released.connect(lambda: self._opLib.rob_action(robOP.Request.STOP, 0.0))

        # select pose event
        for btn in self.poseId_buttons:
            btn.clicked.connect(self.Handle_PoseSelectedChanged)

        '''  this approach fails.
        self.btnOp_mapping = [
            (self.mainForm.btnOp_svon,      axisOP.servo_on),
            (self.mainForm.btnOp_svoff,     axisOP.servo_off),
            (self.mainForm.btnOp_home,      axisOP.home),
            (self.mainForm.btnOp_clrAlm,    axisOP.clear_alm),
            (self.mainForm.btnOp_stop,      axisOP.stop),
            (self.mainForm.btnOp_find_ORG,   axisOP.search_org),
            (self.mainForm.btnOp_setToZero,  axisOP.set_pos_to_zero),
            (self.mainForm.btnOp_setToOffset,axisOP.set_pos_to_offset),
            (self.mainForm.btnOp_mvToZero,   axisOP.mv_to_zero),
        ]

        for btn, op in self.btnOp_mapping:
            btn.clicked.connect(lambda op2=op: self._opLib.axis_action(self.nowAxisId, op2))
        '''
        pass

    @QtCore.Slot()
    def Handle_UpdateStatus(self):
        msg = self._opLib.msg

        for axisId in range(self.axisNum):
            grayColor = u"background-color: rgb(128, 128, 128);"
            greenColor = u"background-color: rgb(128, 255, 128);"
            redColor = u"background-color: rgb(255, 128, 128);"

            # update SVO LED
            svoLED = getattr(self.mainForm, 'led_svo' + str(axisId))
            isSvOn = msg.axes[axisId].is_servo_on
            if isSvOn:
                svoLED.setStyleSheet(greenColor)
            else:
                svoLED.setStyleSheet(grayColor)

            # update ORG LED
            orgLED = getattr(self.mainForm, 'led_org' + str(axisId))
            isOrgOn = msg.axes[axisId].is_org
            if isOrgOn:
                orgLED.setStyleSheet(redColor)
            else:
                orgLED.setStyleSheet(grayColor)

            # update PEL LED
            pelLED = getattr(self.mainForm, 'led_pel' + str(axisId))
            isPelOn = msg.axes[axisId].is_pel
            if isPelOn:
                pelLED.setStyleSheet(redColor)
            else:
                pelLED.setStyleSheet(grayColor)

            # update NEL LED
            nelLED = getattr(self.mainForm, 'led_nel' + str(axisId))
            isNelOn = msg.axes[axisId].is_nel
            if isNelOn:
                nelLED.setStyleSheet(redColor)
            else:
                nelLED.setStyleSheet(grayColor)

            # update ALM LED
            almLED = getattr(self.mainForm, 'led_alm' + str(axisId))
            isAlmOn = msg.axes[axisId].is_alm
            if isAlmOn:
                almLED.setStyleSheet(redColor)
            else:
                almLED.setStyleSheet(grayColor)

            # update deg
            txtDeg = getattr(self.mainForm, 'txt_deg' + str(axisId))
            axisDeg = msg.axes[axisId].deg
            txtDeg.setText(f"{axisDeg:.2f}")

            # update cmd
            txtCmd = getattr(self.mainForm, 'txt_cmd' + str(axisId))
            axisCmd = msg.axes[axisId].mnet_command
            txtCmd.setText(str(axisCmd))

            # update pos
            txtPos = getattr(self.mainForm, 'txt_pos' + str(axisId))
            axisPos = msg.axes[axisId].mnet_encoder
            txtPos.setText(str(axisPos))

        # update pose
        for poseId in range(6):
           self.poseTexts[poseId].setText(str(msg.end_pose[poseId]))
           pass

        pass

    def Handle_AxisSelectedChanged(self):
        for axisId in range(self.axisNum):
            radBtn = getattr(self.mainForm, 'radBtn_axis' + str(axisId))
            if radBtn.isChecked():
                self.nowAxisId = axisId
                break

        if self.mainForm.radBtn_axisAll.isChecked():
            self.nowAxisId = -1
        pass

    def Handle_PoseSelectedChanged(self):
        for i in range(6):
            if self.poseId_buttons[i].isChecked():
                self.nowPoseId = i
                break
        pass

    def Handle_JogDistSelectedChanged(self):
        dist_table = [-1, 10, 1, 0.1]
        for i in range(4):
            radBtn = getattr(self.mainForm, 'radBtn_jogDist_' + str(i))
            if radBtn.isChecked():
                nowDist = dist_table[i]
                break

        print("selected jod dist: " + str(nowDist))
        self._opLib.rob_action(robOP.Request.JOG_DIST, nowDist)
        # self._opLib.set_robot_parameter(robotParam.jog_dist, nowDist)
        pass

    def Handle_speed_slider(self):
        speed = self.mainForm.sld_speed.value()
        print("Slider value changed: ", speed)
        self._opLib.rob_action(robOP.Request.FEEDRATE, speed)
        # self._opLib.set_robot_parameter(robotParam.move_speed, speed)


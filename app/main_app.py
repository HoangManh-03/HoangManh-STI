#!/usr/bin/env python3

# This Python file uses the following encoding: utf-8

"""
Developer: Le Duc Anh
Company: STI Viet Nam
Date: 13/03/2024
"""

import sys
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QDialog, QApplication, QWidget, QTableWidgetItem, QLabel
from PyQt5.QtGui import QPixmap, QIcon, QFont, QPalette, QBrush, QMovie
from PyQt5.QtCore import Qt, QTimer, QDateTime, QThread
import time
from Information import Mission_Error
import os
import config
import math
from sti_msgs import *
import threading
from multiprocessing import Process

class icon_picture():
    def __init__(self):
            current_dir = current_dir = os.path.dirname(os.path.abspath(__file__))
            image_path = os.path.join(current_dir, "interface", "background")
            self.image_path_2 = os.path.join(current_dir, "interface", "icon")
            self.url1 = os.path.join(image_path, '1.png')
            self.url2 = os.path.join(image_path, '2.png')
            self.url3 = os.path.join(image_path, '3.png')
            self.url4 = os.path.join(image_path, '4.png')
            self.url5 = os.path.join(image_path, '5.png')
            self.url6 = os.path.join(image_path, '6.png')
            self.url7 = os.path.join(image_path, '7.png')
            self.url8 = os.path.join(image_path, '8.png')
            self.Icon_on = QIcon(os.path.join(self.image_path_2, 'a.png'))
            self.Icon_off = QIcon(os.path.join(self.image_path_2, 'b.png'))
            self.icon_wifi = QIcon(os.path.join(self.image_path_2, 'wifi.png'))
            self.icon_nowifi = QIcon(os.path.join(self.image_path_2, 'nowifi.png'))

            
            self.Icon_xoaloi1 = QIcon(os.path.join(self.image_path_2, 'xoaloi1.png'))
            self.Icon_xoaloi2 = QIcon(os.path.join(self.image_path_2, 'xoaloi2.png'))
            self.canhbao = QIcon(os.path.join(self.image_path_2, 'canhbao.png'))
            self.nguyhiem = QIcon(os.path.join(self.image_path_2, 'nguyhiem.png'))
            self.stop = QIcon(os.path.join(self.image_path_2, 'stophere.png'))

class Menu(QWidget):
        def __init__(self):
            self.white = '#FFFFFF'
            self.blue = '#3E4095'
            self.blue_2 = '#E9F1FE'
            self.green_2 = '#00CC00'
            super(Menu, self).__init__()
            current_dir = os.path.dirname(os.path.abspath(__file__))
            ui_file_path = os.path.join(current_dir, "main_app.ui")
            loadUi(ui_file_path, self)            
            self.Information = Mission_Error(1)
            self.Information.start()
            self.icon = icon_picture()
            self.setStyleSheet(f"background-color: transparent; color: black;")            
            self.Icon_setSize()
            self.Icon_transparent()
            self.Icon_click()
            self.Config_bar()
            self.Icon_menuReturn()
            self.Control_manual()
            self.Process_keyBoard() #   cancel mission process
            self.timer = QTimer()
            self.timer_launch = QTimer()
            self.timer.timeout.connect(self.Info_label)
            self.timer_launch.timeout.connect(self.Launch_status)
            self.timer.start(200)
            self.timer_launch.start(200)
            self.timer.timeout.connect(self.update_slider)
            self.timer.timeout.connect(self.showTime)
            self.Control_getPoint()
            self.Info_tab.setStyleSheet(f"background-color: white; color: Black;")
            
            # Set hinh nen
            current_dir = current_dir = os.path.dirname(os.path.abspath(__file__))
            image_path = os.path.join(current_dir, "interface", "background")
            image_path_2 = os.path.join(current_dir, "interface", "icon")
            
            self.Auto_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url3});}}")
            self.Launch_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url1});}}")
            self.menu_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url2});}}")
            self.Manual_backGround.setStyleSheet(f"QWidget {{background-image: url({self.icon.url4});}}")
            self.Info_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url5});}}")
            self.Cancel_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url7});}}")
            self.Charge_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url6});}}")
            self.Quick_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url6});}}")
            self.FixError_background.setStyleSheet(f"QWidget {{background-image: url({self.icon.url6});}}")
            
            movie = QMovie(os.path.join(image_path_2, 'pin.gif'))
            movie.setScaledSize(self.Charge_icon.size())
            self.Charge_icon.setMovie(movie)
            movie.start()            
            
            self.AGV_status = 0
            self.Set_cancelButton()
            self.Button_info()
            self.d_traffic = 0
            
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

        def showTime(self):
            current_time = QDateTime.currentDateTime()
            label_time = current_time.toString('hh:mm:ss')
            self.Time.setText(label_time)
            self.Date.setText(current_time.toString('dd/MM/yyyy'))
            
        def Icon_click(self):
            self.Mode_auto.clicked.connect(self.Slot_auto)
            self.Mode_manual.clicked.connect(self.Slot_manual)
            self.Mode_info.clicked.connect(self.Slot_info)
            self.Mode_cancelMission.clicked.connect(self.Slot_cancelMission)
            self.Auto_mission.clicked.connect(self.Slot_cancelMission)
            self.Launch.clicked.connect(self.To_launch)
            self.exit.clicked.connect(self.Exit_app)
            self.To_charge.clicked.connect(self.Slot_Charge)
            self.Auto_manual.clicked.connect(self.Slot_manual)
            self.Auto_info.clicked.connect(self.Slot_info)
            self.Cancel_manual.clicked.connect(self.Slot_manual)
            self.Cancel_auto.clicked.connect(self.Slot_auto)
            self.Cancel_info.clicked.connect(self.Slot_info)
            self.Info_auto.clicked.connect(self.Slot_auto)
            self.Info_manual.clicked.connect(self.Slot_manual)
            self.Info_toMission.clicked.connect(self.Slot_cancelMission)
            self.Manual_mission.clicked.connect(self.Slot_cancelMission)
            self.Manual_info.clicked.connect(self.Slot_info)
            self.Manual_auto.clicked.connect(self.Slot_auto)
            self.Mode_getPoint.clicked.connect(self.Slot_getPoint)
            
        def Exit_app(self):
            QApplication.instance().quit()
            self.Information.is_exist = 0
            # self.Manual.is_exist = 0
            
        def Auto_warning(self):
            self.Auto_warningFront.setStyleSheet("background-color: transparent;")
            self.Auto_warningBack.setStyleSheet("background-color: transparent;")
            self.Auto_warningNav.setStyleSheet("background-color: transparent;")
            self.Auto_warningLift.setStyleSheet("background-color: transparent;")
            self.Auto_warningFront.hide()
            self.Auto_warningBack.hide()
            self.Auto_warningNav.hide()
            self.Auto_warningLift.hide()

        def Config_bar(self):
            # self.Information.name_card = "wlp4s0"
            self.baterry.setTextVisible(False)
            self.ip = config.get_ipAuto(self.Information.name_card)
            self.mac = config.get_MAC(self.Information.name_card)
            self.wifi = config.get_qualityWifi(self.Information.name_card)
            self.hostname = config.get_hostname()
            
            self.Ip_AGV.setText(self.ip)
            self.AGV_IP.setText(self.ip)
            self.Info_mac.setText(self.mac)
            self.Info_wifi.setText(str(self.wifi))

        def Icon_setSize(self):
            self.Icon_size = 125
            self.Mode_auto.setFixedSize(self.Icon_size, self.Icon_size)
            self.Mode_manual.setFixedSize(self.Icon_size, self.Icon_size)
            self.Mode_info.setFixedSize(self.Icon_size, self.Icon_size)
            self.Mode_cancelMission.setFixedSize(self.Icon_size, self.Icon_size)
            
#################################################################################################
###################################   Xóa nền tinh chỉnh CSS  ###################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

        def Icon_transparent(self):
            self.Wifi.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 10px;")
            font = QFont("Lato Heavy", 11)
            self.Info_hardware.setStyleSheet("font: 11pt 'Lato Heavy';")
            self.Info_mission.setStyleSheet("font: 11pt 'Lato Heavy';")
            #button

            self.Mode_auto.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Mode_manual.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Mode_info.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Mode_cancelMission.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Auto_mission.setStyleSheet(f"background-color: {self.blue}; color: {self.white};border-radius: 30px;")
            self.Mode_getPoint.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")

            self.Auto_return.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Auto_cancel.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_1.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_2.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_3.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_4.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_5.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_6.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_7.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_8.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_9.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_0.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_return.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Cancel_confirm.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Info_return.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Manual_return.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Manual_auto.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Manual_mission.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Manual_info.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Manual_volume.setStyleSheet(f"background-color: transparent; color: black;border-radius: 25px;")            
            
            self.Manual_charge.setStyleSheet(f"background-color: transparent; color: black;border-radius: 25px;")
            
            self.W_quickPoint.setStyleSheet("background-color: transparent; color: black; font: 14pt 'Lato Medium';")
            self.Quick_startTest.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_getShelfves.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_get1.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_get2.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_get3.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_number.setStyleSheet(f"background-color: white; color: black;border-radius: 50px;")
            self.Quick_beforeMission.setStyleSheet(f"background-color: white; color: black;border-radius: 50px;")
            self.Quick_afterMission.setStyleSheet(f"background-color: white; color: black;border-radius: 50px;")
            self.Quick_cancelShelf.setStyleSheet("background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
            self.Quick_return.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Quick_x.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_y.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_z.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_guongph.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_guongsd.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_x_2.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_y_2.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_z_2.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_guongph_2.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Quick_guongsd_2.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")

                        
            self.Manual_brake.setStyleSheet(f"background-color: transparent; color: black;border-radius: 25px;")
            self.Launch.setStyleSheet(f"background-color: transparent; color: black;border-radius: 30px;")
            self.Auto_mission.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            self.Charge_return.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.To_charge.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Auto_manual.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Auto_info.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Cancel_manual.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Cancel_auto.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Cancel_info.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Info_auto.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Info_manual.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Info_toMission.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Info_autoMission.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
            self.Info_handle.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
            self.Info_stopTraffic.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
            self.Manual_auto.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Manual_mission.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;")
            self.Manual_info.setStyleSheet(f"background-color: transparent; color: white;border-radius: 25px;") 
            
            self.Manual_liftup.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftdown.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftstop.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_lift_4.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            self.Manual_lift_5.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            self.Manual_lift_6.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            self.Manual_charge.setStyleSheet(f"background-color: transparent; color: black;border-radius: 25px;")
            
            self.lba.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.lbb.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.lbc.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Manual_position_x.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Manual_position_y.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.Manual_position_z.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.reflector_ph.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.reflector_sd.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.lbph.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            self.lbsd.setStyleSheet("background-color: transparent; color: black; border: 1px solid black;")
            #Label
            self.Cancel_output2.setStyleSheet(f"background-color: transparent;")
            self.exit.setStyleSheet(f"background-color: transparent; color: {self.white};border-radius: 30px;")
            self.Ip_AGV.setStyleSheet(f"background-color: transparent;")
            self.Cancel_output.setStyleSheet(f"background-color: transparent;")
            self.Auto_listError.setStyleSheet(f"background-color: {self.blue}; color: white;")
            self.Auto_listError_2.setStyleSheet(f"background-color: {self.blue}; color: white;")
            # self.Manual_listError.setStyleSheet(f"background-color: {self.blue}; color: white;")
            self.Manual_listError_2.setStyleSheet(f"background-color: {self.blue}; color: white;")
            self.Launch_percent.setStyleSheet(f"background-color: transparent;color: white")
            self.Launch_process.setStyleSheet(f"background-color: transparent;color: white")
            self.Table_point.setStyleSheet(f"background-color: white;color: black")
            self.Table_target.setStyleSheet(f"background-color: white;color: black")
            self.Table_info.setStyleSheet(f"background-color: white;color: black")
            self.Charge_percent.setStyleSheet(f"background-color: transparent;color: white")
            self.Auto_emer.setStyleSheet(f"background-color: transparent;")
           
            self.W_manual.setStyleSheet(f"background-color: transparent; color: black;")
            self.W_auto.setStyleSheet(f"background-color: transparent; color: black;")
            self.W_info.setStyleSheet(f"background-color: transparent; color: black;")
            
            self.Error_return.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            self.Cancel_del.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            
            

            self.Info_tab.setStyleSheet("""
                QTabWidget::pane {
                    /* This makes the content area transparent */
                    background-color: transparent;
                }
                QTabWidget::tab-bar {
                    /* This makes the tab bar transparent */
                    background-color: transparent;
                }
                QTabBar::tab {
                    background-color: transparent;
                    color: black;
                }
                QTabBar::tab:selected {
                    background-color: lightgray;
                }
            """)         
            
            self.Manual_cancel.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            self.Manual_upSpeed.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            self.Manual_downSpeed.setStyleSheet(f"background-color: transparent; color: white;border-radius: 30px;")
            self.Press_button()
            
            self.Info_sensorFront.setStyleSheet("background-color: white;color: black")
            self.Info_sensorAbove.setStyleSheet("background-color: white;color: black")
            self.Info_sensorBack.setStyleSheet("background-color: white;color: black")
            self.Info_RTC.setStyleSheet("background-color: white;color: black")
            self.Info_485.setStyleSheet("background-color: white;color: black")
            self.Info_NAV350.setStyleSheet("background-color: white;color: black")
            self.Info_motorLeft.setStyleSheet("background-color: white;color: black")
            self.Info_motorRight.setStyleSheet("background-color: white;color: black")
            self.Info_motorLift.setStyleSheet("background-color: white;color: black")
            self.Info_power.setStyleSheet("background-color: white;color: black")
            self.Info_resetButton.setStyleSheet("background-color: white;color: black")
            self.Info_emgFront.setStyleSheet("background-color: white;color: black")
            self.Info_emgBack.setStyleSheet("background-color: white;color: black")
            self.Info_bumper.setStyleSheet("background-color: white;color: black")
            self.Info_emg.setStyleSheet("background-color: white;color: black")
            self.Info_temp.setStyleSheet("background-color: white;color: black")
            self.Info_listError.setStyleSheet("background-color: white;color: black")
            self.pushButton_17.setStyleSheet("background-color: white;color: black")
            
        def Press_button(self):
            self.Up.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 30px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            self.Down.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 30px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            self.Left.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 30px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            self.Right.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 30px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            self.Stop_AGV.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 30px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            
            self.Manual_upSpeed.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 20px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
            self.Manual_downSpeed.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: transparent;
                    border-radius: 20px;
                }
                QPushButton:pressed {
                    background-color: rgba(255, 255, 255, 0.7);
                    color: black;
                }
            """)
                             

#################################################################################################
####################################        set icon size     ###################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
 
        def Set_icon(self):
            if self.Information.main_request.sound_on == 1:
                self.Manual_volume.setIcon(self.icon.Icon_on)
                self.Manual_volume.setIconSize(self.Manual_volume.size())
                self.v = 0
            else:
                self.Manual_volume.setIcon(self.icon.Icon_off)
                self.Manual_volume.setIconSize(self.Manual_volume.size())
                self.v = 1

            if self.Information.main_request.charge == 1:
                self.Manual_charge.setIcon(self.icon.Icon_on)
                self.Manual_charge.setIconSize(self.Manual_charge.size())
                self.c = 0
            else:
                self.Manual_charge.setIcon(self.icon.Icon_off)
                self.Manual_charge.setIconSize(self.Manual_charge.size())
                self.c = 1

            if self.Information.app_button.bt_disableBrake:
                self.Manual_brake.setIcon(self.icon.Icon_on)
                self.Manual_brake.setIconSize(self.Manual_brake.size())
                self.b = 0
            else:
                self.Manual_brake.setIcon(self.icon.Icon_off)
                self.Manual_brake.setIconSize(self.Manual_brake.size())
                self.b = 1
                            
        
        def Set_cancelButton(self):
            
            self.Manual_cancel.setIcon(self.icon.Icon_xoaloi1)
            self.Manual_cancel.setIconSize(self.Manual_cancel.size())
            self.Auto_cancel.setIcon(self.icon.Icon_xoaloi1)
            self.Auto_cancel.setIconSize(self.Auto_cancel.size())
            
            self.Auto_above.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            self.Auto_back.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            self.Auto_front.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            self.Manual_above.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            self.Manual_front.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            self.Manual_back.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
            
            self.Auto_above.setIconSize(self.Auto_above.size())
            self.Auto_back.setIconSize(self.Auto_back.size())
            self.Auto_front.setIconSize(self.Auto_front.size())
            self.Manual_above.setIconSize(self.Manual_above.size())
            self.Manual_front.setIconSize(self.Manual_front.size())
            self.Manual_back.setIconSize(self.Manual_back.size())      
            if self.wifi > 50:
                self.Wifi.setIcon(self.icon.icon_wifi)
                self.Wifi.setIconSize(self.Wifi.size())      
            else :
                self.Wifi.setIcon(self.icon.icon_nowifi)
                self.Wifi.setIconSize(self.Wifi.size())            

                                                                  
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################                                                    
                                                                 
        # Function slot action         
        def Slot_auto(self):
            self.Information.To_auto()
            self.baterry.setValue(80)
            self.stackedWidget.setCurrentWidget(self.W_auto)
            
        def Slot_getPoint(self):
            self.Information.To_hand()
            self.stackedWidget.setCurrentWidget(self.W_quickPoint)
        
        def Slot_manual(self):
            self.Information.To_hand()
            self.baterry.setValue(50)
            self.stackedWidget.setCurrentWidget(self.W_manual)
            
        def Slot_info(self):
            self.Information.To_hand()
            self.baterry.setValue(100)
            self.stackedWidget.setCurrentWidget(self.W_info)
            
        def Slot_cancelMission(self):
            self.Information.To_hand()
            self.baterry.setValue(0)         
            self.stackedWidget.setCurrentWidget(self.W_cancelMission)

        def Slot_Charge(self):
            self.Information.To_hand()
            self.stackedWidget.setCurrentWidget(self.W_Charge)


        def Icon_menuReturn(self):
            self.Auto_return.clicked.connect(self.Menu_return)     
            self.Manual_return.clicked.connect(self.Menu_return)
            self.Info_return.clicked.connect(self.Menu_return)
            self.Cancel_return.clicked.connect(self.Menu_return)           
            self.Charge_return.clicked.connect(self.Menu_return)
            self.Quick_return.clicked.connect(self.Menu_return)
            
            
        def Menu_return(self):
            #self.Information.To_hand()
            self.stackedWidget.setCurrentWidget(self.menu)



###### ---- Manual ---- ######                
        def Control_manual(self):
            self.v = 0
            self.l = 0
            self.c = 0
            self.b = 0
            self.Up.clicked.connect(self.Information.move_up)
            self.Down.clicked.connect(self.Information.move_down)
            self.Right.clicked.connect(self.Information.move_left)
            self.Left.clicked.connect(self.Information.move_right)
            self.Stop_AGV.clicked.connect(self.Information.move_stop)
            self.Manual_volume.clicked.connect(self.Button_volume)
            self.Manual_charge.clicked.connect(self.Button_charge)
            
            ###### ---- Chỉnh tốc độ 2 nút ---- ###### 
            self.Manual_upSpeed.clicked.connect(self.Information.up_speed)
            self.Manual_downSpeed.clicked.connect(self.Information.down_speed)
            
            ###### ---- Nút xóa lỗi ---- ######
            self.Manual_cancel.pressed.connect(self.Information.reset_EMC)
            self.Auto_cancel.pressed.connect(self.Information.reset_EMC)
            self.Manual_cancel.released.connect(self.Information.Unreset_EMC)
            self.Auto_cancel.released.connect(self.Information.Unreset_EMC)
            self.Manual_brake.clicked.connect(self.Button_brake)
            self.Manual_cancel.pressed.connect(self.change_icon_xoaloi_1)
            self.Manual_cancel.released.connect(self.change_icon_xoaloi_2)
            self.Auto_cancel.pressed.connect(self.change_icon_xoaloi_1)
            self.Auto_cancel.released.connect(self.change_icon_xoaloi_2)
            
            self.Manual_liftup.clicked.connect(self.Button_lift_up)
            self.Manual_liftdown.clicked.connect(self.Button_lift_down)
            self.Manual_liftstop.clicked.connect(self.Button_lift_stop)
            
            self.Auto_listError_2.clicked.connect(self.Auto_toError)
            self.Manual_listError_2.clicked.connect(self.Auto_toError)
            self.Error_return.clicked.connect(self.Error_return_)
        
        def Error_return_(self):
            print("aaa")
            if self.Information.NN_infoRespond.mode == 1:
                print("bbbbb")
                self.Slot_manual()
            elif self.Information.NN_infoRespond.mode == 2:
                print("ccccc")
                self.Slot_auto()
                
        def Auto_toError(self):
            self.stackedWidget.setCurrentWidget(self.W_FixError)
            
        def change_icon_xoaloi_1(self):
            self.Manual_cancel.setIcon(self.icon.Icon_xoaloi2)
            self.Auto_cancel.setIcon(self.icon.Icon_xoaloi2)
            

        def change_icon_xoaloi_2(self):
            self.Manual_cancel.setIcon(self.icon.Icon_xoaloi1)     
            self.Auto_cancel.setIcon(self.icon.Icon_xoaloi1)
                  
        def Button_volume(self):
            if self.v == 0:
                self.Manual_volume.setIcon(self.icon.Icon_off)
                self.Information.Manual_volume()
                self.v = 1
            elif self.v == 1:
                self.Manual_volume.setIcon(self.icon.Icon_on)
                self.Information.Manual_volume()
                self.v = 0
            
        def Button_lift_up(self):
            self.Information.Manual_lift_up()
            self.Manual_liftup.setStyleSheet(f"background-color: {self.green_2}; color: white;border-radius: 20px;")
            self.Manual_liftdown.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftstop.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            
        def Button_lift_down(self):
            self.Information.Manual_lift_down()
            self.Manual_liftup.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftdown.setStyleSheet(f"background-color:  {self.green_2}; color: white;border-radius: 20px;")
            self.Manual_liftstop.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")

        def Button_lift_stop(self):
            self.Information.Manual_lift_stop()
            self.Manual_liftup.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftdown.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 20px;")
            self.Manual_liftstop.setStyleSheet(f"background-color:  {self.green_2}; color: white;border-radius: 20px;")

        def Button_charge(self):
            if self.c == 0:
                self.Manual_charge.setIcon(self.icon.Icon_off)
                self.Information.Manual_charge()
                self.c = 1
            else:
                self.Manual_charge.setIcon(self.icon.Icon_on)
                self.Information.Manual_charge()
                self.c = 0                
        
        def Button_brake(self):
            if self.b == 0:
                self.Manual_brake.setIcon(self.icon.Icon_off)
                self.Information.Manual_brake()
                self.b = 1
            else:
                self.Manual_brake.setIcon(self.icon.Icon_on)
                self.Information.Manual_brake()
                self.b = 0
        
        def check_Manual_sensorLiftUp(self):
            pass
        def check_Manual_sensorLiftDown(self):
            pass
        def check_Manual_sensorLiftOn(self):
            pass        
                
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
       
                
###### ---- Quick Point ---- ######
        def Show_quickPoint(self):
            if self.Quick_number.currentText() == '1':
                self.Quick_get2.hide()
                self.Quick_get3.hide()
                self.Quick_t2.hide()
                self.Quick_t3.hide()
            elif self.Quick_number.currentText() == '2':
                self.Quick_get2.show()
                self.Quick_get3.hide()
                self.Quick_t2.show()
                self.Quick_t3.hide()
            elif self.Quick_number.currentText() == '3':
                self.Quick_get2.show()
                self.Quick_get3.show()
                self.Quick_t2.show()
                self.Quick_t3.show()

        def Control_getPoint(self):
            self.Quick_getShelfves.clicked.connect(self.Get_point_Shelves)
            self.Quick_get1.clicked.connect(self.Get_point_1)
            self.Quick_get2.clicked.connect(self.Get_point_2)
            self.Quick_get3.clicked.connect(self.Get_point_3)
            self.Quick_startTest.clicked.connect(self.Start_getPoint)
            self.Quick_cancelShelf.clicked.connect(self.Cancel_getPoint)
            
            # => Lấy điểm kệ hàng
        def Get_point_Shelves(self):
            print("Get point shelves")
            self.Information.shelves_x = self.Information.float_RobotPose_x
            self.Information.shelves_y = self.Information.float_RobotPose_y
            
            # => Sau đó lấy điểm 1
        def Get_point_1(self):
            print("Get point 1")
            self.Information.Request.target_x = self.Information.float_RobotPose_x
            self.Information.Request.target_y = self.Information.float_RobotPose_y
            self.Information.Request.target_z = round(self.Information.quaternion_to_euler(self.Information.robotPose_nav.pose.orientation),3)
            self.Information.degree_z = self.Information.float_RobotPose_z
            self.Information.Request.list_x[0] = self.Information.Request.target_x
            self.Information.Request.list_y[0] = self.Information.Request.target_y

            # => Sau đó lấy điểm 2
        def Get_point_2(self):
            print("Get point 2")
            self.Information.Request.list_x[1] = self.Information.Request.list_x[0]
            self.Information.Request.list_y[1] = self.Information.Request.list_y[0]
            self.Information.Request.list_x[0] = self.Information.float_RobotPose_x
            self.Information.Request.list_y[0] = self.Information.float_RobotPose_y
            
            # => Sau đó lấy điểm 3
        def Get_point_3(self):
            print("Get point 3")
            self.Information.Request.list_x[2] = self.Information.Request.list_x[1]
            self.Information.Request.list_y[2] = self.Information.Request.list_y[1]
            self.Information.Request.list_x[1] = self.Information.Request.list_x[0]
            self.Information.Request.list_y[1] = self.Information.Request.list_y[0]
            self.Information.Request.list_x[0] = self.Information.float_RobotPose_x
            self.Information.Request.list_y[0] = self.Information.float_RobotPose_y
            
        def Start_getPoint(self):
            if self.Information.Start_getPoint == 0 and self.Information.Request.target_z != 0:
                self.Quick_startTest.setStyleSheet(f"background-color: {self.blue}; color: black; border-radius: 10px; border: 2px solid black;")
                self.Information.send_mission()
                self.Information.Start_getPoint = 1
                self.Information.To_auto() #Chuyển sang Auto
                print("Chuyen sang auto roi")
            elif self.Information.Start_getPoint == 1:
                self.Quick_startTest.setStyleSheet(f"background-color: white; color: black; border-radius: 10px; border: 2px solid black;")
                self.Information.stop_mission()
                self.Information.To_hand() #Chuyển sang manual
                self.Information.Start_getPoint = 0
                print("Chuyen sang manual roi")

        def Cancel_getPoint(self):
            self.Information.shelves_x = 0.0
            self.Information.shelves_y = 0.0
            self.Information.Request.target_x = 0.0
            self.Information.Request.target_y = 0.0
            self.Information.Request.target_z = 0.0
            self.Information.Request.list_x = [0.0,0.0,0.0,0.0,0.0]
            self.Information.Request.list_y = [0.0,0.0,0.0,0.0,0.0]
            
            #Sua o day
        def Set_mission(self):
            number_text = self.Quick_number.currentText()
            before_text = self.Quick_beforeMission.currentText()
            after_text = self.Quick_afterMission.currentText()
            if number_text == "1":
                self.Information.Request.list_id = [1,0,0,0,0]
            elif number_text == "2":
                self.Information.Request.list_id = [1,2,0,0,0]
            elif number_text == "3":
                self.Information.Request.list_id = [1,2,3,0,0]
            
            if before_text == "Nâng kệ":
                self.Information.Request.before_mission = 1
            elif before_text == "Hạ kệ":
                self.Information.Request.before_mission = 2
            
            if after_text == "Nâng kệ":
                self.Information.Request.after_mission = 65
            elif after_text == "Hạ kệ":
                self.Information.Request.after_mission = 66
            elif after_text == "Về sạc":
                self.Information.Request.after_mission = 6
        

        def To_launch(self):    #Test
            self.stackedWidget.setCurrentWidget(self.W_launch)


#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
    

###### ---- Cancel Mission ---- ######
        def Process_keyBoard(self):
            self.Cancel_1.clicked.connect(self.Add_number)
            self.Cancel_2.clicked.connect(self.Add_number)
            self.Cancel_3.clicked.connect(self.Add_number)
            self.Cancel_4.clicked.connect(self.Add_number)
            self.Cancel_5.clicked.connect(self.Add_number)
            self.Cancel_6.clicked.connect(self.Add_number)
            self.Cancel_7.clicked.connect(self.Add_number)
            self.Cancel_8.clicked.connect(self.Add_number)
            self.Cancel_9.clicked.connect(self.Add_number)
            self.Cancel_0.clicked.connect(self.Add_number)
            self.Cancel_del.clicked.connect(self.Add_number)
            self.Cancel_confirm.clicked.connect(self.Cancel_mission)
        
        def Add_number(self):
            self.Cancel_output2.clear()
            if self.sender() == self.Cancel_1:
                self.Cancel_output.setText(self.Cancel_output.text() + "1")
            elif self.sender() == self.Cancel_2:
                self.Cancel_output.setText(self.Cancel_output.text() + "2")
            elif self.sender() == self.Cancel_3:
                self.Cancel_output.setText(self.Cancel_output.text() + "3")
            elif self.sender() == self.Cancel_4:
                self.Cancel_output.setText(self.Cancel_output.text() + "4")
            elif self.sender() == self.Cancel_5:
                self.Cancel_output.setText(self.Cancel_output.text() + "5")
            elif self.sender() == self.Cancel_6:
                self.Cancel_output.setText(self.Cancel_output.text() + "6")
            elif self.sender() == self.Cancel_7:
                self.Cancel_output.setText(self.Cancel_output.text() + "7")
            elif self.sender() == self.Cancel_8:
                self.Cancel_output.setText(self.Cancel_output.text() + "8")
            elif self.sender() == self.Cancel_9:
                self.Cancel_output.setText(self.Cancel_output.text() + "9")
            elif self.sender() == self.Cancel_0:
                self.Cancel_output.setText(self.Cancel_output.text() + "0")
            elif self.sender() == self.Cancel_del:
                self.Cancel_output.setText(self.Cancel_output.text()[:-1])
        
        def Cancel_mission(self):
            print(self.Cancel_output.text())
            if self.Cancel_output.text() == "1110":
                self.Cancel_output2.setText("Xác nhận hủy lệnh")
                self.Cancel_output.clear()
                # Thêm topic hủy lệnh
            else:
                self.Cancel_output2.setText("Mã không hợp lệ")
                self.Cancel_output.clear()

###### ---- Launch ---- ######

        def Launch_status(self):    #Status_launch
            # print("helllooo")
            self.Launch_percent.setText(f"{self.Information.status_launch.persent} %")
            self.Launch_processBar.setValue(self.Information.status_launch.persent)
            self.Launch_process.setText(f"Tiến trình hiện tại: {self.Information.status_launch.position}, {self.Information.status_launch.notification}")
            if self.Information.mode == 1:
                self.stackedWidget.setCurrentWidget(self.W_manual)
                self.timer_launch.stop()
            elif self.Information.mode == 2:
                self.stackedWidget.setCurrentWidget(self.W_auto)
                self.timer_launch.stop()

        def update_slider(self):
            self.current_value = self.Auto_line.value()
            # Set the new value on the slider
            if self.Information.NN_infoRespond.status == 0:
                new_value = self.current_value + 1
                if new_value > 99:
                    self.Auto_line.setValue(0)
                else:
                    self.Auto_line.setValue(new_value)
            elif self.Information.NN_infoRespond.status == 1:
                self.Auto_line.setValue(self.current_value)
            elif self.Information.NN_infoRespond.status == 2:
                self.Auto_line.setValue(self.current_value)


#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

# Button ở mục Info

        def Button_info(self):
            self.Info_autoMission.clicked.connect(self.Continue_auto)
            self.Info_handle.clicked.connect(self.Continue_handle)
            self.Info_stopTraffic.clicked.connect(self.Disconnect_traffic)
        def Continue_auto(self):
            self.Info_autoMission.setStyleSheet(f"background-color: red; color: white;border-radius: 25px;")
            self.Info_handle.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
            self.Information.To_auto()
        def Continue_handle(self):
            self.Info_autoMission.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
            self.Info_handle.setStyleSheet(f"background-color: red; color: white;border-radius: 25px;")
            self.Information.To_hand()
        def Disconnect_traffic(self):
            if self.d_traffic == 0:
                self.Info_stopTraffic.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 25px;")
                self.Information.Disconnect_traffic(0)
                print("Disconnect traffic")
                self.d_traffic = 1
            elif self.d_traffic == 1:
                self.Info_stopTraffic.setStyleSheet(f"background-color: red; color: white;border-radius: 25px;")
                self.Information.Disconnect_traffic(1)
                print("Connect traffic")
                self.d_traffic = 0
                
                

#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################
#################################################################################################

###### ---- Info update ---- ######
        def Info_label(self):
            
            self.Show_quickPoint()
            self.Auto_speed.setText(f"{round(self.Information.speed, 2)} km/h")
            print(self.Information.speed)
            self.baterry_value.setText(f"100%")
            
            self.Auto_process.setText(f"{self.Information.show_job(self.Information.NN_infoRespond.process)}")
            
            
            self.Set_mission()
            # Hien thi gia tri khi get point:
            self.Quick_shelves.setText(f"X: {self.Information.shelves_x} Y: {self.Information.shelves_y}")
            self.Quick_t1.setText(f"X: {self.Information.Request.target_x} Y: {self.Information.Request.target_y}")
            self.Quick_t2.setText(f"X: {self.Information.Request.list_x[1]} Y: {self.Information.Request.list_y[1]}")
            self.Quick_t3.setText(f"X: {self.Information.Request.list_x[2]} Y: {self.Information.Request.list_y[2]}")
            self.Quick_offset.setText(f"{self.Information.Request.offset}")
            self.Quick_targetz.setText(f"{self.Information.Request.target_z}")
            self.Quick_targetz_r.setText(f"{self.Information.degree_z}")
            
            # Vi tri x y z o che do bang tay
            # self.Manual_position.setText(f"{self.Information.robotPose_nav_x}       {self.Information.robotPose_nav_y}       {self.Information.robotPose_nav_z}")
            self.Manual_position_x.setText(f"{self.Information.robotPose_nav_x}")
            self.Manual_position_y.setText(f"{self.Information.robotPose_nav_y}")
            self.Manual_position_z.setText(f"{self.Information.robotPose_nav_z}")
            
            self.reflector_a.setText(f"{self.Information.lbv_numbeReflector}")
            self.reflector_b.setText(f"{self.Information.lbv_reflectorDetect}")
            self.reflector_ph.setText(f"{self.Information.lbv_numbeReflector} gương")
            self.reflector_sd.setText(f"{self.Information.lbv_reflectorDetect} gương")

            #Vi tri x y z o che do lay diem nhanh
            self.Quick_x.setText(f"{self.Information.robotPose_nav_x}")
            self.Quick_y.setText(f"{self.Information.robotPose_nav_y}")
            self.Quick_z.setText(f"{self.Information.robotPose_nav_z}")
            self.Quick_guongph.setText(f"{self.Information.lbv_numbeReflector}")
            self.Quick_guongsd.setText(f"{self.Information.lbv_reflectorDetect}")
            
            self.Auto_emer.setText("Trạng thái bình thường")
            
            self.Charge_percent.setText("Đang sạc 50%")
            
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################

            self.Auto_listError.clear()
            if self.Information.NN_infoRespond.status == 0:
                
                
                self.Auto_line.setStyleSheet("""
                    QSlider::groove:horizontal {
                        border: 1px solid #999999;
                        height: 30px;
                        background: white;
                    }
                    QSlider::handle:horizontal {
                        background: orange;
                        border: 2px solid #000000;
                        width: 40px;
                        margin: -2px 0;
                        border-radius: 3px;
                    }
                    QSlider::sub-page:horizontal {
                        background: green;
                    }
                """)

                self.Auto_listError.addItems(self.Information.listError)
                self.Auto_listError_2.setText(self.Information.listError[0])
                self.Manual_listError_2.setText(self.Information.listError[0])
                self.Auto_listError_2.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
                self.Manual_listError_2.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            elif self.Information.NN_infoRespond.status == 2:
                
                self.Auto_line.setStyleSheet("""
                    QSlider::groove:horizontal {
                        border: 1px solid #999999;
                        height: 30px;
                        background: white;
                    }

                    QSlider::handle:horizontal {
                        background: red;
                        border: 2px solid #000000;
                        width: 40px;
                        margin: -2px 0;
                        border-radius: 3px;
                    }
                    QSlider::sub-page:horizontal {
                        background: green;
                    }
                """)
                
                self.Auto_listError.addItems(self.Information.listError)
                self.Auto_listError.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
                self.Auto_listError_2.setText(self.Information.listError[0])
                self.Auto_listError_2.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
                self.Manual_listError_2.setText(self.Information.listError[0])
                self.Manual_listError_2.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
            elif self.Information.NN_infoRespond.status == 1:
                
                self.Auto_line.setStyleSheet("""
                    QSlider::groove:horizontal {
                        border: 1px solid #999999;
                        height: 30px;
                        background: white;
                    }

                    QSlider::handle:horizontal {
                        background: red;
                        border: 2px solid #000000;
                        width: 40px;
                        margin: -2px 0;
                        border-radius: 3px;
                    }
                    QSlider::sub-page:horizontal {
                        background: green;
                    }
                """)
                self.Auto_listError.addItems(self.Information.listError)
                self.Auto_listError.setStyleSheet(f"background-color: orange; color: white;border-radius: 10px;")
                self.Auto_listError_2.setText(self.Information.listError[0])
                self.Auto_listError_2.setStyleSheet(f"background-color: orange; color: white;border-radius: 10px;")
                self.Manual_listError_2.setText(self.Information.listError[0])
                self.Manual_listError_2.setStyleSheet(f"background-color: orange; color: white;border-radius: 10px;")
                
                self.Auto_lenh.setText(f"{self.Information.NN_infoRespond.process}")
                

            self.Manual_speed.setText(f"{self.Information.app_button.vs_speed}")
            self.Set_icon()
#################################################################################################
####################################             S            ###################################
####################################             T            ###################################
####################################             I            ###################################
#################################################################################################    

            self.Name = self.Information.NN_infoRequest.name_agv

            self.baterry.setValue(100) 
            if self.Information.Status_battery == 1:
                self.baterry.setStyleSheet(f"QProgressBar::chunk {{"f"background-color: {self.blue};}}")
            elif self.Information.Status_battery == 2:
                self.baterry.setStyleSheet(f"QProgressBar::chunk {{"f"background-color: yellow;}}")
            elif self.Information.Status_battery == 3:
                self.baterry.setStyleSheet(f"QProgressBar::chunk {{"f"background-color: red;}}")
            
            
            #####---- Safety ----#####
            if self.Information.NN_cmdRequest.target_id is not None:
                for row in range(min(5, len(self.Information.NN_cmdRequest.list_id))):
                    self.Table_point.setItem(0, row, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.list_id[row], 3)}"))
                    self.Table_point.setItem(1, row, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.list_x[row], 3)}"))
                    self.Table_point.setItem(2, row, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.list_y[row], 3)}"))
                    self.Table_point.setItem(3, row, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.list_speed[row], 3)}"))
                self.Table_target.setItem(0, 0, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.target_id, 3)}"))
                self.Table_target.setItem(0, 1, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.target_x, 3)}"))
                self.Table_target.setItem(0, 2, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.target_y, 3)}"))
                self.Table_target.setItem(0, 3, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.target_z, 3)}"))
                self.Table_target.setItem(0, 4, QTableWidgetItem(f"     {round(self.Information.NN_cmdRequest.before_mission, 3)}, {round(self.Information.NN_cmdRequest.after_mission, 3)}"))

            if self.Information.HC_info.vacham == 1:
                self.Auto_emer.setStyleSheet("color: red;")
                self.Auto_emer.setText("Va chạm vật cản")
                self.Manual_AGV_status.setStyleSheet("color: red;")
                self.Manual_AGV_status.setText("Va chạm vật cản")
            if self.Information.HC_info.zone_sick_ahead == 1 or self.Information.HC_info.zone_sick_behind == 1 or self.Information.safety_NAV.data == 1:
                self.Auto_emer.setStyleSheet("color: red;")
                self.Auto_emer.setText("Có vật cản")
                self.Manual_AGV_status.setStyleSheet("color: red;")
                self.Manual_AGV_status.setText("Có vật cản")
            else:
                self.Auto_emer.setStyleSheet(f"color: {self.blue};")
                self.Auto_emer.setText("Trạng thái bình thường")
                self.Manual_AGV_status.setStyleSheet(f"color: {self.blue};")
                self.Manual_AGV_status.setText("Trạng thái bình thường")

            if self.Information.HC_info.zone_sick_ahead == 0:
                self.Auto_front.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Manual_front.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Auto_front.setIcon(QIcon())
                self.Manual_front.setIcon(QIcon())
                
            elif self.Information.HC_info.zone_sick_ahead == 1:
                self.Auto_front.setIcon(self.icon.nguyhiem)
                self.Manual_front.setIcon(self.icon.nguyhiem)
                self.AGV_status = 1
            else:
                self.Auto_front.setIcon(self.icon.canhbao)
                self.Manual_front.setIcon(self.icon.canhbao)
                
            if self.Information.HC_info.zone_sick_behind == 0:
                self.Auto_back.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Manual_back.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Auto_back.setIcon(QIcon())
                self.Manual_back.setIcon(QIcon())
            elif self.Information.HC_info.zone_sick_behind == 1:
                self.Auto_back.setIcon(self.icon.nguyhiem)
                self.Manual_back.setIcon(self.icon.nguyhiem)
            else:
                self.Auto_back.setIcon(self.icon.canhbao)
                self.Manual_back.setIcon(self.icon.canhbao)
            
            if self.Information.safety_NAV.data == 0:
                self.Auto_above.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Manual_above.setStyleSheet(f"background-color: transparent; color: white;border-radius: 10px;")
                self.Auto_above.setIcon(QIcon())
                self.Manual_above.setIcon(QIcon())
            elif self.Information.safety_NAV.data == 1:
                self.Auto_above.setIcon(self.icon.nguyhiem)
                self.Manual_above.setIcon(self.icon.nguyhiem)
            else:
                self.Auto_above.setIcon(self.icon.canhbao)
                self.Manual_above.setIcon(self.icon.canhbao)
                
            if self.Information.OC_status.sensorLift.data == 1:
                self.Manual_lift_4.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
            else:
                self.Manual_lift_4.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            
            if self.Information.OC_status.sensorUp.data == 1:
                self.Manual_lift_5.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
            else:
                self.Manual_lift_5.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")
            
            if self.Information.OC_status.sensorDown.data == 1:
                self.Manual_lift_6.setStyleSheet(f"background-color: red; color: white;border-radius: 10px;")
            else:
                self.Manual_lift_6.setStyleSheet(f"background-color: {self.blue}; color: white;border-radius: 10px;")    
            
            #####---- Info tab 2 ----#####

        def app_loop(self):
            while True:
                self.Info_label()
                time.sleep(1)
    
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    welcome_screen = Menu()
    welcome_screen.show()
    sys.exit(app.exec_())

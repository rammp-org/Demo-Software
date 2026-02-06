from PyQt5.QtWidgets import QMainWindow, QWidget, QMenuBar, QStatusBar
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QRect, QMetaObject, QCoreApplication
from components.Buttons import BaseButton

class DevWindow(QMainWindow):

    def __init__(self, stacked_widget, teensy_controller):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.teensy_controller = teensy_controller
        self.setupUi()

    def to_main_menu(self):
        self.stacked_widget.setCurrentIndex(0)  # Set to User Window (Screen 0)

    def setupUi(self):
        self.setObjectName("SecondWindow")
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.setCentralWidget(self.centralwidget)

        self.mainButton = BaseButton(self.centralwidget, geometry=(20, 20, 100, 80))
        self.mainButton.clicked.connect(self.to_main_menu)

        self.fc_up_btn = BaseButton(self.centralwidget, geometry=(210, 40, 161, 81))
        self.fc_up_btn.clicked.connect(self.teensy_controller.FC_UP_pressed)

        self.ml_up_btn = BaseButton(self.centralwidget, geometry=(70, 150, 161, 81))
        self.ml_up_btn.clicked.connect(self.teensy_controller.ML_UP_pressed)

        self.fc_down_btn = BaseButton(self.centralwidget, geometry=(410, 40, 161, 81))
        self.fc_down_btn.clicked.connect(self.teensy_controller.FC_DOWN_pressed)

        self.mr_up_btn = BaseButton(self.centralwidget, geometry=(550, 150, 161, 81))
        self.mr_up_btn.clicked.connect(self.teensy_controller.MR_UP_pressed)

        self.mr_down_btn = BaseButton(self.centralwidget, geometry=(550, 270, 161, 81))
        self.mr_down_btn.clicked.connect(self.teensy_controller.MR_DOWN_pressed)

        self.ml_down_btn = BaseButton(self.centralwidget, geometry=(70, 270, 161, 81))
        self.ml_down_btn.clicked.connect(self.teensy_controller.ML_DOWN_pressed)

        self.rc_down_btn = BaseButton(self.centralwidget, geometry=(410, 370, 161, 81))
        self.rc_down_btn.clicked.connect(self.teensy_controller.RC_DOWN_pressed)

        self.rc_up_btn = BaseButton(self.centralwidget, geometry=(210, 370, 161, 81))
        self.rc_up_btn.clicked.connect(self.teensy_controller.RC_UP_pressed)

        self.lc_forward_btn = BaseButton(self.centralwidget, geometry=(245, 150, 131, 81))
        self.lc_forward_btn.clicked.connect(self.teensy_controller.L_CARRIAGE_FORWARD_pressed)

        self.lc_backward_btn = BaseButton(self.centralwidget, geometry=(245, 270, 131, 81))
        self.lc_backward_btn.clicked.connect(self.teensy_controller.L_CARRIAGE_BACKWARD_pressed)

        self.rc_forward_btn = BaseButton(self.centralwidget, geometry=(405, 150, 131, 81))
        self.rc_forward_btn.clicked.connect(self.teensy_controller.R_CARRIAGE_FORWARD_pressed)

        self.rc_backward_btn = BaseButton(self.centralwidget, geometry=(405, 270, 131, 81))
        self.rc_backward_btn.clicked.connect(self.teensy_controller.R_CARRIAGE_BACKWARD_pressed)

        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 800, 20))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("SecondWindow", "SecondWindow"))
        self.mainButton.setText(_translate("SecondWindow", "Main Menu"))
        self.fc_up_btn.setText(_translate("MainWindow", "FC UP"))
        self.fc_down_btn.setText(_translate("MainWindow", "FC DOWN"))
        self.ml_up_btn.setText(_translate("MainWindow", "ML UP"))
        self.mr_up_btn.setText(_translate("MainWindow", "MR UP"))
        self.ml_down_btn.setText(_translate("MainWindow", "ML DOWN"))
        self.mr_down_btn.setText(_translate("MainWindow", "MR DOWN"))
        self.rc_up_btn.setText(_translate("MainWindow", "RC UP"))
        self.rc_down_btn.setText(_translate("MainWindow", "RC DOWN"))
        self.lc_forward_btn.setText(_translate("MainWindow", "L CARRIAGE FORWARD"))
        self.lc_backward_btn.setText(_translate("MainWindow", "L CARRIAGE BACKWARD"))
        self.rc_forward_btn.setText(_translate("MainWindow", "R CARRIAGE FORWARD"))
        self.rc_backward_btn.setText(_translate("MainWindow", "R CARRIAGE BACKWARD"))
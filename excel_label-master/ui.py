from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 400)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(750, 60, 221, 39))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(750, 100, 221, 39))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(750, 140, 221, 39))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(750, 180, 221, 39))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(750, 220, 221, 39))
        self.pushButton_5.setObjectName("pushButton_4")

        self.pushButton_12 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_12.setGeometry(QtCore.QRect(830, 10, 31, 31))
        self.pushButton_12.setText("")
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_13 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_13.setGeometry(QtCore.QRect(890, 10, 31, 31))
        self.pushButton_13.setText("")
        self.pushButton_13.setObjectName("pushButton_13")
        self.pushButton_14 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_14.setGeometry(QtCore.QRect(950, 10, 31, 31))
        self.pushButton_14.setText("")
        self.pushButton_14.setObjectName("pushButton_14")

        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(10, 60, 700, 300))
        self.textBrowser.setObjectName("textBrowser")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 982, 23))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.pushButton.clicked.connect(MainWindow.p1_ck)
        self.pushButton_2.clicked.connect(MainWindow.p2_ck)
        self.pushButton_3.clicked.connect(MainWindow.p3_ck)
        self.pushButton_4.clicked.connect(MainWindow.p4_ck)
        self.pushButton_5.clicked.connect(MainWindow.p5_ck)

        self.pushButton_12.clicked.connect(self.close)  # 关闭窗口
        self.pushButton_14.clicked.connect(self.showMinimized)  # 最小化窗口

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))


        self.pushButton.setText(_translate("MainWindow", "previous"))
        self.pushButton_2.setText(_translate("MainWindow", "next"))
        self.pushButton_3.setText(_translate("MainWindow", "Positive"))
        self.pushButton_4.setText(_translate("MainWindow", "Negative"))
        self.pushButton_5.setText(_translate("MainWindow", "Drop"))

        self.pushButton_3.setStyleSheet('''QPushButton:hover{background:red;}''')
        self.pushButton_4.setStyleSheet('''QPushButton:hover{background:blue;}''')
        self.pushButton_5.setStyleSheet('''QPushButton:hover{background:green;}''')
        
        self.pushButton_12.setStyleSheet('''QPushButton{background:#F76677;border-radius:15px;}
                        QPushButton:hover{background:red;}''')
        self.pushButton_13.setStyleSheet('''QPushButton{background:#F7D674;border-radius:15px;}
                        QPushButton:hover{background:yellow;}''')
        self.pushButton_14.setStyleSheet('''QPushButton{background:#6DDF6D;border-radius:15px;}
                        QPushButton:hover{background:green;}''')

        self.setWindowOpacity(0.95)  # 设置窗口透明度
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)  # 设置窗口背景透明

        self.setWindowFlag(QtCore.Qt.FramelessWindowHint)  # 隐藏边框
        pe = QtGui.QPalette()
        self.setAutoFillBackground(True)
        pe.setColor(QtGui.QPalette.Window, QtCore.Qt.lightGray)  # 设置背景色
        # pe.setColor(QtGui.QPalette.Background,QtCore.Qt.blue)
        self.setPalette(pe)

        self.centralwidget.setStyleSheet('''
                    QPushButton{
                            border:2px solid #F3F3F5;color:white;padding-left:5px;
                            height:35px;
                            font-size:12px;
                            padding-right:5px;
                    }
                    QPushButton:hover{ color:white;
                        border:2px solid #F3F3F5;
                        border-radius:15px;
                        background:black;
                    }
                    QWidget#centralwidget{
                        background:Gray;
                        border-top:1px solid white;
                        border-bottom:1px solid white;
                        border-left:1px solid white;
                        border-top-left-radius:10px;
                        border-bottom-left-radius:10px;
                    }
                    ''')

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()  # 获取鼠标相对窗口的位置
            event.accept()
            self.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))  # 更改鼠标图标

    def mouseMoveEvent(self, QMouseEvent):
        if QtCore.Qt.LeftButton and self.m_flag:
            self.move(QMouseEvent.globalPos() - self.m_Position)  # 更改窗口位置
            QMouseEvent.accept()

    def mouseReleaseEvent(self, QMouseEvent):
        self.m_flag = False
        self.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
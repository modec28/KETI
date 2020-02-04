import RPi.GPIO as GPIO
import smbus
import time
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as fc



class Ui_TestLayout(QtWidgets.QWidget):
	def setupUi(self, TestBox):
		TestBox.setObjectName("TestLayout")
		TestBox.resize(1600,900)

#		self.LABLE = QtWidgets.QLabel(TestBox)
#		self.LABLE.setGeometry(QtCore.QRect(0,0,91,61))
#		self.LABLE.setObjectName("LABLE")
#		self.LABLE.setText("Hello World")

		self.BUTTON_ADC = QtWidgets.QPushButton(TestBox)
		self.BUTTON_ADC.setGeometry(QtCore.QRect(0,61,120,61))
		self.BUTTON_ADC.setObjectName("BUTTON_ADC")
		self.BUTTON_ADC.setText("ADC_CAPTURE")
		self.BUTTON_ADC.clicked.connect(self.ADC_capture)

#		fig = plt.Figure()
#		ax = fig.add_subplot(111)
#		canvas = fc(fig)
#		canvas.draw()

#		lay = QtWidgets.QHBoxLayout()
#		self.setLayout(lay)
#		lay.addWidget(canvas)
#		canvas.show()

		self.fig = plt.Figure()
		self.canvas = fc(self.fig)

		ADC_layout = QtWidgets.QVBoxLayout()
		ADC_layout.setGeometry(QtCore.QRect(0,120,300,300))
		ADC_layout.addWidget(self.canvas)

		Head_layout = QtWidgets.QVBoxLayout()
		Head_layout.addWidget(self.BUTTON_ADC)

		layout = QtWidgets.QHBoxLayout()
		layout.addLayout(ADC_layout)
		layout.setStretchFactor(ADC_layout, 0) #Enable resize layout
		layout.setStretchFactor(Head_layout, 1) #Disable resize layout

		self.setLayout(layout)

		ax = self.fig.add_subplot(111)
		ax.plot([1,2,3],[25,23,12])
		self.canvas.draw()

	def ADC_capture(self):
		self.fig.clear()

		self.LABLE.setText("Button Click")
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)

		i2c = smbus.SMBus(1)
		Slave_address = 0x0A
		value1 = [0xAA, 0x09, 0x04, 0x97, 0x01, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00]
		value2 = [0xAA, 0x09, 0x04, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x00]
		ADC = [0xAA, 0x09, 0x04, 0x98, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9E, 0x00]
		i2c.write_i2c_block_data(Slave_address,0x55,ADC)

		data = []
		while(1):
			P = GPIO.wait_for_edge(17,GPIO.FALLING,timeout=2000)
			if P is None:
#				print("Timeout")
				break
			else:
				data.append(i2c.read_i2c_block_data(Slave_address,0x0D))

		x=[]
		y=[]
		for i in range(1,101):
			x.append(i)
		for i in range(1,8):
			for j in range(1,14):
				if(data[i][2*j+5]>=128):
					y.append(-1*((255^data[i][2*j+4])+1+(256*(data[i][2*j+5]^255))))
				else:
					y.append(data[i][2*j+4]+(256*data[i][2*j+5]))
		for j in range(1,10):
			if(data[8][2*j+5]>=128):
				y.append(-1*((255^data[i][2*j+4])+1+(256*(data[8][2*j+5]^255))))
			else:
				y.append(data[8][2*j+4]+(256*data[8][2*j+5]))

		ax= self.fig.add_subplot(111)
		ax.plot(x,y)
		self.canvas.draw()


if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	TestLayout = QtWidgets.QDialog()
	ui = Ui_TestLayout()
	ui.setupUi(TestLayout)
	TestLayout.show()
	sys.exit(app.exec_())

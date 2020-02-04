import RPi.GPIO as GPIO
from smbus2 import SMBus
import time
import sys
import numpy as np
from PyQt5.QtWidgets import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

i2c = SMBus(1)
Slave_address = 0x0A
value1 = [0xAA, 0x09, 0x04, 0x97, 0x01, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00]
value2 = [0xAA, 0x09, 0x04, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x00]
ADC = [0xAA, 0x09, 0x04, 0x98, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9E, 0x00]
 
class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
 
        self.setLayout(self.layout)
        self.setGeometry(200, 200, 800, 600)
 
    def initUI(self):
        self.fig = plt.Figure()
        self.canvas = FigureCanvas(self.fig)
        
        Button_ADC = QPushButton()
        Button_ADC.setText("ADC_Capture")
        Button_ADC.clicked.connect(self.ADC_capture)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(Button_ADC)

        #cb = QComboBox()
        #cb.addItem('Graph1')
        #cb.addItem('Graph2')
        #cb.activated[str].connect(self.onComboBoxChanged)
        #layout.addWidget(cb)
 
        self.layout = layout
 
        #self.onComboBoxChanged(cb.currentText())
 
    #def onComboBoxChanged(self, text):
    #    if text == 'Graph1':
    #        self.doGraph1()
    #    elif text == 'Graph2':
    #        self.doGraph2()
    def ADC_capture(self):
        self.fig.clear()

        #self.LABLE.setText("Button Click")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        
        i2c.write_i2c_block_data(Slave_address,0x55,ADC)
        data = []
        while(1):
            P = GPIO.wait_for_edge(17,GPIO.FALLING,timeout=3000)
            if P is None:
                #print("Timeout")
                break
            else:
                raw_data = i2c.read_i2c_block_data(Slave_address,0,32)
                data.append(raw_data)
        x=[]
        y=[]
        y2=[]
        for i in range(1,201):
            x.append(i)
        #UPS
        for i in range(1,16):
            for j in range(1,14):
                if(data[i][2*j+5]>=128):
                    y.append(-1*((255^data[i][2*j+4])+1+(256*(data[i][2*j+5]^255))))
                else:
                    y.append(data[i][2*j+4]+(256*data[i][2*j+5]))
        for j in range(1,6):
            if(data[16][2*j+5]>=128):
                y.append(-1*((255^data[16][2*j+4])+1+(256*(data[16][2*j+5]^255))))
            else:
                y.append(data[16][2*j+4]+(256*data[16][2*j+5]))
        #DNS        
        for i in range(18,33):
            for j in range(1,14):
                if(data[i][2*j+5]>=128):
                    y2.append(-1*((255^data[i][2*j+4])+1+(256*(data[i][2*j+5]^255))))
                else:
                    y2.append(data[i][2*j+4]+(256*data[i][2*j+5]))
        for j in range(1,6):
            if(data[33][2*j+5]>=128):
                y2.append(-1*((255^data[16][2*j+4])+1+(256*(data[33][2*j+5]^255))))
            else:
                y2.append(data[33][2*j+4]+(256*data[33][2*j+5]))
        
        self.ADC_Graph(x,y,y2)
		
 
    def ADC_Graph(self,x,y,y2):
        #x = np.arange(0, 10, 0.5)
        #y1 = np.sin(x)
        #y2 = np.cos(x)
        
        self.fig.clear()
 
        ax = self.fig.add_subplot(111)
        #ax.plot(x, y1, label="sin(x)")
        #ax.plot(x, y2, label="cos(x)", linestyle="--")
        ax.plot(x,y)
        ax.plot(x,y2)
        
        #ax.set_xlabel("x")
        #ax.set_xlabel("y")
        
        #ax.set_title("sin & cos")
        #ax.legend()
        
        self.canvas.draw()
            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()

import kivy
kivy.require('1.9.1') 
import serial
import math

from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.clock import Clock
from kivy.uix.textinput import TextInput
from kivy.core.window import Window

class DataDisplay(FloatLayout):

    def __init__(self, seri, **kwargs):
        super(DataDisplay, self).__init__(**kwargs)
        self.vals = dict()
        self.ser = seri
        self.widgs = []
        label = Label(text='Label:  ',size_hint=(.5,1),font_size='50 sp',halign='right',valign='top')
        label.text_size = label.size
        self.widgs.append(label)
        label = Label(text='Value',size_hint=(.5,1),pos_hint={'x':.5},font_size='50 sp',halign='left',valign='top')
        label.text_size = label.size
        self.widgs.append(label)
        for widg in self.widgs:
            self.add_widget(widg)

    def updateSerial(self, arg):
        if not self.ser.in_waiting == 0:
            data = (self.ser.readline().decode('utf-8')[:-2]).split(':')
            if len(data)==1:
                data = ['Last Output',data[0]]
            if not data[0] in self.vals:
                print("Test")
                self.vals[data[0]] = Label(text=data[1],size_hint=(.5,1),pos_hint={'x':.5},halign='left',valign='top')
                self.widgs.append(Label(text=data[0]+":  ",size_hint=(.5,1),halign='right',valign='top'))
                self.widgs.append(self.vals[data[0]])
                self.add_widget(self.widgs[-2])
                self.add_widget(self.widgs[-1])
                i = 0
                for widg in self.widgs:
                    widg.size_hint=(.5,2./len(self.widgs))
                    widg.pos_hint ={'x':.5*(i%2),'y':1-(math.floor((i+2)/2))*(1.0/(len(self.widgs)/2))}
                    widg.text_size = widg.size
                    i+=1
            else:
                self.vals[data[0]].text = data[1]

    def resize(self):
        for widg in self.widgs:
            widg.text_size = widg.size
        
class FullDisplay(FloatLayout):

    def __init__(self, **kwargs):
        super(FullDisplay, self).__init__(**kwargs)
        self.ser = serial.Serial('/dev/ttyUSB0',19200)
        self.display = DataDisplay(self.ser, size_hint=(1,.9),pos_hint={'y':.1})
        self.add_widget(self.display)
        term = TextInput(multiline=False,size_hint=(1,.1))
        term.bind(on_text_validate=self.on_enter)
        self.add_widget(term)
        Clock.schedule_interval(self.display.updateSerial,1/25.)
        
    def on_enter(self,instance):
        self.ser.write(bytes(instance.text,'utf-8'))
        instance.text=''


class DisplayApp(App):

    def __init__(self):
        super(DisplayApp,self).__init__()
        self.display = FullDisplay()

    def build(self):
        def resize(window, width, height):
            self.display.display.resize();
        Window.bind(on_resize=resize)
        return self.display

    def on_stop(self):
        super(DisplayApp, self).on_stop()
        print("Stopping Now")
        self.display.ser.close();
        print("Stopped Serial")

DisplayApp().run()

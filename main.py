import tkinter as tk
import asyncio
import serial
import serial.tools.list_ports
import os
from PIL import ImageTk, Image
from tkinter import ttk
import pynmea2
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time

absolute_path = os.path.dirname(__file__)

class App:
    async def exec(self):
        self.window = Window(asyncio.get_event_loop())
        await self.window.show()
        

class Window(tk.Tk):
    def __init__(self, loop):
        self.loop = loop
        self.root = tk.Tk()
        self.root.title("Distance Sensor")
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg='#121212')
        self.lastLat = ""
        self.lastLon = ""
        self.lastGPSFix = ""
        self.measurementActive = False
        self.sensorData = [[0,0,0,0,0,0,0,0,0]]
        self.debug = True 

        self.df = pd.DataFrame(columns=['lat', 'lon', 'distance', 'laser1', 'laser2', 'laser3', 'laser4', 'laser5', 'laser6'])
        self.activeFrame = "rightFrame"

        self.chartFrame = tk.Frame(self.root, bg='#CDE5FF',width=800,height=600)

        self.rightFrame = tk.Frame(self.root, bg='#CDE5FF',width=800,height=600)
        self.rightFrame.pack(side=tk.RIGHT)

        leftFrame = tk.Frame(self.root, bg='#006399',width=227,height=600)
        leftFrame.pack(side=tk.LEFT)   

        #Debug label
        self.label = tk.Label(text="",bg='#006399',fg='#FFFFFF',font=("Arial", 25))
        self.label.place(x=100,y=100,anchor=tk.CENTER)
        self.animation = "░▒▒▒▒▒"

        rechthoek = Image.open(absolute_path + "/rechthoek.png")
        rechthoek = ImageTk.PhotoImage(rechthoek)

        rechthoekLabel = tk.Label(self.rightFrame,image=rechthoek, bg='#CDE5FF')
        rechthoekLabel.image = rechthoek
        rechthoekLabel.place(relx=0.5, y=500, anchor=tk.CENTER)  

        #Save as label
        self.labelSaved = tk.Label(self.chartFrame, text="", bg='#CDE5FF',fg='#000000',font=("Roboto", 16),width=50)
        self.labelSaved.place(relx=0.5,y=580,anchor="center")        

        #Distance label
        self.labelDistance = tk.Label(self.rightFrame, text="", bg='#FFFFFF',fg='#000000',font=("Roboto", 25),width=20)
        self.labelDistance.place(relx=0.5,y=500,anchor="center")
        self.lastDistance = ""
        self.lastSensorReading = ["0","0","0","0","0","0"]

        #Connected label
        self.labelConnected = tk.Label(leftFrame, text="", bg='#006399',fg='#FFFFFF',font=("Roboto", 12))
        self.labelConnected.place(relx=0.5, y=330, anchor=tk.CENTER)
        self.connected = False
        
        #Location label
        self.labelLoc = tk.Label(leftFrame, text="Location:", bg='#006399',fg='#FFFFFF',font=("Roboto", 12))
        self.labelLoc.place(relx=0.5, y=230, anchor=tk.CENTER)

        #GPS lat label
        self.labelLat = tk.Label(leftFrame, text="", bg='#006399',fg='#FFFFFF',font=("Roboto", 12))
        self.labelLat.place(relx=0.5, y=250, anchor=tk.CENTER)

        #GPS lon label
        self.labelLon = tk.Label(leftFrame, text="", bg='#006399',fg='#FFFFFF',font=("Roboto", 12))
        self.labelLon.place(relx=0.5, y=270, anchor=tk.CENTER)

        #GPS last fix
        self.lastFix = tk.Label(leftFrame, text="Last fix:", bg='#006399',fg='#FFFFFF',font=("Roboto", 10))
        self.lastFix.place(relx=0.5, y=290, anchor=tk.CENTER)        

        #Versie label
        self.labelVersie = tk.Label(leftFrame, text="Graslandproductiemeter v0.1", bg='#006399',fg='#FFFFFF',font=("Roboto", 8))
        self.labelVersie.place(relx=0.5, y=20, anchor=tk.CENTER)

        image = Image.open(absolute_path + "/gras.png")
        image = ImageTk.PhotoImage(image)
        gras1 = tk.Label(image=image, bg='#006399')
        gras1.image = image
        gras1.place(x=15, y=500)
        gras2 = tk.Label(image=image, bg='#006399')
        gras2.image = image
        gras2.place(x=90, y=500)

        rechthoekKlein = Image.open(absolute_path + "/rechthoekKlein.png")
        rechthoekKlein = ImageTk.PhotoImage(rechthoekKlein)
        
        rechthoekKleinLabel = tk.Label(leftFrame, text="Quit",fg='#FFFFFF',font=("Roboto", 12), image=rechthoekKlein, bg='#006399')
        rechthoekKleinLabel.image = rechthoekKlein
        rechthoekKleinLabel.place(relx=0.5, y=450, anchor=tk.CENTER)
        rechthoekKleinLabel.bind("<Button-1>", self.click)
        
        rechthoekKleinLabelText = tk.Label(leftFrame, text="Quit",fg='#000000',font=("Roboto", 21), bg='#FFFFFF')
        rechthoekKleinLabelText.place(relx=0.5, y=450, anchor=tk.CENTER)
        rechthoekKleinLabelText.bind("<Button-1>", self.click)

        rechthoekKleinLabel2 = tk.Label(leftFrame, image=rechthoekKlein, bg='#006399')
        rechthoekKleinLabel2.image = rechthoekKlein
        rechthoekKleinLabel2.place(relx=0.5, y=380, anchor=tk.CENTER)
        rechthoekKleinLabel2.bind("<Button-1>", self.measure)

        self.rechthoekKleinLabel2Text = tk.Label(leftFrame, text="Measure",fg='#000000',font=("Roboto", 21), bg='#FFFFFF')
        self.rechthoekKleinLabel2Text.place(relx=0.5, y=380, anchor=tk.CENTER)
        self.rechthoekKleinLabel2Text.bind("<Button-1>", self.measure)

        self.laserImage = Image.open(absolute_path + "/laser.png")
        self.laserImage = ImageTk.PhotoImage(self.laserImage)
        self.laserLabel = tk.Label(self.rightFrame, image=self.laserImage, bg='#CDE5FF')
        self.laserLabel.image = self.laserImage
        self.laserLabel.place(relx=0.5, y=235, anchor=tk.CENTER)

        self.chartImageLabel = tk.Label(self.chartFrame, bg='#CDE5FF')
        self.chartImageLabel.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        #bind click to label
        self.chartImageLabel.bind("<Button-1>", self.changeFrame)      

        self.laser1LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser1LabelText.place(relx=0.62, y=280, anchor=tk.CENTER)

        self.laser2LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser2LabelText.place(relx=0.62, y=310, anchor=tk.CENTER)

        self.laser3LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser3LabelText.place(relx=0.62, y=340, anchor=tk.CENTER)

        self.laser4LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser4LabelText.place(relx=0.62, y=370, anchor=tk.CENTER)

        self.laser5LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser5LabelText.place(relx=0.62, y=400, anchor=tk.CENTER)

        self.laser6LabelText = tk.Label(self.rightFrame, text="○",fg='#000000',font=("Roboto", 18), bg='#CDE5FF')
        self.laser6LabelText.place(relx=0.62, y=430, anchor=tk.CENTER)          
    
        #Find arduino
        self.port = self.find_arduino()
        self.arduino = serial.Serial(self.port, baudrate=115200)

        #Find GPS
        self.portGPS = self.findGPS()
        self.gps = serial.Serial(self.portGPS, baudrate=9600)
        
        #Start tasks
        self.loop.create_task(self.readArduino())
        self.loop.create_task(self.readGPS())
        self.loop.create_task(self.ping())   


    async def show(self):
        while True:
            if self.measurementActive:
                self.labelDistance["text"] = "Distance: " + "{:.2f}".format(float(self.lastDistance)) + " m"
            else:
                self.labelDistance["text"] = "Standby"
            sensor_labels = [
                self.laser1LabelText, self.laser2LabelText, self.laser3LabelText,
                self.laser4LabelText, self.laser5LabelText, self.laser6LabelText
            ]

            for i, label in enumerate(sensor_labels):
                label["text"] = "●" if self.lastSensorReading[i] == "1" else "○"
            # self.label["text"] = self.animation
            self.label["text"] = ""
            self.animation = self.animation[1:] + self.animation[0]  
            self.labelConnected["text"] = "Connected" if self.connected else "Not connected"
            # Calculate last GPS fix from datetime now - datetime last fix
            if self.lastGPSFix != "":
                lastFix = datetime.now() - self.lastGPSFix
                self.lastFix["text"] = "Last fix: " + str(lastFix.seconds) + " seconds ago"
                self.labelLat["text"] = "Lat: " + str(self.lastLat)
                self.labelLon["text"] = "Lon: " + str(self.lastLon)                    
            else:
                self.lastFix["text"] = "Last fix: No fix yet"
                self.labelLat["text"] = "Lat: -"
                self.labelLon["text"] = "Lon: -"                
                      
            self.root.update()
            await asyncio.sleep(.1)

    def click(self,event):
        self.root.destroy()

    def changeFrame(self,event):
        self.chartFrame.pack_forget()
        self.rightFrame.pack(side=tk.RIGHT)
        self.activeFrame = "rightFrame"
     
    def measure(self,event):
        try:
            if self.measurementActive == False:
                # If measurement is not active, start measurement
                if self.activeFrame == "chartFrame":
                    self.chartFrame.pack_forget()
                    self.rightFrame.pack(side=tk.RIGHT)
                    self.activeFrame = "rightFrame"
                self.arduino.write(b"r")
                self.lastDistance = "0.000"
                self.sensorData = [[self.lastLat, self.lastLon,0,0,0,0,0,0,0]]
                self.measurementActive = True
                self.rechthoekKleinLabel2Text["text"] = "Stop"
            else:
                # If measurement is active, stop measurement
                self.measurementActive = False
                if len(self.sensorData) > 2 or self.debug == True:
                    self.rechthoekKleinLabel2Text["text"] = "Saving..."
                    self.root.update()     
                    self.df = pd.DataFrame(self.sensorData, columns=['lat', 'lon', 'distance', 'laser1', 'laser2', 'laser3', 'laser4', 'laser5', 'laser6'])
                    # make columns lat, lon and distance as float
                    self.df['lat'] = self.df['lat'].astype(float)
                    self.df['lon'] = self.df['lon'].astype(float)
                    self.df['distance'] = self.df['distance'].astype(float)
                    # round lat, lon and distance to 4 decimals
                    self.df = self.df.round(4)
                    # sort df by distance
                    self.df = self.df.sort_values(by=['distance'])
                    # reset index
                    self.df = self.df.reset_index(drop=True)
                    now = datetime.now()
                    self.df.to_csv(absolute_path + "/measurementData/data_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".csv", index=False)
                    self.labelSaved["text"] = "Saved as: data_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
                    # if debug is True, load test data from CSV to dataframe
                    if self.debug == True:
                        self.df = pd.read_csv(absolute_path + "/testData.csv")
                    # Create new dataframe with distance from 0 to last distance with 0.0001 steps
                    x = np.arange(0.0,self.df['distance'].iloc[-1],0.0001) 
                    df_distance = pd.DataFrame(data = x, columns=["distance"])
                    df_distance = df_distance.round(4)
                    # Merge df_distance with self.df on distance
                    self.df = pd.merge(df_distance, self.df, on='distance', how='left')
                    # add missing values using ffill
                    self.df = self.df.fillna(method='ffill')

                    # Calculate from dataframe percentage of laser 1 to 6 where values are 1 
                    # Count number of rows where laser 1 to 6 is 1
                    # Divide by total number of rows
                    # Multiply by 100 to get percentage
                    laser1Percentage = (self.df['laser1'].sum() / self.df['laser1'].count()) * 100
                    laser2Percentage = (self.df['laser2'].sum() / self.df['laser2'].count()) * 100
                    laser3Percentage = (self.df['laser3'].sum() / self.df['laser3'].count()) * 100
                    laser4Percentage = (self.df['laser4'].sum() / self.df['laser4'].count()) * 100
                    laser5Percentage = (self.df['laser5'].sum() / self.df['laser5'].count()) * 100
                    laser6Percentage = (self.df['laser6'].sum() / self.df['laser6'].count()) * 100

                    # Create bar chart with the percentages per laser and save as png
                    # set size of chart
                    plt.figure(figsize=(6,4))
                    plt.bar(["Laser 1", "Laser 2", "Laser 3", "Laser 4", "Laser 5", "Laser 6"], [laser1Percentage, laser2Percentage, laser3Percentage, laser4Percentage, laser5Percentage, laser6Percentage]) 
                    plt.ylabel("Percentage")
                    plt.xlabel("Laser")
                    plt.title('Percentage of laser that is blocked')
                    plt.savefig(absolute_path + "/measurementData/chart_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".png")
                    plt.close()
                    self.chartImage = Image.open(absolute_path + "/measurementData/chart_" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".png")
                    self.chartImage = ImageTk.PhotoImage(self.chartImage)
                    self.chartImageLabel.configure(image=self.chartImage)
                    self.chartImageLabel.image = self.chartImage
                    self.rightFrame.pack_forget()
                    self.chartFrame.pack(side=tk.RIGHT)
                    self.activeFrame = "chartFrame"             
                self.rechthoekKleinLabel2Text["text"] = "Measure"
        except Exception as e:
            print("Something went wrong when starting or stopping a measurement..")
            print(e)
            self.measurementActive = False
            self.rechthoekKleinLabel2Text["text"] = "Measure"

    async def readArduino(self):
        while True:
            if (self.arduino.in_waiting > 0):
                try:
                    raw = self.arduino.read_until().decode("utf-8")
                    # check if first character is ':'
                    if raw[0] == ":":
                        distance = raw.split(":")[1].strip()
                        self.lastDistance = distance
                    elif raw[0] == "1" or raw[0] == "0":
                        #Split string at space and add the first 6 elements to list
                        data = raw.split(" ")[0:6]
                        self.lastSensorReading = data
                        distance = raw.split(" ")[6].strip()
                        self.lastDistance = distance
                        if self.measurementActive == True:
                            self.sensorData.append([self.lastLat, self.lastLon, self.lastDistance, data[0], data[1], data[2], data[3], data[4], data[5]])
                except Exception as e:
                    print("Something went wrong when reading and parsing arduino code")
                    print(e)
            await asyncio.sleep(0.00001)

    #Read GPS
    async def readGPS(self):
        while True:
            if (self.gps.in_waiting > 0):
                raw = self.gps.read_until().decode("utf-8")
                try:
                    gps = pynmea2.parse(raw,check=True)
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))
                    continue
                if hasattr(gps, "lat") and hasattr(gps, "lon"):
                    if float(gps.latitude) > 0  and float(gps.longitude) > 0:
                        self.lastLat = round(float(gps.latitude),4)
                        self.lastLon = round(float(gps.longitude),4)
                        self.lastGPSFix = datetime.now()
            await asyncio.sleep(0.001)


    async def ping(self):
        while True:
            hostname = "google.com"
            response = os.system("ping -c 1 " + hostname + ">/dev/null 2>&1")
            if response == 0:
                self.connected = True
            else:
                self.connected = False  
            await asyncio.sleep(60)                         

    def find_arduino(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "Arduino" in p.manufacturer:
                port = p.device
        return port

    def findGPS(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "u-blox" in p.manufacturer:
                port = p.device
        return port

asyncio.run(App().exec())
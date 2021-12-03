from tkinter import *
import tkinter as tk
from threading import Timer
import serial

# -------------PV----------------
font_tuple=("Arial",12)
button_height=0.25
button_relx=0

# ----------functions to toggle LED`s-------------
def toggle_blue_LED():
    serialcom.write('1'.encode())
def toggle_orange_LED():
    serialcom.write('2'.encode())
def toggle_red_LED():
    serialcom.write('3'.encode())
def toggle_green_LED():
    serialcom.write('4'.encode())

# ----------GUI start-------------
window=Tk()                 #create a top-level window (root)
window.title('lab5_python_serial')
window.geometry("1920x1080") #defines the width, height and coordinates of the top left corner of the frame
canvas=tk.Canvas(window)
canvas.place(relwidth=1,relheight=1)

background_image=tk.PhotoImage(file="wallpaper.png")
background_label=tk.Label(window,image=background_image)
background_label.place(relwidth=1,relheigh=1)

frame=tk.Frame(window,bg="#66c2ff")
frame.place(relx=0.1,rely=0.3,relwidth=0.2,relheight=0.25)

lbl=Label(window, text="", fg='black', font=("Helvetica", 16))
lbl.place(x=600,y=350)

btn1=Button(frame,font=font_tuple, text="press to toggle the blue LED", fg='black', command= toggle_blue_LED, anchor="center")
btn1.place(relx=button_relx,rely=0,relwidth=1,relheight=button_height)
btn2=Button(frame,font=font_tuple, text="press to toggle the orange LED", fg='black', command= toggle_orange_LED, anchor="center")
btn2.place(relx=button_relx,rely=0.25,relwidth=1,relheight=button_height)
btn3=Button(frame,font=font_tuple, text="press to toggle the red LED", fg='black', command= toggle_red_LED, anchor="center")
btn3.place(relx=button_relx,rely=0.5,relwidth=1,relheight=button_height)
btn4=Button(frame,font=font_tuple, text="press to toggle the green LED", fg='black', command= toggle_green_LED, anchor="center")
btn4.place(relx=button_relx,rely=0.75,relwidth=1,relheight=button_height)

# ----------GUI end-------------

serialcom = serial.Serial('COM8', 115200)

def read_temp():
    temp = serialcom.readline()
    res=temp.decode('utf')
    lbl.configure(text=str(res))
    lbl.config(text=str(res))
    Timer(0.1,read_temp).start()

read_temp()
window.mainloop()
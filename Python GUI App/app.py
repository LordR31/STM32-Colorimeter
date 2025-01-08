import bluetooth
from tkinter import *
import threading

target_address = "98:DA:50:03:A4:B5"

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_address, 1))  # RFCOMM 1
print("Connected to HC-05")

command = ''
hex = "#FFFFFF"
time = ""

button_event = threading.Event()

def app_on():
    global command
    command = '1'
    button_event.set()

def app_off():
    global command
    command = '0'
    button_event.set()

def set_it_2_4():
    global command
    command = '2'
    button_event.set()

def set_it_24():
    global command
    command = '3'
    button_event.set()

def set_it_101():
    global command
    command = '4'
    button_event.set()

def set_it_154():
    global command
    command = '5'
    button_event.set()

def set_it_700():
    global command
    command = '6'
    button_event.set()

def set_gain_x1():
    global command
    command = '7'
    button_event.set()

def set_gain_x4():
    global command
    command = '8'
    button_event.set()

def set_gain_x16():
    global command
    command = '9'
    button_event.set()

def set_gain_x60():
    global command
    command = 'A'
    button_event.set()

def app_control():
    global command
    while True:
        button_event.wait()
        button_event.clear()
        sock.send(command.encode())

def receive_data():
    global hex, text_label, time
    while True:
        data = sock.recv(64).decode('utf-8') # receive data
        print(data)                          # print data

        hex_position = data.find(" #")       # find HEX
        if hex_position != -1:
            hex = data[hex_position + 1:hex_position + 8]
            window.after(100, update_color)  # window update
        
        time_position = data.find("taken: ")
        if time_position != -1:
            time = data[time_position + 7:].strip()

        window.after(100, update_text) # update text

def update_color():
    global hex
    window.configure(bg=hex)

def update_text():
    global time
    text_label.config(text=time)              # update label text
    window.update_idletasks()                 # force label update

def window_control():
    global window, text_label, time
    window = Tk(className='STM32 Colour Window')
    window.geometry('400x400')
    
    button_on = Button(window, text="App ON", command=app_on)
    button_off = Button(window, text="App OFF", command=app_off)
    
    button_set_it_2_4 = Button(window, text="IT 2.4", command=set_it_2_4)
    button_set_it_24 = Button(window, text="IT 24", command=set_it_24)
    button_set_it_101 = Button(window, text="IT 101", command=set_it_101)
    button_set_it_154 = Button(window, text="IT 154", command=set_it_154)
    button_set_it_700 = Button(window, text="IT 700", command=set_it_700)
    
    button_set_gain_x1 = Button(window, text="Gain x1", command=set_gain_x1)
    button_set_gain_x4 = Button(window, text="Gain x4", command=set_gain_x4)
    button_set_gain_x16 = Button(window, text="Gain x16", command=set_gain_x16)
    button_set_gain_x60 = Button(window, text="Gain x60", command=set_gain_x60)
    
    button_on.grid(row=0, column=0, padx=10, pady=10)
    button_off.grid(row=0, column=1, padx=10, pady=10)
    
    button_set_it_2_4.grid(row=1, column=0, padx=10, pady=10)
    button_set_it_24.grid(row=1, column=1, padx=10, pady=10)
    button_set_it_101.grid(row=1, column=2, padx=10, pady=10)
    button_set_it_154.grid(row=1, column=3, padx=10, pady=10)
    button_set_it_700.grid(row=1, column=4, padx=10, pady=10)
    
    button_set_gain_x1.grid(row=2, column=0, padx=10, pady=10)
    button_set_gain_x4.grid(row=2, column=1, padx=10, pady=10)
    button_set_gain_x16.grid(row=2, column=2, padx=10, pady=10)
    button_set_gain_x60.grid(row=2, column=3, padx=10, pady=10)

    text_label = Label(window, text=time, anchor="ne", justify=RIGHT)
    text_label.grid(row=0, column=6, columnspan=2, sticky="ne")

    window.mainloop()

if __name__ == "__main__":
    t1 = threading.Thread(target=window_control)
    t2 = threading.Thread(target=app_control, daemon=True) 
    t3 = threading.Thread(target=receive_data, daemon=True)

    t1.start()
    t2.start()
    t3.start()

    t1.join()

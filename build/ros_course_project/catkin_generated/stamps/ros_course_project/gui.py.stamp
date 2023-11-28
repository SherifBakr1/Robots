#!/usr/bin/env python3

import tkinter as tk

class GUI:

    def __init__(self, direction_key_callback, coordinate_callback):
        self.main = tk.Tk()
        self.main.title("GUI")
        self.main.geometry("500x500")
        self.main.resizable(False, False)
        self.main.configure(background="white")
        self.main.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.direction_key_callback = direction_key_callback
        self.coordinate_callback = coordinate_callback

        self.create_widgets()
        self.main.mainloop()

    def create_widgets(self):
        self.label = tk.Label(self.main, text="TurtleBot Control Panel", font=("Arial", 20))
        self.label.pack()

        ## Direction Keys
        self.button_forward = tk.Button(self.main, text="Forward", command=self.on_forward)
        self.button_forward.pack()
        self.button_backward = tk.Button(self.main, text="Backward", command=self.on_backward)
        self.button_backward.pack()
        self.button_left = tk.Button(self.main, text="Left", command=self.on_left)
        self.button_left.pack()
        self.button_right = tk.Button(self.main, text="Right", command=self.on_right)
        self.button_right.pack()

        ## Coordinate Input
        self.label_coordinate = tk.Label(self.main, text="Enter Coordinate:")
        self.label_coordinate.pack()
        self.label_coordinate_x = tk.Label(self.main, text="X:")
        self.label_coordinate_x.pack()
        self.entry_coordinate_x = tk.Entry(self.main)
        self.entry_coordinate_x.pack()
        self.label_coordinate_y = tk.Label(self.main, text="Y:")
        self.label_coordinate_y.pack()
        self.entry_coordinate_y = tk.Entry(self.main)
        self.entry_coordinate_y.pack()
        self.label_coordinate_z = tk.Label(self.main, text="Z:")
        self.label_coordinate_z.pack()
        self.entry_coordinate_z = tk.Entry(self.main)
        self.entry_coordinate_z.pack()
        self.button_coordinate = tk.Button(self.main, text="Go", command=self.on_coordinate)
        self.button_coordinate.pack()

        ## Current Position Display
        self.label_position = tk.Label(self.main, text="", font=("Arial", 10))
        self.label_position.pack()

        ## Error Message
        self.label_error = tk.Label(self.main, text="", font=("Arial", 10))
        self.label_error.pack()
    
    def on_forward(self):
        self.direction_key_callback("forward")

    def on_backward(self):
        self.direction_key_callback("backward")

    def on_left(self):
        self.direction_key_callback("left")
    
    def on_right(self):
        self.direction_key_callback("right")
    
    def on_coordinate(self):
        x = self.entry_coordinate_x.get()
        y = self.entry_coordinate_y.get()
        z = self.entry_coordinate_z.get()

        if x == "" or y == "" or z == "":
            self.label_error.configure(text="Please enter a value for x, y, and z.")
            return

        self.coordinate_callback(x, y, z)
    
    def update_position(self, x, y, z):
        self.label_position.configure(text="X: {}, Y: {}, Z: {}".format(x, y, z))

    def on_closing(self):
        self.main.destroy()
        exit()

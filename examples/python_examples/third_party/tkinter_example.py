#!/usr/bin/env python3
from tkinter import Tk, messagebox, Button

# object of TK()
main = Tk()
# function to use the
# askquestion() function
def Submit():
    messagebox.askquestion("Form","Do you want to Submit")
# setting geometry of window
# instance
main.geometry("100x100")
# creating Window
B1 = Button(main, text = "Submit", command = Submit)
# Button positioning
B1.pack()
# infinite loop till close
main.mainloop()

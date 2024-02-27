import tkinter as tk

class userInterface:
   def __init__(self):
      self.root = tk.Tk()
      self.root.resizable(False, False)
      #self.label = tk.Label(self.root, text="Hello, world!")
      #self.label.pack()
      self.canvas = tk.Canvas(self.root, width = 500, height = 500)
      self.canvas.pack()
      self.image_1 = tk.PhotoImage(file=r"C:\frc\RedSide.png")
      self.image_2 = tk.PhotoImage(file=r"C:\frc\BlueSide.png")
      self.image = self.canvas.create_image(0,20, anchor=tk.NW, image=self.image_1)
      self.currentImage = self.image_1
      self.note = tk.IntVar()
      self.canvas.pack()
      self.w = tk.Radiobutton (self.root, text="Note 1", variable=self.note,value = 1, command=self.noteChoice)
      self.x = tk.Radiobutton (self.root, text="Note 2", variable=self.note,value = 2, command=self.noteChoice)
      self.y = tk.Radiobutton (self.root, text="Note 3", variable=self.note,value = 3, command=self.noteChoice)
      self.w.pack(anchor = tk.W)
      self.x.pack(anchor = tk.W)
      self.y.pack(anchor = tk.W)
      self.B = tk.Button(self.root, text ="Change Field", command = self.switch_image)
      self.B.place(x=0,y=0)
      self.var1 = tk.IntVar()
      self.var2 = tk.IntVar()
      self.var3 = tk.IntVar()
      self.var4 = tk.IntVar()
      self.var5 = tk.IntVar()
      self.c1 = tk.Checkbutton(self.root, text='1',variable=self.var1, onvalue=1, offvalue=0, command=self.print_selection)
      self.c1.place(x=400, y=70)
      #self.c1.pack()
      self.c2 = tk.Checkbutton(self.root, text='2',variable=self.var2, onvalue=1, offvalue=0, command=self.print_selection)
      self.c2.place(x=400, y=160)
      #self.c2.pack()
      self.c3 = tk.Checkbutton(self.root, text='3',variable=self.var3, onvalue=1, offvalue=0, command=self.print_selection)
      self.c3.place(x=400, y=250)
      #self.c1.pack()
      self.c4 = tk.Checkbutton(self.root, text='4',variable=self.var4, onvalue=1, offvalue=0, command=self.print_selection)
      self.c4.place(x=400, y=340)
      #self.c2.pack()
      self.c5 = tk.Checkbutton(self.root, text='5',variable=self.var5, onvalue=1, offvalue=0, command=self.print_selection)
      self.c5.place(x=400, y=430)
      #self.c2.pack()
   def print_selection(self):
    if (self.var1.get() == 1):
        print('One choosen ')
    if (self.var2.get() == 1):
        print('2 choosen')
    if (self.var3.get() == 1):
        print('One choosen ')
    if (self.var4.get() == 1):
        print('2 choosen')
    if (self.var5.get() == 1):
        print('One choosen ')
   def noteChoice(self):
      selection = "You selected the option " + str(self.note.get())
      print(selection)
   def switch_image(self):
    if self.currentImage == self.image_1:
        print("IN IF")
        self.canvas.itemconfig(self.image, image=self.image_2)
        self.currentImage = self.image_2
        self.c5.place(x=50, y=70)
        self.c4.place(x=50, y=160)
        self.c3.place(x=50, y=250)
        self.c2.place(x=50, y=340)
        self.c1.place(x=50, y=430)
    else:
        print("IN ELSE")
        self.canvas.itemconfig(self.image, image=self.image_1)
        self.currentImage = self.image_1
        self.c1.place(x=400, y=70)
        self.c2.place(x=400, y=160)
        self.c3.place(x=400, y=250)
        self.c4.place(x=400, y=340)
        self.c5.place(x=400, y=430)
UI = userInterface()
UI.root.mainloop()
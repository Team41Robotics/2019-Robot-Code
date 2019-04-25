from tkinter import *
from networktables import NetworkTables
from networktables.util import ChooserControl

NetworkTables.initialize(server='10.0.41.2')
sd = NetworkTables.getTable('SmartDashboard')
ch = ChooserControl('Zeroing at Init')

font = '"Comic Sans MS" '

root = Tk()
root.title('Custom SmartDashboard')
width = 500
height = 800
root.geometry('{}x{}'.format(width, height))

ctx = Canvas(root, width=width, height=height, background='white')
ctx.pack()

ctx.create_text(width/2, height/8, text='Zero?', font=font+'60', fill='black')

def draw_btns(fillNo = 'gray', fillYes = 'gray'):
	ctx.delete('noFill')
	ctx.create_rectangle(0, height/4, width/2, height/2, fill=fillNo, outline='black', tag='noFill')
	ctx.delete('noTxt')
	ctx.create_text(width/4, 3*height/8, text='No', font=font+'40', fill='white', tag='noTxt')
	ctx.delete('yesFill')
	ctx.create_rectangle(width/2, height/4, width, height/2, fill=fillYes, outline='black', tag='yesFill')
	ctx.delete('yesTxt')
	ctx.create_text(3*width/4, 3*height/8, text='Yes', font=font+'40', fill='white', tag='yesTxt')

draw_btns(fillNo='blue')

pressures = []

def update():
	# Get moving average of pressures
	pressures.append(sd.getNumber('Pressure Sensor (PSI)', 0))
	if len(pressures) > 30:
		pressures.pop(0)
	avg = round(sum(pressures) / len(pressures))

	# Display stuff
	ctx.delete('psi')
	ctx.create_text(width/2, 5*height/8, text='{} PSI'.format(avg), fill='black', font=font+'80', tag='psi')

	ctx.delete('above60color')
	ctx.delete('above60txt')
	above60color = 'red'
	above60txt = 'Below\n60 PSI'
	if avg > 60:
		above60color = 'green'
		above60txt = 'Above\n60 PSI'
	if sd.getBoolean('pressurized?', False):
		above60color = 'blue'
		above60txt = 'Full\nPressure'
	ctx.create_rectangle(0, 3*height/4, width, height, fill=above60color, tag='above60color')
	ctx.create_text(width/2, 7*height/8, text=above60txt, fill='white', font=font+'50', justify=CENTER, tag='above60txt')
	
	root.after(50, update)

def handle_click(event):
	if event.y > height/4 and event.y < height/2: # Some button was pressed
		if event.x < width/2: # No button
			ch.setSelected('No zero')
			draw_btns(fillNo='blue')
		else: # Yes button
			ch.setSelected('Yes zero')
			draw_btns(fillYes='blue')


ctx.bind('<Button-1>', handle_click)
root.after(50, update)

root.mainloop()

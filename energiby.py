import argparse
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import numpy.matlib

import math
from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import udp_client
from threading import Thread 

from pprint import pprint
import urllib.request

import json

#plt.style.use('fivethirtyeight')
plt.rcParams['toolbar'] = 'None'


oscSenderTeensy = udp_client.SimpleUDPClient("192.168.1.102",7134)

# Variables used for the live plot
global x_values, y_values, bio_raw, index, run, t, td

# Data about the electrical system
el_mul = 0.01
el_t      = np.linspace(0,48,49,True)
el_210129 = [3865., 3864., 3823., 3848., 3937., 4123, 4534, 5147, 5377, 5385, 5389, 5368, 5223, 5094, 5134, 4963, 5040, 5498, 5366, 4941, 4586, 4312, 4099, 3869]
el_210129 = el_210129+el_210129
el_210129.append(el_210129[0])
el_210129 = np.array(el_210129) * el_mul


time_vector = np.zeros(481)
need_vector = np.zeros(481)
need_min_vector = np.zeros(481)
need_max_vector = np.zeros(481)

lastNeed = el_210129[0]

for x in range(481):
    t = x / 10.0
    alpha = 0.15
    lastNeed = np.interp(t, el_t, el_210129)*alpha + lastNeed*(1.0-alpha)
    need_vector[x] = lastNeed
    need_min_vector[x] = lastNeed - 5
    need_max_vector[x] = lastNeed + 5
    time_vector[x] = t


steps = 0
firstStep = True

# Add normal distributed create consumption range
# e1 = np.random.normal(0.1, 0.01, size=el_210129.shape)
# e2 = np.random.normal(0.1, 0.01, size=el_210129.shape)
e1 = 0.1
e2 = 0.1
consumption_min = el_210129*(1.0-abs(e1))
consumption_max = el_210129*(1.0+abs(e2))

print(el_t.shape)
print(el_210129.shape)


def timeOfDay(t):
    while(t > 24):
        t -= 24
    return t


#url = 'https://www.energidataservice.dk/proxy/api/datastore_search?resource_id=fixedresidualconsumption&limit=15'
#fileobj = urllib.request.urlopen(url)
#jdata = json.loads(fileobj.read())
#print(jdata)



x_values = []
y_values = []
b_values = []
v_values = []
s_values = []
index = 0
run = False
t = 0
td = 0

# Vind generator
vind_n      = 0
vind_N      = 15
vind_mean   = 2000 * el_mul
vind_sd     = 1500 * el_mul
vind_alpha  = 0.01 # Alpha value for 1st order lowpass filter
vind_a1     = 0.1  # 
vind_v1     = vind_mean
vind_value  = vind_mean
vind_tmp    = vind_mean
vind_vector = np.zeros(481)

production_alpha = 0.9 # Alpha value for 1st order lowpass filter
production_value = vind_mean

oven_amount_initial = 13.0
oven_amount = oven_amount_initial
oven_amount_max = 26.0
oven_amount_ok_min = 8.0
oven_amount_ok_max = 18.0
oven_amount_to_fill = 4.0
oven_consumption_rate = 0.1

storage_amount = 64.0
storage_amount_max = 64.0



def fillOven():
    global oven_amount, storage_amount
    spaceInOven = oven_amount_max - oven_amount
    if storage_amount >= oven_amount_to_fill and spaceInOven >= oven_amount_to_fill:
        oven_amount    = oven_amount    + oven_amount_to_fill
        storage_amount = storage_amount - oven_amount_to_fill
    elif spaceInOven >= oven_amount_to_fill:
        oven_amount    = oven_amount + storage_amount
        storage_amount = 0

def vind():
    global vind_value, vind_v1, vind_tmp, vind_n, vind_sd
    if vind_n >= vind_N:
        vind_tmp = np.random.normal(vind_mean,vind_sd)
        print(vind_tmp)
        vind_n = 0
    else:
        vind_n = vind_n+1
    vind_v1 = vind_tmp*vind_a1 + vind_v1*(1-vind_a1) # Compute 1st lowpass filter
    vind_value = vind_v1*vind_alpha + vind_value*(1-vind_alpha) # Compute 2nd lowpass filter
    vind_value = max(vind_value,0)
    return vind_value

def makeNewVindParameters():
    global vind_value, vind_v1, vind_tmp, vind_n, vind_sd, vind_mean, vind_vector
    # Reset Vind
    vind_mean   = np.random.normal(1000, 1000)
    if vind_mean<0:
        vind_mean = 0
    vind_sd   = abs(np.random.normal(0, 1500))

    vind_mean = vind_mean * el_mul
    vind_sd   = vind_sd   * el_mul

    vind_v1     = vind_mean
    vind_value  = vind_mean
    vind_tmp    = vind_mean
    for x in range(481):
        vind_vector[x] = vind()



# Sol generator
sol_alpha  = 0.1 # Alpha value for 1st order lowpass filter
sol_max    = 0.07 * el_210129.mean()
sol_v1  = 0.0
sol_value  = 0.0
sol_vector = np.zeros(481)


def sol(td):
    global sol_value, sol_v1
    sol = 0.0
    if td > 5 and td < 13: 
        sol_alpha = 0.1
        sol = 1.0
    else:
        sol = 0.0
        sol_alpha = 0.05 

    sol *= sol_max
    sol_v1 = sol*sol_alpha + sol_v1*(1-sol_alpha) # Compute 2nd lowpass filter
    sol_value = sol_v1*sol_alpha + sol_value*(1-sol_alpha) # Compute 2nd lowpass filter
    return sol_value

def makeNewSolVector():
    global sol_value, sol_v1, sol_vector
    sol_value = 0
    sol_v1 = sol_value
    for x in range(481):
        td = timeOfDay(x/10.0)
        sol_vector[x] = sol(td)


# Compute the Total Production
bio_max = 60
bio_raw = 0.0
bio_alpha = 0.03 # Alpha value for 1st order lowpass filter
bio_v1 = 0.0
bio_value = 0.0

def bio():
    global bio_raw, bio_value, bio_v1, oven_amount
    #bio_v1 = bio_raw*bio_alpha + bio_v1*(1-bio_alpha) # Compute 2nd lowpass filter
    bio_v1 = bio_raw
    oven_factor = 0.9 + 0.3*oven_amount/oven_amount_max
    bio_factor = 1.0

    consumption = (0.9 + 0.3*bio_v1/bio_max) * oven_factor * oven_consumption_rate

    if oven_amount > oven_amount_ok_max+0.5:
        consumption *= 1 + (oven_amount - oven_amount_ok_max)
    elif oven_amount < oven_amount_ok_min-0.5:
        bio_factor = max(1 - 0.2*(oven_amount_ok_min - oven_amount), 0.0)

        
    oven_amount = max(oven_amount - consumption, 0.0)

    if(oven_amount == 0.0): bio_factor = 0

    bio_new = bio_v1 * bio_factor
    bio_value = bio_new*bio_alpha + bio_value*(1-bio_alpha) # Compute 2nd lowpass filter

    return bio_value


# Compute the Total Production
production_alpha = 0.5 # Alpha value for 1st order lowpass filter
production_value = vind_mean

def production(index):
    global production_value, bio_raw
    p = vind_vector[index] + sol_vector[index] + bio()
    production_value = p*production_alpha + production_value*(1-production_alpha)
    return production_value


# Define the Figure and axes
fig, (ax1) = plt.subplots(nrows=1, ncols=1, figsize=(8, 4))
plt.subplots_adjust(bottom=0.25)
fig.set_facecolor('grey')
ax1.set_xlim([0,48]) # Set the x-limits
ax1.set_ylim([0,100]) # Set the y-limits
ax1.set_xlabel('Time [h]')
ax1.set_ylabel('Power [MW]')
ax1.set_xticks([0  ,6  ,12  ,18  ,24 ,30 ,36  ,42  ,48])
xlabels = ['0:00','6:00','12:00','18:00','0:00','6:00','12:00','18:00','0:00']
ax1.set_xticklabels(xlabels)

# Plot the Range as to stay within 
ax1.fill_between(time_vector, need_min_vector, need_max_vector, label="Behov")

lb, = ax1.plot(x_values,b_values,'g-', label="Biomasse / Affald") # Create a line with the data
lv, = ax1.plot(x_values,v_values,'b-', label="Vind") # Create a line with the data
ls, = ax1.plot(x_values,s_values,'r-', label="Sol") # Create a line with the data
l,  = ax1.plot(x_values,y_values,'k-', label="Samlet Produktion") # Create a line with the data

ax1.legend(loc='upper left')

def updatePlot():
    l.set_xdata(x_values)
    l.set_ydata(y_values)
    lb.set_xdata(x_values)
    lb.set_ydata(b_values)
    lv.set_xdata(x_values)
    lv.set_ydata(v_values)
    ls.set_xdata(x_values)
    ls.set_ydata(s_values)
    oscSenderTeensy.send_message("/ElData", [vind_vector[index], sol_vector[index], bio_value, oven_amount, storage_amount, production_value, need_min_vector[index]])


def clear():
    global x_values, y_values, b_values, v_values, s_values, bio_raw, bio_value, index, run, t, td
    global vind_v1, vind_value, vind_tmp, production_value
    global storage_amount, oven_amount
    global steps
    x_values = []
    y_values = []
    b_values = []
    v_values = []
    s_values = []
    storage_amount = storage_amount_max
    oven_amount = oven_amount_initial
    makeNewVindParameters()
    makeNewSolVector()
    bio_value = need_vector[0] - vind_vector[0]
    index = 0
    t = 0
    td = 0
    production_value = need_vector[0]
    steps = 0
    updatePlot()



# Animate Function for the plotting
def animate(i):
    global index, run, t, td, steps
    if run:
        t = index * 0.1
        td = timeOfDay(t)
        y = production(index)
        x_values.append(t)
        y_values.append(y)
        b_values.append(bio_value)
        v_values.append(vind_vector[index])
        s_values.append(sol_vector[index])
        updatePlot()        
        index = index+1
        if t >= 48.0:
            run = run = False
            print("Consumption {0}".format(index))

    time.sleep(.02)

# --------------------------------------------------------------
# ------------------------- OSC --------------------------------
# --------------------------------------------------------------
# Function to recieve value over osc 
def oscValue(addr, value):
    global bio_raw, bio_max
    bio_raw = value * bio_max
    print("[{0}] ~ {1}".format(addr, bio_raw))

def oscCmd(addr, value):
    global x_values, y_values, bio_raw, index, run
    if value == 'clear':
        clear()
    elif value == 'run':
        run = True
    elif value == 'stop':
        run = False
    elif value == 'StartButton':
        run = False
        clear()
        run = True
    elif value == 'FillButton':
        fillOven()
        
    print("[{0}] ~ {1}".format(addr, value))

def oscAmountInOven(addr, value):
    global amountInOven
    amountInOven = value
    print("[{0}] ~ {1}".format(addr, bio_raw))

# Setup the OSC Functionality
dispatcher = dispatcher.Dispatcher()
parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="0.0.0.0", help="The ip to listen on")
parser.add_argument("--port", type=int, default=7133, help="The port to listen on")
args = parser.parse_args()
dispatcher.map("/filter", print)
dispatcher.map("/value", oscValue)
dispatcher.map("/cmd", oscCmd)
dispatcher.map("/AmountInOven", oscAmountInOven)

server = osc_server.ThreadingOSCUDPServer((args.ip, args.port), dispatcher)
print("Serving on {}".format(server.server_address))


# Start Osc in a Thread
oscThread = Thread(target = server.serve_forever)
oscThread.start()

clear()
run = True #TODO Remove This Line!!

# Start the Animation Funtion
ani = FuncAnimation(fig, animate, interval = 10)

# Show the plot on screen
plt.ioff()
plt.tight_layout()
figman = plt.get_current_fig_manager()
figman.full_screen_toggle()
plt.show()



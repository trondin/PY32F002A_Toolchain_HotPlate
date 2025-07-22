import os
import math
import matplotlib.pyplot as plt

# start point
#T1=-25
T1=10
# end point
#T2=100
T2=220
# step
deltaT=5
#reference point
ntcT=25
# NTC B coefficient
ntcB=3950
# R0 - resistabce at reference point
#R25=10
R25=100
# pull up tesistor
#RUP=10
RUP=5.1
# ADC maximul value 12bit - 4095, 10bit - 1023
ADCscale=4095
#ADCscale=1023

csv_file_name = os.path.dirname(__file__) +"/ntc_lut.csv"
csv_file = open(csv_file_name, "w")
csv_file.write("R25,Rref,B,Rpullup,ADC"+"\r\n")
csv_file.write(str(R25)+","+ str(ntcT)+","+str(ntcB)+","+str(RUP)+","+str(ADCscale)+"\r\n\r\n")
csv_file.write("T°C,R,scale,#ADC"+"\r\n")

h_file_name = os.path.dirname(__file__) +"/ntc_lut.h"
h_file = open(h_file_name, "w")
h_file.write("#ifndef _NTC_LUT_H\r\n")
h_file.write("#define _NTC_LUT_H\r\n\r\n")
h_file.write("// temperure lookup\r\n")
h_file.write("const uint16_t lookup[][2]={\r\n")


temp = []
data1 = []
data2 = []
for t in range(T1,T2+1,deltaT):

    res = R25*math.exp(ntcB*(1/(t+273)-1/(ntcT+273)))
    scale = res/(res+RUP)
    ADC=scale*ADCscale

    csv_file.write(str(t)+",")
    csv_file.write(str(round(res,5))+",")   
    csv_file.write(str(round(scale,5))+",")     
    csv_file.write(str(int(round(ADC,0)))+"\r\n")     

    h_file.write(str(int(round(ADC,0)))+",\t")   
    h_file.write(str(t*10)+",\r\n")

    temp.append(t)
    data1.append(res)
    data2.append(ADC)

csv_file.close()

h_file.write("};\r\n\r\n")
h_file.write("#endif\r\n")
h_file.close()


# chart
fig, ax1 = plt.subplots()
color = 'tab:red'
ax1.set_xlabel('Temperature °C')
ax1.set_ylabel('Resistance', color=color)
ax1.plot(temp, data1, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second Axes that shares the same x-axis
color = 'tab:blue'
ax2.set_ylabel('ADC code', color=color)  # we already handled the x-label with ax1
ax2.plot(temp, data2, color=color)
ax2.tick_params(axis='y', labelcolor=color)

label = 'R='+str(R25)+"\n"
label += 'B='+str(ntcB)+"\n"
label += 'Rup='+str(RUP)
xl = max(temp)
yl = max(data1)
ax1.text(xl, yl, label,  ha='right', va='top', fontsize = 13)

fig.canvas.manager.set_window_title('NTC test')

plt.show()


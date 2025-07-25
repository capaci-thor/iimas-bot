from datetime import datetime
import time

timestamp = time.time()
date_time = datetime.fromtimestamp(timestamp)
str_date_time = date_time.strftime("%d-%m-%Y_%H-%M-%S")

path = '/home/iibot/datos/'
file = open(path + 'datos_' + str_date_time+'.csv', 'x')
for i in range(40):
    file.write(str(i))

file.close()
import numpy as np
import pandas as pa
from scipy import stats


pd = pa.read_csv("read.csv")
pwm = pd["pwm"]
vel_r = pd["v_r"]
vel_l = pd["v_l"]


slope_r, intercept_r, r_r, p_r, std_err_r = stats.linregress(vel_r, pwm)
slope_l, intercept_l, r_l, p_l, std_err_l = stats.linregress(vel_l, pwm)
x = 0.1451
outR = slope_r * x + intercept_r
outL = slope_l * x + intercept_l

print(outL)
print(outR)




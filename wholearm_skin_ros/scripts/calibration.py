#!/usr/bin/env python3

import pickle
import scipy.linalg
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# TODO: change this to the exact number of the taxel you want to calibrate
filename = "link3_36"

data = pickle.load(open("data_collection_" + filename + ".pickle", "rb"))

prev_val = 0
skin = np.array(data['skin'])
ft = np.array(data['ft'])

maxlen = len(data['skin'])
halflen = int(maxlen/2)

def linear_regression(x, y):
    A = scipy.vstack([x, scipy.ones(len(x))]).T
    m, c = scipy.linalg.lstsq(A, y)[0]
    print("linear_regression")
    print(m)
    return m, c

def log_func(x, a, b, k):
    return a * np.log(b * abs(x)) + k

def log_linear_regression(x, y):
    coeff, _ = curve_fit(log_func, x, y)
    print (coeff)
    return coeff

def poly3_regression(x, y):
    coeff = np.polyfit(x, y, 3)
    print (coeff)
    return coeff

def poly5_regression(x, y):
    coeff = np.polyfit(x, y, 5)
    print (coeff)
    return coeff

m, c = linear_regression(ft, skin)
model_l = {'m': m, 'c': c}
with open("fitted_model_" + filename + ".pickle", 'wb') as handle:
    pickle.dump(model_l, handle, protocol=pickle.HIGHEST_PROTOCOL)

a0, a1, a2, a3 = poly3_regression(ft, skin)
model_p3 = {'a0': a0, 'a1': a1, 'a2': a2, 'a3': a3}
with open("fitted_poly3_model_" + filename + ".pickle", 'wb') as handle:
    pickle.dump(model_p3, handle, protocol=pickle.HIGHEST_PROTOCOL)

c0, c1, c2, c3, c4, c5 = poly5_regression(ft, skin)
model_p5 = {'c0': c0, 'c1': c1, 'c2': c2, 'c3': c3, 'c4': c4, 'c5': c5}
with open("fitted_poly5_model_" + filename + ".pickle", 'wb') as handle:
    pickle.dump(model_p5, handle, protocol=pickle.HIGHEST_PROTOCOL)

fig, (ax1, ax2,) = plt.subplots(2)
ax1.plot(ft, skin, 'o' 'c', label='Original data', markersize=2)
ax1.set_xlabel('Forque readings')
ax1.set_ylabel('Skin readings')
ax1.set_title('Forque vs. Skin data')
ax1.legend()

ratio = (max(skin)-min(skin))/(min(ft)-max(ft))

ax2.plot(np.array(data['skin_time']), skin, 'c', label='Skin data')
ax2.plot(np.array(data['skin_time']), ft*ratio, 'r', label='Forque data (scaled by %f)' %ratio)
ax2.set_xlabel('Time')
ax2.set_ylabel('Skin readings')
ax2.set_title('Skin vs. Time')
ax2.legend()

plt.show()

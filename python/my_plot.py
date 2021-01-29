#!/usr/bin/env python

# > pip install matplotlib
# > pip install numpy

import numpy as np
import matplotlib.pyplot as plt

x = np.arange(0, 4*np.pi, 0.1)   # start,stop,step
y = np.sin(x)
z = np.cos(x)

plt.plot(x, y, label='Z', linewidth=2.0, color='black', linestyle='dashed')
plt.show()


plt.plot(x, y, x, z)
# string must be enclosed with quotes '  '
plt.xlabel('x values from 0 to 4pi')
plt.ylabel('sin(x) and cos(x)')
plt.title('Plot of sin and cos from 0 to 4pi')
# legend entries as seperate strings in a list
plt.legend(['sin(x)', 'cos(x)'])
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


def computeCoef(x, y):
    # http://www2.lawrence.edu/fast/GREGGJ/CMSC210/arithmetic/interpolation.html
    try:
        some = np.matrix(
            [[x[0] ** 2, x[0], 1], [x[1] ** 2, x[1], 1], [x[2] ** 2, x[2], 1]]
        )
        some2 = np.linalg.inv(some)
        some3 = np.matrix([y[0], y[1], y[2]]).T
    except:
        some = np.matrix(
            [[y[0] ** 2, y[0], 1], [y[1] ** 2, y[1], 1], [y[2] ** 2, y[2], 1]]
        )
        some2 = np.linalg.inv(some)
        some3 = np.matrix([x[0], x[1], x[2]]).T
    coef = some2 * some3
    return coef.A1


# Sample x and y data
x = np.array([15.215215215215217, 22.122122122122125, 29.52952952952953])
y = np.array([16.982723891176295, 19.90793340510944, 18.438007707275883])

# x = np.array([4.44125, 8.8825, 17.765])
# y = np.array([1.74375, 3.4875, 6.95])

# Testing diff things 
# x = np.array([0, 10, 0])
# y = np.array([20, 10, 0])
# coefficients = computeCoef(x, y)

# Fit a polynomial curve to the data
coefficients = np.polyfit(x, y, 2)  # fit a 2nd order polynomial (quadtratic)

# Generate points along the fitted curve
x_fit = np.linspace(x.min(), x.max() + 5, 100)
y_fit = np.polyval(coefficients, x_fit)

# Plot the data and fitted curve
plt.scatter(x, y, label="Data")
plt.plot(x_fit, y_fit, label="Fitted Curve")
plt.scatter([35.53553553553554], [13.959372470509138], label="next data")
plt.legend()
plt.show()

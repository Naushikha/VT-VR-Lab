import numpy as np
import matplotlib.pyplot as plt

quadType = 0


def computeCoef(x, y):
    global quadType
    # http://www2.lawrence.edu/fast/GREGGJ/CMSC210/arithmetic/interpolation.html
    try:
        some = np.matrix(
            [[x[0] ** 2, x[0], 1], [x[1] ** 2, x[1], 1], [x[2] ** 2, x[2], 1]]
        )
        some2 = np.linalg.inv(some)
        some3 = np.matrix([y[0], y[1], y[2]]).T
        quadType = 0
    except:  # sideways quadratic function
        some = np.matrix(
            [[y[0] ** 2, y[0], 1], [y[1] ** 2, y[1], 1], [y[2] ** 2, y[2], 1]]
        )
        some2 = np.linalg.inv(some)
        some3 = np.matrix([x[0], x[1], x[2]]).T
        quadType = 1
    coef = some2 * some3
    return coef.A1


# Sample x and y data
x = [15.215215215215217, 22.122122122122125, 29.52952952952953]
y = [16.982723891176295, 19.90793340510944, 18.438007707275883]
nextAIS = [35.53553553553554, 13.959372470509138]
# x = [4.44125, 8.8825, 17.765]
# y = [1.74375, 3.4875, 6.95]

# x = [0, 10, 0]
# y = [20, 10, 0]
# nextAIS = [-10, -10]

# x = [10, 0, -10]
# y = [10, 5, -10]
# nextAIS = [-20, -20]

x = np.array(x)
y = np.array(y)
coefficients = computeCoef(x, y)

# Fit a polynomial curve to the data using polyfit
# coefficients = np.polyfit(x, y, 2)  # fit a 2nd order polynomial (quadtratic)

# Generate points along the interpolation
if quadType == 0:
    x_fit = np.linspace(x.min() - 5, x.max() + 5, 100)
if quadType == 1:
    x_fit = np.linspace(y.min() - 5, y.max() + 5, 100)
y_fit = np.polyval(coefficients, x_fit)

# Plot the data and interpolation
plt.scatter(x, y, label="Available AIS Reports")
plt.scatter(nextAIS[0], nextAIS[1], label="Next AIS Report")
if quadType == 0:
    plt.plot(x_fit, y_fit, label="Quadtratic Interpolation")
if quadType == 1:
    plt.plot(y_fit, x_fit, label="Quadtratic Interpolation")
plt.legend()
plt.show()

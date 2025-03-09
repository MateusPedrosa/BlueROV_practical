import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Step 1: Load the data from the Excel file
file_path = "/home/mateus/Documents/bluerov/T200-Public-Performance-Data-10-20V-September-2019.xlsx"  # Replace with your file path
data = pd.read_excel(file_path, sheet_name='12 V')

# Assuming the Excel file has two columns: 'x' and 'y'
print(data.columns)
x = data['Force (N)'].values
y = data['PWM (µs)'].values

# Step 2: Separate the data into two sets
# Curve 1: From 0 onwards (x >= 0)
x_positive = x[x >= 0]
y_positive = y[x >= 0]

# Curve 2: From 0 backwards (x < 0)
x_negative = x[x < 0]
y_negative = y[x < 0]

# Step 3: Perform linear regression for each curve
# For the positive curve
reg_positive = LinearRegression()
reg_positive.fit(x_positive.reshape(-1, 1), y_positive)

# For the negative curve
reg_negative = LinearRegression()
reg_negative.fit(x_negative.reshape(-1, 1), y_negative)

# Get the equations of the lines
# Positive curve: y = m1 * x + b1
m1 = reg_positive.coef_[0]
b1 = reg_positive.intercept_
print(f"Positive thrust equation: y = {m1:.2f}f + {b1:.2f}")

# Negative curve: y = m2 * x + b2
m2 = reg_negative.coef_[0]
b2 = reg_negative.intercept_
print(f"Negative thrust equation: y = {m2:.2f}f + {b2:.2f}")

# Step 4: Plot the original data and the interpolated lines
plt.plot(x_positive, y_positive, color='blue', label='Positive Thrust Original')#s=5)
plt.plot(x_negative, y_negative, color='red', label='Negative Thrust Original')# s=5)

# Plot the interpolated lines
x_fit_positive = np.linspace(min(x_positive), max(x_positive), 100)
y_fit_positive = m1 * x_fit_positive + b1
plt.plot(x_fit_positive, y_fit_positive, color='blue', linestyle='--', label='Positive Thrust Interpolated')

x_fit_negative = np.linspace(min(x_negative), max(x_negative), 100)
y_fit_negative = m2 * x_fit_negative + b2
plt.plot(x_fit_negative, y_fit_negative, color='red', linestyle='--', label='Negative Thrust Interpolated')

plt.ylabel('ESC PWM (µs)')
plt.xlabel('Thrust (N)')
plt.title('ESC PWM vs Thrust')
plt.legend()
plt.grid()
plt.show()

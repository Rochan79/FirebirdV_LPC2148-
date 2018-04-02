
import matplotlib.pyplot as plt
import numpy as np
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import train_test_split

dataset_x=[left_distance,centre_distance,right_distance]
dataset_y=[turn_mode,Left_PWM,Right_PWM]

X_train, X_test, y_train, y_test = train_test_split(dataset_x, dataset_y, test_size=0.33, random_state=42)

regr = linear_model.LinearRegression()

regr.fit(X_train,y_train)

y_predict=regr.predict(X_test)


print("Mean squared error: %.2f" % mean_squared_error(y_test, y_predict))

plt.scatter(X_test, y_test,  color='black')
plt.plot(X_test, y_pred, color='blue', linewidth=3)

plt.xticks(())
plt.yticks(())

plt.show()

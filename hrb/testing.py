from numpy import mean, array


a = array([[1, 1070], [2, 1091], [3, 1115], [4, 1093]])


print(a[:,0])
m = mean(a[:,0])
print(m)
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def lcg(seed, size): 
    m = 2**31
    a = 1103515245
    c = 12345
    randlist = []
    randlist.append(seed)
    for i in range(1, size): 
        randlist.append((randlist[i-1] * a + c) % m) 
    return randlist


x = lcg(22, 1000)
y = lcg(155, 1000)
z = lcg(54, 1000)

fig = plt.figure()
ax2 = Axes3D(fig)
ax2.scatter(x, y, z)
plt.show()

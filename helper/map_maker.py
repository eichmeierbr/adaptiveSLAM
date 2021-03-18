import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


grid = np.zeros([800,800])

# Remove all hallways
# img=mpimg.imread('building_with_rooms.png')
img = mpimg.imread('maps/hallways.png')
img = 1-img[:,:,0]

plt.imshow(img, cmap='Greys')
plt.show()

# np.savetxt('buliding_easy.csv', img, delimiter=',')
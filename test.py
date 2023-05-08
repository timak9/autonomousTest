import random
import numpy as np



randomList = np.random.uniform(16,100,1000)
randomList2 = np.random.normal(16,100,1000)

randomList.sort()
randomList2.sort()

print(randomList,randomList.mean())
print(randomList2,randomList2.mean())



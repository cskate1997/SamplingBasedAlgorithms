import numpy as np
from scipy import spatial
import networkx as nx
import matplotlib.pyplot as plt
nnodes = 30
r = 0.15
positions =  np.random.rand(nnodes,2) 
kdtree = spatial.KDTree(positions)
pairs = list(kdtree.query_pairs(r))
print("this is pairs", pairs)











# pts = [(1,4), (7,6), (9,0), (7,9)]
# pairs=[]
# pairs_without_weights=[]
# for k in pts:
#     z = [k]
#     set1 = set(pts)    # output {(7, 9), (9, 0), (7, 6), (1, 4)} 
#     set2 = set(z)      # output  {(1, 4)}
#     res = list(set1-set2)     # output  [(7, 9), (9, 0), (7, 6)]
#     print("this is ans",set1,set2,res)
#     for j in res:
#         if self.check_collision(k,j)==False:
#             d = self.dis(k,j)
#             pairs_without_weights.append((k,j))
#             pairs.append((k,j,d))
#         elif:
#             continue
#         else:
#             pts.remove(k)



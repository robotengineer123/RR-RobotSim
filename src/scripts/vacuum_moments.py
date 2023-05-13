import numpy as np

#%% Get the average moment arms
edge1 = np.sqrt((0.099*2)**2 + (0.135*2)**2)
edge2 = np.sqrt((0.099*2)**2 + (0.135)**2)
edge3 = (0.099*2)
edge4 = edge2
edge5 = edge1
edge_arm = (edge1+edge2+edge3+edge4+edge5)/5

center1 = np.sqrt((0.135*2)**2 + (0.099)**2)
center2 = np.sqrt((0.135)**2 + (0.099)**2)
center3 = 0.099
center4 = center2
center5 = center1
center6 = 0.135*2
center7 = 0.135
center8 = 0
center9 = center7
center10 = center6
center11 = center1
center12 = center2
center13 = center3
center14 = center4
center15 = center5
center_arm = (center1+center2+center3+center4+center5+center6+center7+center8+center9+center10+center11+center12+center13+center14+center15)/15

total = (edge_arm + center_arm)/2

#%% Calculate moment contributions from edge and center
center_vac = 7
edge_vac = 11
center = 0.121469
edge = 0.076674

F = 700

N_center = (center_vac*1013)*center
N_side = (edge_vac*1013)*edge
N = N_center + N_side

M_edge = edge_arm*N_side/N*F
M_center = center_arm*N_center/N*F
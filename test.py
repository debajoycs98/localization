import numpy as np
from scipy.spatial import KDTree

def find_matches_with_kdtree(shorter_list, longer_list, tolerance=0.5):
    # Convert lists to numpy arrays for efficiency and compatibility with scipy
    shorter_array = np.array(shorter_list)
    longer_array = np.array(longer_list)
    
    # Create a KDTree from the longer list
    tree = KDTree(longer_array)
    
    # For each point in the shorter list, query the tree for any points in the longer list within the tolerance
    matches = []
    for point in shorter_array:
        # Query the tree for points within the tolerance
        indices = tree.query_ball_point(point, tolerance)
        for i in indices:
            matches.append((tuple(point), tuple(longer_array[i])))
            
    return matches


shorter_list = [(1,2),(2,4),(3,3)]
longer_list = [(1,2.3),(1.7,3.3),(2.9,3.5),(2.7,2.8)]
print(find_matches_with_kdtree(shorter_list,longer_list))
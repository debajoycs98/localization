# # import math
# # import matplotlib.pyplot as plt

# # def cells_intersected_by_circle(center, radius):
# #     (cx, cy) = center
# #     cells = set()
    
# #     # Determine the bounding box of the circle
# #     min_x = math.floor(cx - radius)
# #     max_x = math.ceil(cx + radius)
# #     min_y = math.floor(cy - radius)
# #     max_y = math.ceil(cy + radius)
    
# #     # Iterate over each cell in the bounding box
# #     for x in range(min_x, max_x + 1):
# #         for y in range(min_y, max_y + 1):
# #             # Check if the center of the cell is within the circle's radius
# #             if math.sqrt((x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2) <= radius:
# #                 cells.add((x, y))
                
# #     return cells

# # # Example usage
# # center = (0, 0)
# # radius = 2
# # intersected_cells = cells_intersected_by_circle(center, radius)
# # print(intersected_cells)

# # Obstacle 1: A circle with radius 0.75 centered at (2, 2)
# # Obstacle 2: A circle with radius 0.75 centered at (-2, 2)
# # Obstacle 3: A circle with radius 0.75 centered at (2, -2)

# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# import math

# def cells_intersected_by_circle(center, radius):
#     (cx, cy) = center
#     cells = set()
    
#     # Determine the bounding box of the circle
#     min_x = math.floor(cx - radius)
#     max_x = math.ceil(cx + radius)
#     min_y = math.floor(cy - radius)
#     max_y = math.ceil(cy + radius)
    
#     # Iterate over each cell in the bounding box
#     for x in range(min_x, max_x + 1):
#         for y in range(min_y, max_y + 1):
#             # Check if the center of the cell is within the circle's radius
#             if math.sqrt((x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2) <= radius:
#                 cells.add((x, y))
                
#     return cells

# # Define centers and radii for multiple circles
# circles = [((2, 2), 0.75), ((2, -2), 0.75), ((-2, 2), 0.75)]

# # Create figure and axes
# fig, ax = plt.subplots()

# # Process each circle
# for center, radius in circles:
#     intersected_cells = cells_intersected_by_circle(center, radius)
    
#     # Plot each cell for the current circle
#     for (x, y) in intersected_cells:
#         ax.add_patch(patches.Rectangle((x, y), 1, 1, fill=None, edgecolor='red', lw=1.5))
    
#     # Plot the circle
#     circle = plt.Circle(center, radius, color='blue', fill=False, linewidth=2)
#     ax.add_artist(circle)

# # Set limits for x and y axes dynamically based on the circles
# ax.set_xlim(min([c[0] - r - 1 for c, r in circles]), max([c[0] + r + 1 for c, r in circles]))
# ax.set_ylim(min([c[1] - r - 1 for c, r in circles]), max([c[1] + r + 1 for c, r in circles]))

# # Set aspect of the plot to be equal
# ax.set_aspect('equal', adjustable='datalim')

# # Show grid
# ax.grid(True)

# # Show the plot
# plt.show()



import math

def cells_intersected_by_circle(center, radius):
    (cx, cy) = center
    cells = set()
    
    # Determine the bounding box of the circle
    min_x = math.floor(cx - radius)
    max_x = math.ceil(cx + radius)
    min_y = math.floor(cy - radius)
    max_y = math.ceil(cy + radius)
    
    # Iterate over each cell in the bounding box
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            # Check if the center of the cell is within the circle's radius
            if math.sqrt((x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2) <= radius:
                cells.add((x, y))
                
    return cells

def combined_intersected_cells(circles):
    combined_cells = set()
    for center, radius in circles:
        intersected_cells = cells_intersected_by_circle(center, radius)
        combined_cells.update(intersected_cells)
    return combined_cells

# Define centers and radii for multiple circles
circles = [((2, 2), 0.75), ((2, -2), 0.75), ((-2, 2), 0.75)]


# Get the combined set of intersected cells
final_intersected_cells = combined_intersected_cells(circles)
print(final_intersected_cells)

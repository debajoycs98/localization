# # import math

# # def line_intersects_grid(x0, y0, theta, width, height):
# #     """Calculate intersections with the grid boundaries based on an angle theta."""
# #     dx, dy = math.cos(theta), math.sin(theta)
# #     intersections = []

# #     # Right and left boundaries
# #     if dx != 0:
# #         # Right boundary (x = width)
# #         t = (width - x0) / dx
# #         y = y0 + t * dy
# #         if 0 <= y <= height:
# #             intersections.append((width, int(y)))
# #         # Left boundary (x = 0)
# #         t = -x0 / dx
# #         y = y0 + t * dy
# #         if 0 <= y <= height:
# #             intersections.append((0, int(y)))

# #     # Top and bottom boundaries
# #     if dy != 0:
# #         # Top boundary (y = height)
# #         t = (height - y0) / dy
# #         x = x0 + t * dx
# #         if 0 <= x <= width:
# #             intersections.append((int(x), height))
# #         # Bottom boundary (y = 0)
# #         t = -y0 / dy
# #         x = x0 + t * dx
# #         if 0 <= x <= width:
# #             intersections.append((int(x), 0))

# #     # Return the closest intersection point to the starting point (x0, y0)
# #     return min(intersections, key=lambda p: (p[0]-x0)**2 + (p[1]-y0)**2) if intersections else None
# # def bresenham_line(x0, y0, x1, y1):
# #     """ Bresenham's Line Algorithm Producing a list of tuples from (x0, y0) to (x1, y1) """
# #     cells = []
# #     dx = abs(x1 - x0)
# #     dy = -abs(y1 - y0)
# #     sx = 1 if x0 < x1 else -1
# #     sy = 1 if y0 < y1 else -1
# #     err = dx + dy

# #     while True:
# #         cells.append((x0, y0))
# #         if x0 == x1 and y0 == y1:
# #             break
# #         e2 = 2 * err
# #         if e2 >= dy:
# #             err += dy
# #             x0 += sx
# #         if e2 <= dx:
# #             err += dx
# #             y0 += sy
# #     return cells
# # def main(x0, y0, theta):
# #     """Compute and print cells on the line defined by starting point (x0, y0) and angle theta."""
# #     width, height = 50, 50
# #     endpoint = line_intersects_grid(x0, y0, theta, width, height)
# #     if endpoint:
# #         cells = bresenham_line(x0, y0, endpoint[0], endpoint[1])
# #         return cells
# #     else:
# #         return []

# # # Example usage
# # x0, y0 = 25, 25  # Starting point
# # theta = math.pi / 4  # Angle in radians (45 degrees)
# # cells = main(x0, y0, theta)
# # print("Cells on line:", cells)

# import math
# # import numpy as np

# # def find_grid_intersections(x0, y0, theta, grid_width, grid_height):
# #     dx, dy = math.cos(theta), math.sin(theta)
# #     intersections = []

# #     # Function to check if a point is within grid boundaries
# #     def is_within_grid(x, y):
# #         return 0 <= x <= grid_width and 0 <= y <= grid_height

# #     # Checking intersections with vertical grid boundaries (left and right)
# #     if dx != 0:
# #         for x_bound in [0, grid_width]:
# #             t = (x_bound - x0) / dx
# #             y = y0 + t * dy
# #             if is_within_grid(x_bound, y):
# #                 intersections.append((x_bound, y))

# #     # Checking intersections with horizontal grid boundaries (top and bottom)
# #     if dy != 0:
# #         for y_bound in [0, grid_height]:
# #             t = (y_bound - y0) / dy
# #             x = x0 + t * dx
# #             if is_within_grid(x, y_bound):
# #                 intersections.append((x, y_bound))

# #     if not intersections:
# #         return None

# #     # Choose the closest intersection point to the starting point (x0, y0)
# #     closest_point = min(intersections, key=lambda p: (p[0] - x0)**2 + (p[1] - y0)**2)
# #     return closest_point

# # def bresenham_line(x0, y0, x1, y1):
# #     """ Bresenham's Line Algorithm: Producing a list of tuples from (x0, y0) to (x1, y1) """
# #     x0, y0, x1, y1 = map(int, map(round, [x0, y0, x1, y1]))  # Ensure coordinates are integers
# #     cells = []
# #     dx = abs(x1 - x0)
# #     dy = -abs(y1 - y0)
# #     sx = 1 if x0 < x1 else -1
# #     sy = 1 if y0 < y1 else -1
# #     err = dx + dy

# #     while True:
# #         cells.append((x0, y0))
# #         if x0 == x1 and y0 == y1:
# #             break
# #         e2 = 2 * err
# #         if e2 >= dy:
# #             err += dy
# #             x0 += sx
# #         if e2 <= dx:
# #             err += dx
# #             y0 += sy
# #     return cells

# # def main(x0, y0, theta):
# #     grid_width, grid_height = 50, 50
# #     endpoint = find_grid_intersections(x0, y0, theta, grid_width, grid_height)
# #     if endpoint:
# #         cells = bresenham_line(x0, y0, endpoint[0], endpoint[1])
# #         return cells
# #     else:
# #         return []

# # # Example usage
# # x0, y0 = 25, 25  # Starting point
# # theta = math.radians(math.pi/4)  # Angle in radians (90 degrees = vertical line)
# # cells = main(x0, y0, theta)
# # print("Cells on line:", cells)

# def get_grid_cells(x0, y0, theta, grid_width, grid_height):
#     """ Returns all grid cells a line passes through given a start point and an angle within grid boundaries. """
#     cells = []

#     # Step size along the line
#     step_size = 0.5  # Smaller step size for higher resolution
#     dx, dy = math.cos(theta), math.sin(theta)

#     x, y = x0, y0
#     cells.append((int(x), int(y)))  # Start cell

#     # Continue stepping until we reach the boundary
#     while 0 <= int(x) <= grid_width and 0 <= int(y) <= grid_height:
#         x += dx * step_size
#         y += dy * step_size
#         cell = (int(x), int(y))
#         if cell not in cells:  # Check to prevent duplicates
#             cells.append(cell)

#     return cells

# def main():
#     x0, y0 = 25, 25  # Start point at the center of the grid
#     theta = math.radians(math.pi/4)  # 90 degrees, vertical line
#     grid_width, grid_height = 50, 50  # Grid boundaries

#     # Get the cells through which the line passes
#     cells = get_grid_cells(x0, y0, theta, grid_width, grid_height)
#     print("Cells on line:", cells)

# main()

import math

def find_path_until_boundary(x, y, theta, xmax, ymax):
    # Convert angle from degrees to radians
    theta_rad = math.radians(theta)
    
    # Direction vector components
    dx = math.cos(theta_rad)
    dy = math.sin(theta_rad)
    
    # List to hold points
    points = []
    
    # Current position
    cx, cy = x, y
    
    while 0 <= cx <= xmax and 0 <= cy <= ymax:
        # Store current integer position if within bounds
        ix, iy = int(round(cx)), int(round(cy))
        if 0 <= ix <= xmax and 0 <= iy <= ymax:
            points.append((ix, iy))
        
        # Move to the next position
        cx += dx
        cy += dy
        
        # Stop if we go out of bounds to avoid infinite loop
        if not (0 <= cx <= xmax and 0 <= cy <= ymax):
            break
    
    return points

# Example usage
x, y, theta = 10, 10, 0  # Starting point and direction
xmax, ymax = 50, 50       # Boundary limits
path = find_path_until_boundary(x, y, theta, xmax, ymax)
print(path)

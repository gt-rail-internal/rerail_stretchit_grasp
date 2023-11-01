import numpy as np
import matplotlib.pyplot as plt

def plot_3d_vectors(vec1, vec2):
    """
    Plot two 3D vectors using matplotlib.

    Parameters:
    - vec1: tuple or list, first 3D vector to plot
    - vec2: tuple or list, second 3D vector to plot
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Quiver plot
    ax.quiver(0, 0, 0, vec1[0], vec1[1], vec1[2], color='b', label="Vector 1", arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, vec2[0], vec2[1], vec2[2], color='r', label="Vector 2", arrow_length_ratio=0.1)
    
    # Setting limits to ensure both vectors are visible
    max_limit = max(
        abs(vec1[0]), abs(vec1[1]), abs(vec1[2]),
        abs(vec2[0]), abs(vec2[1]), abs(vec2[2])
    )
    
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])
    ax.set_zlim([-max_limit, max_limit])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()

# Example vectors
vector1 = (-0.12641597, 0.99180459, -0.01851095)
vector2 = (0, 0, 1)
plot_3d_vectors(vector1, vector2)
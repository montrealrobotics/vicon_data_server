import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import numpy as np
from matplotlib.patches import Patch

# Create proxy artists for the legend
legend_elements = [
    Patch(facecolor="cyan", edgecolor="black", label="Position Data"),
    Patch(facecolor="magenta", edgecolor="black", label="Velocity Integration"),
]

class ViconAnimator:
    def __init__(self):
        # Set up the figure and 3D axis
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.ax.legend(handles=legend_elements, loc="upper left")

        self.box_vertices = [
            [-0.1, -0.13, -0.05],
            [ 0.1, -0.13, -0.05],
            [ 0.1,  0.13, -0.05],
            [-0.1,  0.13, -0.05],
            [-0.1, -0.13,  0.05],
            [ 0.1, -0.13,  0.05],
            [ 0.1,  0.13,  0.05],
            [-0.1,  0.13,  0.05]
        ]
        self.box_faces = [
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
            [0, 1, 2, 3],
            [4, 5, 6, 7]
        ]

        # Initialize the two objects
        self.box_reference = Poly3DCollection([], facecolors="cyan", edgecolors="black", alpha=0.8)
        self.box_velocity = Poly3DCollection([], facecolors="magenta", edgecolors="black", alpha=0.6)
        self.ax.add_collection3d(self.box_reference)
        self.ax.add_collection3d(self.box_velocity)

        self.position_velocity = np.array([0.0, 0.0, 0.0])
        self.orientation_velocity = R.from_quat([1, 0, 0, 0])
        self.initial_update = True

    def update(self, position, orientation, linear_velocity, angular_velocity, dt):
        # Update the position-driven object
        rotation_reference = R.from_quat(orientation)
        rot_matrix_ref = rotation_reference.as_matrix()
        transformed_vertices_ref = [
            [sum(rot_matrix_ref[i][j] * v[j] for j in range(3)) + position[i] for i in range(3)]
            for v in self.box_vertices
        ]
        self.box_reference.set_verts([[transformed_vertices_ref[i] for i in face] for face in self.box_faces])

        if self.initial_update:
            self.position_velocity = position
            self.orientation_velocity = R.from_quat(orientation)
            self.ax.set_xlim(position[0]- 1, position[0] + 1)
            self.ax.set_ylim(position[1]- 1, position[1] + 1)
            self.ax.set_zlim(0, 2)
            self.initial_update = False
            
        # Update the velocity-driven object (integrate position and orientation)
        self.position_velocity += np.array(linear_velocity) * dt
        angular_velocity = np.array(angular_velocity)
        angle = np.linalg.norm(angular_velocity) * dt
        if angle > 1e-6:
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            delta_rotation = R.from_rotvec(axis * angle)
            self.orientation_velocity = delta_rotation * self.orientation_velocity
        rot_matrix_vel = self.orientation_velocity.as_matrix()
        transformed_vertices_vel = [
            [sum(rot_matrix_vel[i][j] * v[j] for j in range(3)) + self.position_velocity[i] for i in range(3)]
            for v in self.box_vertices
        ]
        self.box_velocity.set_verts([[transformed_vertices_vel[i] for i in face] for face in self.box_faces])

    def show(self):
        plt.legend()
        plt.show()

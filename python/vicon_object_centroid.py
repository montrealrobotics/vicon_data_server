import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def plot_flat_plane(ax, plane_point, size=500, resolution=10):
    """
    Plot a plane in 3D, z values equal to the z in plane_point.

    Args:
        ax: The 3D matplotlib axis.
        plane_point (numpy.ndarray): A point on the plane (centroid of the points).
        size (float): The size of the plane.
        resolution (int): The resolution of the plane grid.
    """
    # Create a grid of points for the plane
    xx, yy = np.meshgrid(
        np.linspace((plane_point[0] - size), plane_point[0] + size, resolution),
        np.linspace(plane_point[1] - size, plane_point[1] + size, resolution)
    )
    # Calculate the corresponding z values for the plane
    zz = np.full_like(xx, plane_point[2])

    # Plot the plane
    ax.plot_surface(xx, yy, zz, alpha=0.3, color='lightblue', rstride=100, cstride=100)

def fit_plane(points):
    """
    Find best fit plane to points.

    Args:
        points (numpy.ndarray): Array of points on the plane surface.

    Returns:
        centroid (numpy.ndarray): A point on the plane (origin of the new frame).
        normal (numpy.ndarray): The normal vector of the plane (defines Z-axis).
    """
    # Compute the centroid of the points
    centroid = np.mean(points, axis=0)

    # Find the plane's normal
    _, _, vh = np.linalg.svd(points - centroid)
    normal = vh[-1]

    return centroid, normal

def transform_to_plane_frame(points, plane_point, plane_normal):
    """
    Transform the points to a coordinate frame aligned with the plane, ensuring the Y-axis
    aligns with the direction from the front marker to the back marker.

    Args:
        points (numpy.ndarray): Array of points (4 x 3) to be transformed [left, right, front, back].
        plane_point (numpy.ndarray): A point on the plane (origin of the new frame).
        plane_normal (numpy.ndarray): The normal vector of the plane (defines Z-axis).

    Returns:
        numpy.ndarray: Transformed points in the new coordinate frame.
        numpy.ndarray: Rotation matrix of the new frame.
    """
    # Normalize the plane normal to define the Z-axis
    z_axis = plane_normal / np.linalg.norm(plane_normal)

    # Extract front and back markers from the points array
    front_marker = points[2]
    back_marker = points[3]

    # Translate the front and back markers to the plane's origin
    front_marker_translated = front_marker - plane_point
    back_marker_translated = back_marker - plane_point

    # Project the translated markers onto the plane
    def project_onto_plane(point, normal):
        normal = normal / np.linalg.norm(normal)
        distance = np.dot(point, normal)
        return point - distance * normal

    front_marker_projected = project_onto_plane(front_marker_translated, z_axis)
    back_marker_projected = project_onto_plane(back_marker_translated, z_axis)

    # Compute the Y-axis from the projected front and back markers
    y_axis = front_marker_projected - back_marker_projected
    y_axis = y_axis / np.linalg.norm(y_axis)

    # Compute the X-axis as the cross product of Y and Z
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)

    # Recompute the Y-axis to ensure orthogonality
    y_axis = np.cross(z_axis, x_axis)

    R = np.vstack([x_axis, y_axis, z_axis]).T

    points_translated = points - plane_point
    points_transformed = points_translated @ R

    # Set z to zero to position all points on the plane.
    points_transformed[:, 2] = 0

    return points_transformed, R


def calculate_cuboid_center(top_markers, side_markers, cuboid_dimensions):
    """
    Calculate the center of a cuboid by projecting side markers onto a best-fit top plane, 
    front and back markers should be aligned with y axis.

    Args:
        top_markers (list): A list of four numpy arrays defining the top plane in global coordinates.
        side_markers (list): A dictionary containing the global positions of side markers as numpy arrays.
        cuboid_dimensions (dict): A dictionary with the cuboid's known dimensions.
                                  Keys: 'width', 'depth', 'height'.

    Returns:
        numpy.ndarray: The global coordinates of the cuboid's center.
        numpy.ndarray: The global orientation of the cuboid's center.
        side_markers (list): A list of four numpy arrays containing the positions of the side markers in the cuboid frame.
    """
    # Fit a plane to the top markers
    top_markers_array = np.array(top_markers)
    plane_point, normal = fit_plane(top_markers_array)

    avg_top_marker_z = np.mean(top_markers_array[:, 2])
    avg_side_marker_z = np.mean(np.array(side_markers)[:, 2])

    # Normal could be pointing up or down, fix it to point up (direction away from side markers)
    if avg_top_marker_z > avg_side_marker_z and normal[2] < 0:
        normal = -normal

    points_transformed, R = transform_to_plane_frame(side_markers, plane_point, normal)

    # Project side points to the top plane
    left_projected = points_transformed[0]
    right_projected = points_transformed[1]
    front_projected = points_transformed[2]
    back_projected = points_transformed[3]

    # We take the front-back direction as the y axis, we calculate the width by projecting the left marker 
    # along the x axis to the edge defined by the right marker
    mid_depth = (front_projected + back_projected) / 2

    direction_y = [0, 1, 0]
    direction_x = [1, 0, 0]

    left_projected, right_projected, direction_y, direction_x = map(np.array, [left_projected, right_projected, direction_y, direction_x])

    numerator = np.dot(right_projected - left_projected, direction_y)
    denominator = np.dot(direction_y, direction_y)  # Equivalent to ||direction_v||^2
    s = -numerator / denominator

    intersection_point = right_projected + s * direction_y

    perpendicular_vector = intersection_point - left_projected
    calculated_width = np.abs(np.dot(perpendicular_vector, direction_x))

    # Calculated offset due to marker size, we use the depth measurement, possible it is more reliable
    marker_depth_offset = (front_projected[1] - back_projected[1] - cuboid_dimensions['depth'])/2
    marker_width_offset = (calculated_width - cuboid_dimensions['width'])/2

    temp_point = left_projected + (calculated_width/2 * direction_x)

    adjustment = np.dot(temp_point - mid_depth, direction_y) * direction_y

    top_plane_center = temp_point - adjustment

    cuboid_center = [top_plane_center[0], top_plane_center[1], top_plane_center[2] - ((cuboid_dimensions['height'] / 2) + marker_depth_offset)]
    cuboid_center_global = cuboid_center @ R.T + plane_point

    r =  Rotation.from_matrix(R)
    yaw, pitch, roll = r.as_euler("zyx",degrees=True)
    cuboid_orientation = [roll, pitch, yaw]

    view_cuboid(top_markers, side_markers, cuboid_center_global, R, top_plane_center, right_projected, left_projected, 
                front_projected, back_projected, normal, cuboid_center, cuboid_dimensions)

    side_points_translated = side_markers - cuboid_center_global
    side_points_transformed = side_points_translated @ R

    return cuboid_center_global, cuboid_orientation, side_points_transformed

def cuboid_model(center, rotation_matrix, dimensions):
    """
    Generate the 8 corners of the cuboid based on center, orientation, and dimensions.

    Args:
        center (list): Center of the cuboid [x, y, z].
        rotation_matrix (numpy.ndarray or list): Rotation matrix (3x3)
        dimensions (dict): Dimensions of the cuboid width, depth, height.

    Returns:
        numpy.ndarray: Global coordinates of the cuboid corners.
    """
    width, depth, height = dimensions['width'], dimensions['depth'], dimensions['height']
    corners_local = np.array([
        [-width / 2, -depth / 2, -height / 2],
        [-width / 2, -depth / 2,  height / 2],
        [-width / 2,  depth / 2, -height / 2],
        [-width / 2,  depth / 2,  height / 2],
        [ width / 2, -depth / 2, -height / 2],
        [ width / 2, -depth / 2,  height / 2],
        [ width / 2,  depth / 2, -height / 2],
        [ width / 2,  depth / 2,  height / 2],
    ])

    # Apply rotation and translation
    corners_global = (rotation_matrix @ corners_local.T).T + center
    return corners_global


def view_cuboid(top_markers_global, side_markers_global, cuboid_center_global, rotation_matrix, top_plane_center, 
                                   right_projected, left_projected, front_projected, back_projected, normal, cuboid_center, dimensions):
    """
    View plots to check calculations

    Args:
        top_markers_global (list): List of numpy arrays, global position of top markers
        side_markers_global (list): List of numpy arrays, global position of side markers
        cuboid_center_global (numpy.array): The calculated center of the cuboid [x, y, z].
        cuboid_center (list): Center of the cuboid [x, y, z].
        rotation_matrix (numpy.ndarray): Rotation matrix (3x3)
        dimensions (dict): Dimensions of the cuboid width, depth, height.
        top_plane_center (numpy.array): The calculated center of the top plane [x, y, z].
        right_projected (numpy.array): Right side marker projected onto top plane.
        left_projected (numpy.array): Left side marker projected onto top plane.
        front_projected (numpy.array): Front side marker projected onto top plane.
        back_projected (numpy.array): Back side marker projected onto top plane.
        normal (numpy.array): Top plane nurmal.

    Returns:
        N/A
    """
    fig = plt.figure()
    ax = fig.add_subplot(211, projection='3d')
    bx = fig.add_subplot(212, projection='3d')

    ax.title.set_text('Global position')
    bx.title.set_text('Projected to top plane')

    # Plot the fitted plane
    plot_flat_plane(bx, plane_point=[0, 0, 0], size=300)

    # Plot projected points
    projected_points = [left_projected, right_projected, front_projected, back_projected]
    for point, label in zip(projected_points, ['Left Projected', 'Right Projected', 'Front Projected', 'Back Projected']):
        bx.scatter(point[0], point[1], point[2], c='green', label=label, marker='x', s=100)

    # Plot top plane center point
    bx.scatter(
        top_plane_center[0], top_plane_center[1], top_plane_center[2],
        c='orange', label='Top Plane Center', s=100
    )

    # Plot cuboid center
    bx.scatter(
        cuboid_center[0], cuboid_center[1], cuboid_center[2],
        c='purple', label='Cuboid Center', s=100
    )

    bx.set_xlabel('X')
    bx.set_ylabel('Y')
    bx.set_zlabel('Z')
    bx.legend()

    # Plot global top markers
    top_markers_global_array = np.array(top_markers_global)
    ax.scatter(
        top_markers_global_array[:, 0], top_markers_global_array[:, 1], top_markers_global_array[:, 2],
        c='blue', label='Top Markers (Global)'
    )

    # Plot global side markers
    side_markers_global_array = np.array(side_markers_global)
    ax.scatter(
        side_markers_global_array[:, 0], side_markers_global_array[:, 1], side_markers_global_array[:, 2],
        c='red', label='Side Markers (Global)'
    )

    # Plot cuboid center in global frame
    ax.scatter(
        cuboid_center_global[0], cuboid_center_global[1], cuboid_center_global[2],
        c='purple', label='Cuboid Center (Global)', s=100
    )
    scale = 30

    x_axis = rotation_matrix[:, 0] * scale
    y_axis = rotation_matrix[:, 1] * scale
    z_axis = rotation_matrix[:, 2] * scale

    ax.quiver(
        cuboid_center_global[0], cuboid_center_global[1], cuboid_center_global[2],
        x_axis[0], x_axis[1], x_axis[2],
        color='red', label='X-axis (Local)'
    )

    ax.quiver(
        cuboid_center_global[0], cuboid_center_global[1], cuboid_center_global[2],
        y_axis[0], y_axis[1], y_axis[2],
        color='green', label='Y-axis (Local)'
    )

    ax.quiver(
        cuboid_center_global[0], cuboid_center_global[1], cuboid_center_global[2],
        z_axis[0], z_axis[1], z_axis[2],
        color='blue', label='Z-axis (Local)'
    )

    corners = cuboid_model(cuboid_center_global, rotation_matrix, dimensions)
    edges = [
        [0, 1], [1, 3], [3, 2], [2, 0],
        [4, 5], [5, 7], [7, 6], [6, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges:
        ax.plot(
            [corners[edge[0], 0], corners[edge[1], 0]],
            [corners[edge[0], 1], corners[edge[1], 1]],
            [corners[edge[0], 2], corners[edge[1], 2]],
            color='blue'
        )

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


if __name__ == "__main__":

    # From Vicon tracker, read the values of 4 top plane markers, 1 marker on each side of robot trunk, 1 marker front and back of truck,
    # aligned with the y axis of trunk.
    # Also get global position of Vicon's calculated center for object.

    # global_pos = [2388.328, 777.810, 113.259]
    # top_markers = [
    #     np.array([-34.1539, -2.64065, 20.1158]),
    #     np.array([14.857, -30.4441, 20.2557]),
    #     np.array([-30.7031, 92.9818, 17.5662]),
    #     np.array([-0.246472, 95.5248, 17.9662])
    # ]

    # # Example side markers
    # side_markers = {
    #     'left': np.array([94.77, 2.37079, -47.5506]),
    #     'right': np.array([-120.225, 2.58994, -46.3027]),
    #     'front': np.array([45.4568, -155.815, -0.464948]),
    #     'back': np.array([46.3402, 121.81, -2.84189])}

    global_pos = [2681.386, 283.077, 115.218]

    top_markers = [
        np.array([-25.4645, 94.4702, 16.0725]),
        np.array([4.86339, 94.9891, 17.979]),
        np.array([-35.6532, 0.340924, 18.6627]),
        np.array([12.3102, -31.7899, 20.1318])
    ]

    side_markers = {
        'left': np.array([95.9398, -4.99776, -46.1972]),
        'right': np.array([-117.104, 8.29429, -51.512]),
        'front': np.array([35.7096, -157.118, 3.88501]),
        'back': np.array([53.5477, 120.051, 0.0377621])}

    top_markers_global = [global_pos + marker for marker in top_markers]

    side_markers_global = [global_pos + side_markers[marker] for marker in side_markers]

    # Known cuboid dimensions (width, depth, height). Measured from the go1 mesh file.
    cuboid_dimensions = {'width': 194, 'depth': 256, 'height': 114}

    center, orientation, side_transformed = calculate_cuboid_center(top_markers_global, side_markers_global, cuboid_dimensions)

    top_relative_to_center = [marker - center for marker in top_markers_global]
    side_relative_to_center = [marker - center for marker in side_markers_global]

    print("The global center of the object is:", center, end='\n\n')
    print("The global orientation of the object is:", orientation, end='\n\n')

    print("These values can be entered in the Vicon interface for the position/orientation of the tracked object.\n\n")
    print("To check the transform is correct, the positions of the side markers (can be seen by clicking on each marker) should now be the following:")

    print("Side markers relative to new center: ", side_transformed)

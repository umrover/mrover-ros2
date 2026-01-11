from dataclasses import dataclass, field


import numpy as np


@dataclass
class Trajectory:
    # Coordinates of the trajectory
    coordinates: np.ndarray
    # Currently tracked coordinate index along trajectory
    cur_pt: int = field(default=0, init=False)

    def get_current_point(self) -> np.ndarray:
        return self.coordinates[self.cur_pt]

    def increment_point(self) -> bool:
        """
        Increments the tracked point in the trajectory, returns true if
        the trajectory is finished
        """
        self.cur_pt = min(self.cur_pt + 1, len(self.coordinates))
        return self.cur_pt >= len(self.coordinates)

    def decerement_point(self) -> bool:
        """
        Increments the tracked point in the trajectory, returns true if
        the trajectory is finished
        """
        self.cur_pt = max(0, self.cur_pt - 1)
        return self.cur_pt <= 0

    def done(self) -> bool:
        return self.cur_pt >= len(self.coordinates)

    def empty(self) -> bool:
        return len(self.coordinates) == 0

    def is_last(self) -> bool:
        if len(self.coordinates) == 0:
            return True
        else:
            return self.cur_pt == len(self.coordinates) - 1

    def clear(self):
        self.coordinates = np.array([])
        self.cur_pt = 0

    def reset(self) -> None:
        """
        Resets the trajectory back to its start
        """
        self.cur_pt = 0


@dataclass
class SearchTrajectory(Trajectory):
    # Associated tag for this trajectory
    tag_id: int

    @classmethod
    def gen_spiral_coordinates(
        cls,
        coverage_radius: float,
        distance_between_spirals: float,
        num_segments_per_rotation: int,
        insert_extra: bool,
        inward_spiral: bool,
        start_radius: float,
        start_angle: float,
    ) -> np.ndarray:
        """
        Generates a set of coordinates for a spiral search pattern centered at the origin
        :param coverage_radius              radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiralradii = angles * (distance_between_spirals / (2*np.pi)) (float)
        :param num_segments_per_rotation:   number of segments that the spiral has per rotation (int)
        :return                             np.ndarray of coordinates
        """
        # The number of spirals should ensure coverage of the entire radius.
        # We add 1 to ensure that the last spiral covers the radius along the entire rotation,
        # as otherwise we will just make the outermost point touch the radius

        # If we have an inward spiral, we want it to begin at a set start radius.
        # So, we calculate the number of spirals needed to go from start_radius to 0
        if inward_spiral:
            num_spirals = np.ceil(start_radius / distance_between_spirals).astype("int") + 1

        # If it is an outward spiral, we simply calculate the number of spirals needed to go from 0 to coverage_radius set in the navigation.yaml file
        else:
            num_spirals = np.ceil(coverage_radius / distance_between_spirals).astype("int") + 1

        # The num_points variable is created as we used the below expression quite a bit in creating "angles" and "radii"
        # num_points is is simply the number of points we generate along the spiral
        num_points = num_segments_per_rotation * num_spirals + 1

        if inward_spiral:
            # angles are evenly spaced between the start angle and 2pi*num_segments_per_rotation
            # an angle is created for each point of the spiral (hence why we include num_points)
            # our start angle is start_angle as we first want to go to the closest point on circle's radius
            in_angles = np.linspace(start_angle, 2 * np.pi * num_spirals, num_points)
            # radii is simply evenly spaced "divisions" of the coverage_radius going inwards on each of the points in num_points

            # The in_radii is simply the reverse as the outward spiral radii, and we switch out
            # coverage_radius for start_radius as we want to go from start_radius to 0
            in_radii = np.linspace(start_radius, 0.0, num_points)
            in_x_coords = np.cos(in_angles) * in_radii
            in_y_coords = np.sin(in_angles) * in_radii
            vertices = np.hstack((in_x_coords.reshape(-1, 1), in_y_coords.reshape(-1, 1)))

        else:
            # angles are evenly spaced between the start angle and 2pi*num_segments_per_rotation
            # an angle is created for each point of the spiral (hence why we include num_points)
            # our start angle is start_angle as we first want to go to directly to the origin first
            angles = np.linspace(start_angle, 2 * np.pi * num_spirals, num_points)
            # radii is simply evenly spaced "divisions" of the coverage_radius going outwards on each of the points in num_points
            radii = np.linspace(0, coverage_radius, num_points)
            # Radii are computed via following polar formula.
            # This is correct because you want the radius to increase by 'distance_between_spirals' every 2pi radians (one rotation)
            # convert to cartesian coordinates
            x_coords = np.cos(angles) * radii
            y_coords = np.sin(angles) * radii
            # we want to return as a 2D matrix where each row is a coordinate pair
            # so we reshape x and y coordinates to be (n, 1) matricies then stack horizontally to get (n, 2) matrix
            vertices = np.hstack((x_coords.reshape(-1, 1), y_coords.reshape(-1, 1)))

        all_points = []

        if insert_extra:
            for i in range(len(vertices) - 1):
                all_points.append(vertices[i])
                vector = vertices[i + 1] - vertices[i]
                magnitude = np.linalg.norm(vector)
                unit_vector = vector / magnitude
                count = 0.0
                while count < magnitude - 3.5:
                    all_points.append(all_points[-1] + (unit_vector * 2.5))  # TODO: figure out how far apart to insert
                    count += 2.5
            return np.array(all_points)

        return vertices

    @classmethod
    def spiral_traj(
        cls,
        center: np.ndarray,
        coverage_radius: float,
        distance_between_spirals: float,
        segments_per_rotation: int,
        tag_id: int,
        insert_extra: bool,
        rover_position: np.ndarray,
        enable_inward: bool,
        inward_begin: float,
    ):
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:                      position to center spiral on (np.ndarray)
        :param coverage_radius:             radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param segments_per_rotation:       number of segments per spiral (int), for example, 4 segments per rotation would be a square spiral, 8 segments per rotation would be an octagonal spiral
        :param tag_id:                      tag id to associate with this trajectory (int)
        :param insert_extra:
        :return:    SearchTrajectory object
        """
        # distance from the center
        distance_from_center = float(np.linalg.norm(rover_position[:2] - center[:2]))

        # vector towards the center from the rover
        direction_from_center = rover_position[:2] - center

        # angle (found with inversr tan) that the rover would have to take to go to the center
        starting_angle = np.arctan2(direction_from_center[1], direction_from_center[0])

        # debugging help
        print("Distance from center:", distance_from_center)
        inward_spiral = False

        # we do an inward spiral if we are more than half the coverage radius away from the center
        if enable_inward:
            # we pass this to the gen_spiral_coordinates function to indicate the necessity of an inward spiral
            inward_spiral = True
            # as we want to do an inward spiral, we need to find the closest point from the rover
            # to the start radius. Multiplies the unit vector with the coverage radius and offsets with the center
            closest_radius_point = (
                center + (direction_from_center / np.linalg.norm(direction_from_center)) * inward_begin
            )

            # finds the vector, in this case, to go to the closest radius point. move_to_center is misleading...help
            # move_to_center = np.linspace(rover_position[:2], closest_radius_point, num=40)

            # so, this is simply the starting angle that the rover would begin its inward spiral at
            # we calculate this using the direction from the closest_radius_point to the center
            starting_angle = np.arctan2(closest_radius_point[1] - center[1], closest_radius_point[0] - center[0])
            zero_centered_spiral_r2 = cls.gen_spiral_coordinates(
                inward_begin,
                distance_between_spirals,
                segments_per_rotation,
                insert_extra,
                inward_spiral,
                inward_begin,
                starting_angle,
            )

        else:
            # all we need to do here is figure out the straight set of points between the rover and the center
            # move_to_center = np.linspace(rover_position[:2], center, num=40)
            zero_centered_spiral_r2 = cls.gen_spiral_coordinates(
                coverage_radius,
                distance_between_spirals,
                segments_per_rotation,
                insert_extra,
                inward_spiral,
                distance_from_center,
                starting_angle,
            )

        # just for debugging
        print("Inward true/false: ", inward_spiral)

        # numpy broadcasting magic to add center to each row of the spiral coordinates
        spiral_coordinates_r2 = zero_centered_spiral_r2 + center

        # add a column of zeros to make it 3D
        spiral_coordinates_r3 = np.hstack(
            (
                spiral_coordinates_r2,
                np.zeros(spiral_coordinates_r2.shape[0]).reshape(-1, 1),
            )
        )
        return SearchTrajectory(
            spiral_coordinates_r3,
            tag_id,
        )
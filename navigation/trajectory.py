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
        max_segment_length: float,  # New parameter for max segment length
    ) -> np.ndarray:
        """
        Generates a set of coordinates for a spiral search pattern centered at the origin.
        If the distance between consecutive points exceeds the max_segment_length, extra points
        are inserted to reduce the segment length.

        :param coverage_radius: radius of the spiral search pattern (float)
        :param distance_between_spirals: distance between each spiral (float)
        :param num_segments_per_rotation: number of segments per spiral (int)
        :param insert_extra: whether to insert extra points between spiral segments
        :param max_segment_length: maximum allowable distance between consecutive points (float)
        :return: np.ndarray of coordinates
        """
        # The number of spirals should ensure coverage of the entire radius.
        num_spirals = np.ceil(coverage_radius / distance_between_spirals).astype("int") + 1
        # The angles are evenly spaced between 0 and 2pi*num_segments_per_rotation
        angles = np.linspace(0, 2 * np.pi * num_spirals, num_segments_per_rotation * num_spirals + 1)

        # Radii are computed via following polar formula.
        radii = angles * (distance_between_spirals / (2 * np.pi))

        # Convert polar to Cartesian coordinates
        x_coords = np.cos(angles) * radii
        y_coords = np.sin(angles) * radii
        vertices = np.hstack((x_coords.reshape(-1, 1), y_coords.reshape(-1, 1)))

        # Function to insert intermediate points if the distance is too large
        def insert_points_if_needed(vertices, max_segment_length):
            all_points = [vertices[0]]
            for i in range(1, len(vertices)):
                p1 = vertices[i - 1]
                p2 = vertices[i]
                # Calculate the distance between consecutive points
                dist = np.linalg.norm(p2 - p1)
                if dist > max_segment_length:
                    # Insert intermediate points along the line between p1 and p2
                    num_insertions = int(np.ceil(dist / max_segment_length)) - 1
                    vector = (p2 - p1) / (num_insertions + 1)
                    for j in range(1, num_insertions + 1):
                        all_points.append(p1 + j * vector)
                all_points.append(p2)
            return np.array(all_points)

        # If insert_extra is True and max_segment_length is set, insert intermediate points
        # if insert_extra and max_segment_length is not None:
        # TODO: fix the parameter declaration for max segment length should be below, temp fix by using 0.5
        # vertices = insert_points_if_needed(vertices, max_segment_length)
        vertices = insert_points_if_needed(vertices, max_segment_length)

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
        max_segment_length: float,  # New parameter for max segment length
    ):
        """
        Generates a spiral search pattern around a center position, assuming rover is at the center position
        :param center:                      position to center spiral on (np.ndarray)
        :param coverage_radius:             radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param segments_per_rotation:       number of segments per spiral (int), for example, 4 segments per rotation would be a square spiral, 8 segments per rotation would be an octagonal spiral
        :param tag_id:                      tag id to associate with this trajectory (int)
        :param insert_extra:                whether to insert extra points
        :param max_segment_length:          maximum length of a segment before extra points are inserted (float)
        :return:    SearchTrajectory object
        """
        zero_centered_spiral_r2 = cls.gen_spiral_coordinates(
            coverage_radius,
            distance_between_spirals,
            segments_per_rotation,
            insert_extra,
            max_segment_length,
        )

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

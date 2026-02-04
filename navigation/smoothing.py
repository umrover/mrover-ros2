from navigation.trajectory import Trajectory
from navigation.context import Context, CostMap
from navigation.coordinate_utils import cartesian_to_ij, ij_to_cartesian, publish_trajectory

from scipy.interpolate import splev, splprep
import numpy as np
import math, sys
from rclpy.node import Node
from mrover.srv import Smoothing



def smoothing(trajectory: Trajectory, context: Context, should_relax: bool, should_interpolate: bool) -> Trajectory:
    """
    Performs relaxation and/or interpolation on given trajectory

    :param trajectory:          Trajectory in cartesian coordinates
    :param should_relax:        Enable relaxation
    :param should_interpolate:  Enable interpolation
    :return:                    Smoothed trajectory
    """
    if should_relax:
        # Perform relaxation
        relaxed_path = Relaxation.relax(
            context, Trajectory(np.apply_along_axis(lambda coord: cartesian_to_ij(context, coord), 1, trajectory.coordinates))
        )
        
        # Convert cost map coordinates to cartesian
        cartesian_coords = np.apply_along_axis(
            lambda coord: ij_to_cartesian(context, coord), 1, relaxed_path.coordinates
        )

        # Add column of zeros to make 3d coordinates
        cartesian_coords = np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))

    else:
        cartesian_coords = trajectory
    
    
    output = Trajectory(cartesian_coords)
    publish_trajectory(output, context, context.relaxed_publisher, [1.0, 0.0, 0.0])

    if should_interpolate:
        # Perform spline interpolation
        spline_path = SplineInterpolation.interpolate(context, output, spacing=2.0)
        plot_spline_path = SplineInterpolation.interpolate(context, output, spacing=0.1)


        publish_trajectory(plot_spline_path, context, context.interpolated_publisher, [0.0, 1.0, 0.0], size=0.1)

        # Check if new interpolated spline is worse than relaxed/original trajectory
        spline_cost = Relaxation.cost_full(context, spline_path)[0]
        old_cost = Relaxation.cost_full(context, output)[0]

        if (spline_cost - old_cost) / (old_cost + 1e-7) > 0.2:
            context.node.get_logger().warning(
                f"Spline cost is much worse than old cost {spline_cost} vs {old_cost}. Using old path instead!"
            )

        else:
            # Use interpolated trajectory only if it isn't terrible
            output = spline_path

    return output


class Relaxation:
    """
    Methods for performing path relaxation and computing the cost of a given path.
    Input trajectories are assumed to be in cost map ij-coordinates
    """

    @staticmethod
    def cost_segment(ctx: Context, trajectory: Trajectory, i: int, j: int) -> float:
        """
        Calculates the cost of the straight-line path between coordinate indices i and j
        in the given trajectory. The method interpolates ceil(length(path)) evenly-spaced
        points in the path segment and returns an approximation of "total cost"
        (cost * path length).

        :param trajectory: Trajectory in ij-coordinates
        :param i:          Start index of segment
        :param j:          End index of segment
        :return:           Cost of segment between indicies i and j of trajectory
        """
        cost_map = ctx.env.cost_map

        # distance between start and end points
        dist = np.sqrt(
            (trajectory.coordinates[i][0] - trajectory.coordinates[j][0]) ** 2
            + (trajectory.coordinates[i][1] - trajectory.coordinates[j][1]) ** 2
        )

        points_per_m = 3
        points_per_cell = points_per_m * cost_map.resolution
        interpolate = math.ceil(points_per_cell * dist)  # total number of interpolated points
        segment_interval = dist / interpolate  # distance between interpolated points
        interpolated = np.column_stack(
            (
                np.linspace(
                    trajectory.coordinates[i][0],
                    trajectory.coordinates[j][0],
                    interpolate,
                ),
                np.linspace(
                    trajectory.coordinates[i][1],
                    trajectory.coordinates[j][1],
                    interpolate,
                ),
            )
        )

        cost = 0.0
        for y, x in interpolated:
            if abs(0.5 - (x - int(x))) < 1e-5 and abs(0.5 - (y - int(y))) < 1e-5:  # Close to grid intersection
                # Get costs from surrounding cost map squares
                cost_arr = [
                    cost_map.data[int(x), int(y)],
                    cost_map.data[int(x) + 1, int(y)],
                    cost_map.data[int(x), int(y) + 1],
                    cost_map.data[int(x) + 1, int(y) + 1],
                ]

                # Only the minimum cost contributes
                cost += segment_interval * (min(cost_arr) + 0.001)
            else:
                # Read cost from corresponding square
                cost += segment_interval * (
                    cost_map.data[int(y + 0.5), int(x + 0.5)] + 0.001
                )  # add small number to make cost nonzero so distance matters
        return cost

    @staticmethod
    def cost_full(ctx: Context, trajectory: Trajectory) -> tuple[float, list[float]]:
        """
        Returns the total cost of the given trajectory, as tuple containing a float 
        and a list of the cost of each individual segment.
        A segment is defined as the cost between two adjacent points in a trajectory.
        """

        segment_costs = []
        total_cost = 0.0

        # Compute and sum for each segment
        for i in range(len(trajectory.coordinates) - 1):
            cost_i = Relaxation.cost_segment(ctx, trajectory, i, i + 1)
            total_cost += cost_i
            segment_costs.append(cost_i)

        return (total_cost, segment_costs)

    @staticmethod
    def relax_single(ctx: Context, trajectory: Trajectory) -> tuple[Trajectory, float]:
        """
        Removes the point in the trajectory whose removal results in maximal cost reduction.
        Returns the new trajectory and the new cost.
        """
        total_cost, segment_costs = Relaxation.cost_full(
            ctx, trajectory
        )

        if len(trajectory.coordinates) <= 2:
            return trajectory, total_cost

        min_idx = -1
        max_diff = -sys.maxsize
        
        # Loops through trajectory nodes, locates the node, that when removed, 
        # decreases the cost the most (distance is factored into cost).
        for i in range(1, len(trajectory.coordinates) - 1):
            cost_i = Relaxation.cost_segment(ctx, trajectory, i - 1, i + 1)
            diff_i = (segment_costs[i - 1] + segment_costs[i]) - cost_i
            if max_diff < diff_i:
                min_idx = i
                max_diff = diff_i

        cost = total_cost - max_diff

        # Returns the trajectory without the node identified in the previous loop
        new_traj = Trajectory(
            np.append(
                trajectory.coordinates[:min_idx],
                trajectory.coordinates[min_idx + 1 :],
                axis=0,
            )
        )
        return (new_traj, cost)

    @staticmethod
    def relax(ctx: Context, trajectory: Trajectory) -> Trajectory:
        """
        Relaxes the given trajectory as much as possible. Calls relax_single until no cost improvement is observed.
        Returns the relaxed trajectory.
        """
        cost, _ = Relaxation.cost_full(ctx, trajectory)

        current_trajectory = trajectory
        candidate_traj, candidate_cost = Relaxation.relax_single(ctx, trajectory)
        
        # Loops until the cost of the relaxed path begins increasing
        # NOTE: adjust rel_tol if the relaxed path is entering high cost areas.
        while candidate_cost <= cost + 0.1 or math.isclose(candidate_cost, cost, rel_tol=0.1):

            manual_new_cost = Relaxation.cost_full(ctx, candidate_traj)
            # ctx.node.get_logger().info(f"Cost: {cost}, New Cost (relax calc): {candidate_cost}, New Cost (manual calc): {manual_new_cost}")

            # If we relax to the point where only the start and end are left, leave the loop
            if len(current_trajectory.coordinates) <= 2:
                break

            current_trajectory = candidate_traj
            cost = candidate_cost

            # Remove a single point 
            candidate_traj, candidate_cost = Relaxation.relax_single(ctx, current_trajectory)

        # Remove first point in trajectory (rover's current position)
        if len(current_trajectory.coordinates) > 1:
            current_trajectory.coordinates = current_trajectory.coordinates[1:]
            
        return current_trajectory


class SplineInterpolation:
    @staticmethod
    def interpolate(ctx: Context, trajectory: Trajectory, spacing: float = 2.0, k:int = 2) -> Trajectory:
        """
        Fits k-degree splines to the given trajectory and returns a new trajectory with
        evenly spaced points sampled from the splines. We approximate the total distance
        of the calculated splines and map the desired distances of each point to the
        default parameterization produce by scipy.
        
        :param spacing:     Distance between trajectory points on the interpolated path (m)
        :param k:           Degree of the splines used to interpolate between points on the input trajectory
        :return:            Interpolated trajectory with points sampled according to given spacing
        """

        # Return if there are not enough trajectory points to perform interpolation
        if len(trajectory.coordinates) < 2:
            return trajectory
        

        # Calculate unit vector pointing in the trajectory's initial direction
        initial = trajectory.coordinates[1] - trajectory.coordinates[0]
        dist = np.hypot(initial[0], initial[1])
        initial /= dist

        # Inserts point very close after the initial point in the direction of the initial unit vector,
        # forcing spline to follow the initial heading
        trajectory.coordinates = np.concatenate([
            trajectory.coordinates[0][np.newaxis, :],
            (trajectory.coordinates[0] + initial * min(dist, 0.5))[np.newaxis, :],
            trajectory.coordinates[1:]
        ], axis=0)
        
        # Attempts to generate the spline
        try:
            spline, u = splprep([trajectory.coordinates[:, 0], trajectory.coordinates[:, 1]], s=0, k=2)
        except Exception as e:
            print(e)
            return trajectory


        # Generates a parameterized spline and samples finely along the curve,
        # Selects evenly-spaced points along the spline based on the specified spacing

        # Make really smooth for accurate distance
        u_fine = np.linspace(u.min(), u.max(), 100)  # NOTE: number of points can be tuned
        x_fine, y_fine = splev(u_fine, spline)

        # Calculate distance from start for each point
        dist = np.cumsum(np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2))
        dist = np.insert(dist, 0, 0)

        samples = int(dist[-1] / spacing)
        target_dist = np.linspace(0, samples * spacing, samples + 1)

        u_spaced = np.interp(target_dist, dist, u_fine)             # map distance to u parameter
        u_spaced = np.append(u_spaced, u[-1])   # make sure end waypoint is preserved

        x_spaced, y_spaced = splev(u_spaced, spline)

        return Trajectory(np.column_stack([x_spaced, y_spaced, np.zeros(x_spaced.shape[0])]))

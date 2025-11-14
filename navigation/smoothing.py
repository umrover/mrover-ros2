from navigation.trajectory import Trajectory
from navigation.context import Context, CostMap 

from scipy.interpolate import splev, splprep
import numpy as np
import math, sys


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
            if abs(0.5 - (x - int(x))) < 1e-5 and abs(0.5 - (y - int(y))) < 1e-5:     # Close to boundary
                cost_arr = [
                    cost_map.data[int(x), int(y)],
                    cost_map.data[int(x) + 1, int(y)],
                    cost_map.data[int(x), int(y) + 1],
                    cost_map.data[int(x) + 1, int(y) + 1]
                    ]
                cost += segment_interval * (min(cost_arr) + 0.001)
            else:
                cost += (
                    segment_interval
                    * (cost_map.data[int(y + 0.5), int(x + 0.5)] + 0.001) # make cost nonzero so distance matters
                )
        return cost

    @staticmethod
    def cost_full(ctx: Context, trajectory: Trajectory) -> tuple[float, list[float]]:
        """
        Returns the total cost of the given trajectory, both as a float and a list of the cost of each individual segment.
        A segment is defined as the cost between two adjacent points in a trajectory.
        """
        segment_costs = []
        total_cost = 0.0

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
        )  # TODO could optimize by putting this in relax() and passing it over

        if len(trajectory.coordinates) <= 2:
            return trajectory, total_cost

        min_idx = -1
        min_cost = -1
        max_diff = -sys.maxsize

        for i in range(1, len(trajectory.coordinates) - 1):
            cost_i = Relaxation.cost_segment(ctx, trajectory, i - 1, i + 1)
            diff_i = (segment_costs[i - 1] + segment_costs[i]) - cost_i
            if max_diff < diff_i:
                min_cost = cost_i
                min_idx = i
                max_diff = diff_i

        cost = total_cost - max_diff

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
        candidate_traj, candidate_cost = Relaxation.relax_single(
            ctx, trajectory
        )

        # TODO: depending on how inefficient this is, we may limit how many times this loop runs
        while candidate_cost < cost or math.isclose(candidate_cost, cost):
            if len(current_trajectory.coordinates) <= 2:
                break

            current_trajectory = candidate_traj
            cost = candidate_cost

            candidate_traj, candidate_cost = Relaxation.relax_single(
                ctx, current_trajectory
            )

        return current_trajectory


class SplineInterpolation:
    @staticmethod
    def interpolate(
        ctx: Context, trajectory: Trajectory, spacing: float = 0.25
    ) -> Trajectory:
        """
        Fits cubic splines to the given trajectory and returns a new trajectory with 
        evenly spaced points sampled from the splines. We approximate the total distance
        of the calculated splines and map the desired distances of each point to the 
        default parameterization produce by scipy.
        """
        # can configure smoothness (s) and other parameters
        try:
            spline, u = splprep(
                [trajectory.coordinates[:, 0], trajectory.coordinates[:, 1]], s=0,k=2
            )
        except Exception as e:
            print(e)
            return trajectory

        # make really smooth for accurate distance
        u_fine = np.linspace(u.min(), u.max(), 100)  # TODO find good number
        x_fine, y_fine = splev(u_fine, spline)

        # calculate distance from start for each point
        dist = np.cumsum(np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2))
        dist = np.insert(dist, 0, 0)

        samples = int(dist[-1] / spacing)
        target_dist = np.linspace(0, samples * spacing, samples + 1)

        u_spaced = np.interp(target_dist, dist, u_fine)  # map distance to u parameter
        u_spaced = np.append(u_spaced, u[-1])  # make sure end waypoint is preserved


        x_spaced, y_spaced = splev(u_spaced, spline)

        return Trajectory(np.column_stack([x_spaced, y_spaced, np.zeros(x_spaced.shape[0])]))

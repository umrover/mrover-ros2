from math import radians, degrees, sin, cos, acos
from random import uniform
from numpy import random


def spherical_to_cartesian(rho, theta, phi):
    x = rho * sin(phi) * cos(theta)
    y = rho * sin(phi) * sin(theta)
    z = rho * cos(phi)

    return z, y, x


def get_random_arc_angle(theta1, theta2, phi1, phi2,
                         theta_sampling="uniform", phi_sampling="uniform",
                         theta_std=0.3, phi_std=0.3):

    theta1 = radians(theta1)
    theta2 = radians(theta2)
    phi1 = radians(phi1)
    phi2 = radians(phi2)

    if theta_sampling == "uniform":
        theta = uniform(theta1, theta2)

    elif theta_sampling == "gaussian":
        theta = random.normal((theta1 + theta2) / 2, theta_std)
        # theta = max(theta1, min(theta2, theta))

    else:
        raise ValueError("Unknown Theta Sampling")

    # x1 = (1 - cos(phi1)) / 2
    # x2 = (1 - cos(phi2)) / 2
    # x = uniform(x1, x2)
    # phi = arccos(1 - 2 * x)

    # The above code simplifies to

    x1 = cos(phi1)
    x2 = cos(phi2)

    if phi_sampling == "uniform":
        x = uniform(x1, x2)

    elif phi_sampling == "gaussian":
        x = random.normal((x1 + x2) / 2, phi_std)
        # x = max(-1, min(1, x))
        x = max(x2, min(x1, x))

    else:
        raise ValueError("Unknown Phi Sampling")

    phi = acos(x)

    return theta, phi


def random_spherical_angle():
    return get_random_arc_angle(0, 360, 0, 180)


def angle_between(v1, v2):
    dot = v1.dot(v2)
    dot /= v1.length * v2.length
    return degrees(acos(dot))

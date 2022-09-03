import numpy as np


def line_sphere_intersection(p1, p2, c, r):
    """
    Implements the line-sphere intersection algorithm.
    https://en.wikipedia.org/wiki/Line-sphere_intersection

    :param p1: start of line segment
    :param p2: end of line segment
    :param c: sphere center
    :param r: sphere radius
    :returns: discriminant (value under the square root) of the line-sphere
        intersection formula, as a np.float64 scalar
    """
    # FILL in your code here

    u_vec = p1 - p2
    mag = np.linalg.norm(u_vec)
    u = u_vec/mag

    # print(u)

    # print(p1-c)


    term1 = np.dot(u,p1-c)**2
    # print(term1)

    term2 = np.linalg.norm(p1-c)**2 - r**2
    # print(term2)

    return term1 - term2

"""

print("---------------------------------------")
p1 = np.array([2,0,0])
p2 = np.array([2,2,0])
c  = np.array([0,0,0])
r  = 4
print("P1:",p1)
print("P2:",p2)
print("c:",c)
print("r:",r)
print("Discriminant = ",line_sphere_intersection(p1,p2,c,r))
print("---------------------------------------")
p1 = np.array([4,0,0])
p2 = np.array([4,2,0])
c  = np.array([0,0,0])
r  = 4
print("P1:",p1)
print("P2:",p2)
print("c:",c)
print("r:",r)
print("Discriminant = ",line_sphere_intersection(p1,p2,c,r))
print("---------------------------------------")
p1 = np.array([5,0,0])
p2 = np.array([5,2,0])
c  = np.array([0,0,0])
r  = 4
print("P1:",p1)
print("P2:",p2)
print("c:",c)
print("r:",r)
print("Discriminant = ",line_sphere_intersection(p1,p2,c,r))
print("---------------------------------------")

"""
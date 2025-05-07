import numpy as np

m = 1
g = 10
e_z = np.array([0,0,1])

NUM_N = 10
NUM_POINTS = 15

low_bound = -10
high_bound = 10

def main():

    ns = gen_candidate_normals(NUM_N)

    for i in range(NUM_N):
        psuedo_inverse_test(ns[i], NUM_POINTS)

def gen_candidate_normals(num_n):

    ns = []

    for i in range(num_n): 
        append = False
        while not append:
            cand = np.random.uniform(-1,1, (3))
            if np.dot(cand, e_z) > 0:
                append = True
                ns.append(cand)

    return ns

def psuedo_inverse_test(n, num_points):

    n = n /np.linalg.norm(n)

    e_z = np.array([0,0,1])

    proj = np.outer(n, n.T)

    F =  proj @ (-1 * m * g *e_z)
    Q = -F
    Q = cross_product_operator(Q)
    A = np.array([[1,0],[0,1],[-n[0]/n[2], -n[1]/n[2]]])

    Q = Q @ A

    points = np.random.uniform(low_bound, high_bound, (2,num_points))
    M = Q @ points

    new_points = np.linalg.pinv(Q) @ M

    if np.linalg.norm(points - new_points) < 0.01:
        print("test passed")
    else:
        print("test failed")

def cross_product_operator(v):
    x,y,z = v

    return np.array([[0,-z, y],
                     [z , 0, -x],
                     [-y, x, 0]])

if __name__ == "__main__":
    main()
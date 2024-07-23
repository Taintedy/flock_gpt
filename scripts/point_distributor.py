import numpy as np
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from sdf import box, sphere, write_binary_stl, rounded_box, capsule
from sdf.mesh import _estimate_bounds
import scipy
from scipy.spatial import ConvexHull, QhullError
from sklearn.cluster import KMeans


class PointDistributer():

    def __init__(self, cost) -> None:
        self.cost = cost
        self.bounds_min, self.bounds_max = _estimate_bounds(self.cost)


    def get_cost_sdf(self, x):
        points = x.reshape(int(x.shape[0]/3), 3)
        sdf_cost = np.sum(np.abs(self.cost(points)))
        cost = sdf_cost
        return cost
    
    def numerical_gradient(self, x, f, epsilon=1e-6):
        """
        Numerically computes the gradient of a function at a given point using the finite difference method.

        Parameters:
            func (function): The function for which the gradient needs to be calculated.
            x (float or numpy.array): The point at which to calculate the gradient.
            epsilon (float): The step size used in the finite difference method.

        Returns:
            gradient (float or numpy.array): The numerical gradient at the given point.
        """
        x = np.asarray(x).flatten()
        num_dimensions = len(x)
        epsilons = np.identity(num_dimensions) * epsilon
        delta = x + epsilons
        fx_plus_eps = np.array([f(d) for d in delta])
        fx = f(x)
        print(fx)
        gradient = (fx_plus_eps - fx) / epsilon
        return gradient 

    def distrib_cost(self, x):
        x = x.reshape(int(x.shape[0]/3), 3)
        dist_cost = 0
        volume = 0
        if x.shape[0] > 1:
            dists = cdist(x, x)
            min_dists = []
            for i, dist in enumerate(dists):
                min_dists.append(np.min(dist[dist>0]))

            min_dists = np.array(min_dists)
            dist_cost = np.sum(min_dists)
            try:
                volume = ConvexHull(x).volume
            except QhullError:
                volume = 0

        sdf_cost = np.sum(np.abs(self.cost(x)))
        
        cost =  100 * sdf_cost - 0.5 * dist_cost - 0.5 * volume
        return cost

    def generate_points(self, num_of_points):
        points = np.random.uniform(low=self.bounds_min, high=self.bounds_max, size=(1000,3))
        
        
        init_shape = points.shape
        points = points.flatten()
        res = scipy.optimize.minimize(self.get_cost_sdf, points, method="L-BFGS-B", options={"maxiter":10000})
        points = res.x.reshape(int(init_shape[0]), 3)


        kmeans = KMeans(n_clusters=num_of_points, random_state=0, n_init="auto").fit(points)
        out_points = kmeans.cluster_centers_
        
        
        out_points = out_points.flatten()
        # for i in range(100):
        #     print(i)
        #     out_points -= 0.01 * self.numerical_gradient(out_points, self.distrib_cost)
        res = scipy.optimize.minimize(self.distrib_cost, out_points, method="L-BFGS-B", options={"maxiter":100})
        out_points = res.x.reshape(int(out_points.shape[0]/3), 3)
        
        
        
        return out_points


# # Example usage
# cost = box(1).translate((0, 0, 2))
# # cost = rounded_box((1, 2, 3), 0.25)

# point_distr = PointDistributer(cost)
# num_points = 8
# distributed_points = point_distr.generate_points(num_points)

# print("Distributed Points:")
# print(distributed_points)

# print(distributed_points)
# fig = plt.figure(figsize=(16, 16))
# ax = fig.add_subplot(111, projection='3d')
# for point in distributed_points:
#     ax.scatter(point[0], point[1], point[2])
# ax.set_xlim([-2.5, 2.5])
# ax.set_ylim([-2.5, 2.5])
# ax.set_zlim([-2.5, 2.5])
# plt.show()
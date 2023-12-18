import numpy as np

class RANSAC:

    @staticmethod
    def run(xy, num_iterations, epsilon):
        """
        Performs the random consensus sample algorithm.
        2 hypothetical inliers are chosen in every 
        iteration of the algorithm and a line is fit
        between them. The number samples in
        "consensus" with the chosen line is computed
        as the number of samples whose distance to
        the line is less than epsilon.
        After num_iterations, the set of inliers with
        the most consensus is returned.
        Args:
            xy: a numpy array of tuples.
            num_iterations: the number of iterations
            of RANSAC to perform.
            epsilon: the threshold distance from
            the chosen line.
        Returns:
            a, b, c: values describing the chosen
            line ax + by + c = 0.
        """

        # Initialize a worst fit
        best_num_fit = 0
        best_abc = np.zeros(3)

        # In each iteration
        for i in range(num_iterations):
            # Choose a line and find how
            # many points it fits.
            abc, num_fit = RANSAC.perform_iteration(xy, epsilon)

            # If it is better than the
            # current model, update.
            if num_fit > best_num_fit:
                best_num_fit = num_fit
                best_abc = abc

        return best_abc

    @staticmethod
    def perform_iteration(xy, epsilon):
        # Choose 2 random points.
        rand = np.random.randint(xy.shape[0], size=2)
        if rand[0] == rand[1]: return (0, 0, 0), 0
        p = xy[rand]

        # Determine the line ax + by + c = 0
        # defined by the two random points.
        a = (p[1][1] - p[0][1])
        b = -(p[1][0] - p[0][0])
        c = p[1][0] * p[0][1] - p[1][1] * p[0][0]

        # Compute the distance from each point in xy
        # to the line.
        # Source Wikipedia: "Distance from a Point to a Line"
        dist_p0_p1 = np.linalg.norm(p[1] - p[0])
        dist = np.abs(a * xy[:,0] + b * xy[:,1] + c)/dist_p0_p1

        # Determine how many points are
        # epsilon away from the line.
        num_fit = np.count_nonzero(dist < epsilon)

        return (a, b, c), num_fit
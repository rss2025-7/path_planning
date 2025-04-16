def find_point_along_trajectory(r, la, p1, p2):
        """
        r: robot pose (x,y)
        la: lookahead distance (float)
        p1: start point of trajectory segment (x,y)
        p2: end point of trajectory segment (x,y)
        """
        p1 = np.array(p1)
        p2 = np.array(p2)

        V = p2 - p1

        a = V.dot(V)
        b = 2 * V.dot(p1 - r)
        c = p1.dot(p1) + r.dot(r) - 2 * p1.dot(r) - la * la

        disc = b**2 - 4 * a * c
        if disc < 0:
            return None

        sqrt_disc = np.sqrt(disc)
        # solutions
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / 2 * a

        # If neither of these is between 0 and 1, then the line segment misses the circle, or hits if extended
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None

        t = max(0, min(1, - b / (2 * a)))
        return p1 + t * V

import numpy as np

class StewartPlatform:
    def __init__(self, base_r, plat_r, offset_deg = 7):
        """
        Initialize the geometry of a 6-6 stewart platform
        offset_deg: minor offset angle in degrees (small angle between two points)
            set offset_deg to 0 for a 3-3 configuration
        """
        
        # initialize radius of platform and pose
        self.base_r = base_r
        self.plat_r = plat_r

        self.X_base = np.zeros(6)
        self.X_plat = np.zeros(6)
        
        self.X_base[4] = 90
        self.X_plat[0] = 1
        self.X_plat[4] = 90

        phi = np.deg2rad(offset_deg) / 2

        # define point positions
        base_angles = np.array([0 - phi, 0 + phi, np.deg2rad(120) - phi, 
            np.deg2rad(120) + phi, np.deg2rad(240) - phi, np.deg2rad(240) + phi])
        plat_angles = base_angles + np.deg2rad(60) # offset plat angles by 60 deg

        self.local_base_points = np.zeros((3,6))
        self.local_base_points[0, :] = base_r * np.cos(base_angles)
        self.local_base_points[1, :] = base_r * np.sin(base_angles)

        self.local_plat_points = np.zeros((3,6))
        self.local_plat_points[0, :] = plat_r * np.cos(plat_angles)
        self.local_plat_points[1, :] = plat_r * np.sin(plat_angles)

    def set_Pose(self, X_base = None, X_plat = None):
        if X_base is not None:
            X_base = np.array(X_base)
            self.X_base = X_base
        if X_plat is not None:
            X_plat = np.array(X_plat)
            self.X_plat = X_plat

    




def main():
    sp = StewartPlatform(5, 5)

    # sp.set_Pose(X_plat = [5, 5, 5, 5, 5, 5])
    print(sp.X_base, sp.X_plat)


if __name__ == '__main__':
    main()
from .header import *


class Quaternion:
    @staticmethod
    def angvel_to_deriv(q: f64array, omega: f64array) -> f64array:
        """
        Compute the derivative of the quaternion given angular velocity.

        Parameters:
        - q: Current quaternion (4x1 array)
        - omega: Angular velocity in body frame (3x1 array)

        Returns:
        - dq: Derivative of the quaternion (4x1 array)
        """
        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
        dq = 0.5 * Quaternion.mul(q, omega_quat)
        return dq
    @staticmethod
    def mul(q1: f64array, q2: f64array) -> f64array:
        """
        Perform quaternion multiplication.

        Parameters:
        - q1: First quaternion (4x1 array)
        - q2: Second quaternion (4x1 array)

        Returns:
        - q: Resulting quaternion (4x1 array)
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])
    
    @staticmethod
    def conj(q: f64array) -> f64array:
        """
        Compute the conjugate of a quaternion.

        Parameters:
        - q: Quaternion (4x1 array)

        Returns:
        - q_conjugate: Conjugate of the quaternion (4x1 array)
        """
        return np.array([q[0], -q[1], -q[2], -q[3]])
        

    @staticmethod
    def rot(q: f64array, v: f64array) -> f64array:
        """
        Rotate a vector using a quaternion.

        Parameters:
        - q: Quaternion (4x1 array)
        - v: Vector to be rotated (3x1 array)

        Returns:
        - v_rotated: Rotated vector (3x1 array)
        """
        q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0.0, v[0], v[1], v[2]])
        v_rotated = Quaternion.mul(Quaternion.mul(q, v_quat), q_conjugate)
        return v_rotated[1:]  # Return only the vector part 
    
    @staticmethod
    def rot_inv(q: f64array, v: f64array) -> f64array:
        """
        Rotate a vector using the inverse of a quaternion.

        Parameters:
        - q: Quaternion (4x1 array)
        - v: Vector to be rotated (3x1 array)

        Returns:
        - v_rotated: Rotated vector (3x1 array)
        """
        q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0.0, v[0], v[1], v[2]])
        v_rotated = Quaternion.mul(Quaternion.mul(q_conjugate, v_quat), q)
        return v_rotated[1:]
    
    @staticmethod
    def lmul_mat(q: f64array, mat: f64array) -> f64array:
        """
        multiply a 4x4 quaternion matrix using a quaternion on the left.
        This will treat the matrixa as a set of four quaternions (in columns), and apply the quaternion to each column.
        A 3x3 matrix may be passed, in which case it will be coerced to a 4x4 matrix.
        Parameters:
        - q: Quaternion (4x1 array)
        - mat: 3x3 matrix to be rotated

        Returns:
        - mat_rotated: Rotated 3x3 matrix
        """
        if mat.shape[0] == 3:
            # Coerce to 4x4 by adding a row of zeros and a column of zeros
            mat = np.block([[0, np.zeros((1, 3))],
                            [np.zeros((3, 1)), mat]])

        v1 = Quaternion.mul(q, mat[:, 0])
        v2 = Quaternion.mul(q, mat[:, 1])
        v3 = Quaternion.mul(q, mat[:, 2])
        v4 = Quaternion.mul(q, mat[:, 3])
        return np.column_stack((v1, v2, v3, v4))
    
    @staticmethod
    def rmul_mat(mat: f64array, q: f64array) -> f64array:
        """
        multiply a 4x4 quaternion matrix using a quaternion on the right.
        This will treat the matrixa as a set of four quaternions (in columns), and apply the quaternion to each column.
        A 3x3 matrix may be passed, in which case it will be coerced to a 4x4 matrix.
        Parameters:
        - q: Quaternion (4x1 array)
        - mat: 3x3 matrix to be rotated

        Returns:
        - mat_rotated: Rotated 3x3 matrix
        """
        if mat.shape[0] == 3:
            # Coerce to 4x4 by adding a row of zeros and a column of zeros
            mat = np.block([[0, np.zeros((1, 3))],
                            [np.zeros((3, 1)), mat]])

        v1 = Quaternion.mul(mat[:, 0], q)
        v2 = Quaternion.mul(mat[:, 1], q)
        v3 = Quaternion.mul(mat[:, 2], q)
        v4 = Quaternion.mul(mat[:, 3], q)
        return np.column_stack((v1, v2, v3, v4))
    
    @staticmethod
    def lmul_mat_inv(q: f64array, mat: f64array) -> f64array:
        """
        multiply a 4x4 quaternion matrix using the inverse of a quaternion on the left.
        This will treat the matrixa as a set of four quaternions (in columns), and apply the quaternion to each column.
        A 3x3 matrix may be passed, in which case it will be coerced to a 4x4 matrix.
        Parameters:
        - q: Quaternion (4x1 array)
        - mat: 3x3 matrix to be rotated

        Returns:
        - mat_rotated: Rotated 3x3 matrix
        """
        return Quaternion.lmul_mat(Quaternion.conj(q), mat)
    @staticmethod
    def rmul_mat_inv(mat: f64array, q: f64array) -> f64array:
        """
        multiply a 4x4 quaternion matrix using the inverse of a quaternion on the right.
        This will treat the matrixa as a set of four quaternions (in columns), and apply the quaternion to each column.
        A 3x3 matrix may be passed, in which case it will be coerced to a 4x4 matrix.
        Parameters:
        - q: Quaternion (4x1 array)
        - mat: 3x3 matrix to be rotated

        Returns:
        - mat_rotated: Rotated 3x3 matrix
        """
        return Quaternion.rmul_mat(Quaternion.conj(q), mat)
    
    @staticmethod
    def rot_billinear_form(q: f64array, mat: f64array) -> f64array:
        """
        Rotate a 3x3 billinear form using a quaternion.
        
        Parameters:
        - q: Quaternion (4x1 array)

        Returns:
        - R: Rotation matrix (3x3 array)
        """
        rot1 = np.column_stack([
            Quaternion.rot(q, mat[:, 0]),
            Quaternion.rot(q, mat[:, 1]),
            Quaternion.rot(q, mat[:, 2])
        ])
        return np.stack([
            Quaternion.rot(q, rot1[0, :]),
            Quaternion.rot(q, rot1[1, :]),
            Quaternion.rot(q, rot1[2, :])
        ])
    

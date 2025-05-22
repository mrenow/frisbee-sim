import numpy as np

class Quaternion:
  def angvel_to_deriv(q, omega):
      """
      Compute the derivative of the quaternion given angular velocity.

      Parameters:
      - q: Current quaternion (4x1 array)
      - omega: Angular velocity in body frame (3x1 array)

      Returns:
      - dq: Derivative of the quaternion (4x1 array)
      """
      omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
      dq = 0.5 * Quaternion.multiply(q, omega_quat)
      return dq

  def multiply(q1, q2):
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
  
  def conjugate(q):
      """
      Compute the conjugate of a quaternion.

      Parameters:
      - q: Quaternion (4x1 array)

      Returns:
      - q_conjugate: Conjugate of the quaternion (4x1 array)
      """
      return np.array([q[0], -q[1], -q[2], -q[3]])
      

  def rotate(q, v):
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
      v_rotated = Quaternion.multiply(Quaternion.multiply(q, v_quat), q_conjugate)
      return v_rotated[1:]  # Return only the vector part 
from math import sin, cos, pi, atan2, asin, sqrt
from geometry_msgs.msg import Pose


def pi2pi(angle):
    return (angle + pi) % (2 * pi) - pi

def quaternion_from_euler(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x,y,z,w]

def yaw_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
     
    return yaw_z

def mypose2rosPose(pose):
    res = Pose()
    res.position.x = pose[0]
    res.position.y = pose[1]
    q = quaternion_from_euler(0, 0, pose[2])
    res.orientation.x = q[0]
    res.orientation.y = q[1]
    res.orientation.z = q[2]
    res.orientation.w = q[3]
    return res

def rosPose2mypose(Pose):
    q = Pose.orientation
    yaw = pi2pi(yaw_from_quaternion([q.x, q.y, q.z, q.w]))
    return [Pose.position.x, Pose.position.y, yaw]    

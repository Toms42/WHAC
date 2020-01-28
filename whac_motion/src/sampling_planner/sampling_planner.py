import rospy
from std_msgs.msg import String, Bool, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PointStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import PointCloud2
from nav_msgs.srv import GetPlan
from collections import namedtuple
import numpy as np
from math import sin, cos
import tf2_ros as tf2
import sensor_msgs.point_cloud2 as pc2
import tf2_sensor_msgs.tf2_sensor_msgs as tf2sm
import tf
from whac_motion.srv import GotoGoal, GotoGoalResponse


class SamplingPlanner:
    def __init__(self):
        Params = namedtuple('Params',
                            " ".join([
                                'bubbleRadius',  # m
                                'wheelbaseWidth',  # m
                                'maxSpeed',  # m/s
                                'maxAngle',  # rad
                                'horizonTime',  # s
                                'numTrajectories',
                                'numPts',
                                'frequency',
                                'carrotDist',
                                'maxWaitBeforeReplan']))
        self.params = Params(
            bubbleRadius=rospy.get_param("~bubbleRadius"),
            wheelbaseWidth=rospy.get_param("~wheelbaseWidth"),
            maxSpeed=rospy.get_param('~maxSpeed'),
            maxAngle=rospy.get_param("~maxAngle"),
            horizonTime=rospy.get_param("~horizonTime"),
            numTrajectories=rospy.get_param("~numTrajectories"),
            numPts=rospy.get_param("~numPts"),
            carrotDist=rospy.get_param("~carrotDist"),
            maxWaitBeforeReplan=rospy.get_param("~maxWaitBeforeReplan"),
            frequency=rospy.get_param("~frequency")
        )

        self.cloudTopic = rospy.get_param('~cloud_topic', 'cloud')

        self.tfb = tf2.Buffer()
        self.tfl = tf2.TransformListener(self.tfb)
        self.tfr = tf.TransformerROS()

        self.followPath = False

        self.x = [0, 0, 0]
        self.obstacles = [[1., 0.]]
        self.globalPath = Path()
        self.carrotIdx = 0
        self.cx = None

        self.odom_frame = rospy.get_param('odom_frame', 'odom')
        self.base_frame = rospy.get_param('base_frame', 'base_link')

        if self.params.numTrajectories % 2 == 0:
            rospy.logerr("Error! Number of trajectories must be odd!")

        self.trajPubs = []
        for i in range(self.params.numTrajectories):
            self.trajPubs.append(rospy.Publisher("traj_candidates/traj_{}".format(i), Path, queue_size=10))

        self.localPlanPub = rospy.Publisher("~local_plan", Path, queue_size=10)
        self.globalPlanPub = rospy.Publisher("~global_plan", Path, queue_size=10)
        self.cmdVelPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.carrotPub = rospy.Publisher("~carrot_pos", PointStamped, queue_size=10)

        self.obstacleSub = rospy.Subscriber(self.cloudTopic, PointCloud2, self.updateObstacles)

        rospy.wait_for_service(rospy.get_param('planner_service', '/astar_planner/get_plan'), )
        self.globalPlannerSrv = rospy.ServiceProxy(rospy.get_param('planner_service', '/astar_planner/get_plan'), GetPlan)

        map(float, self.x)

    def computeTrajectories(self):
        trajectories = []
        inputs = []

        for i in range(-self.params.numTrajectories / 2 + 1, self.params.numTrajectories / 2 + 1):
            angVel = float(i) * self.params.maxAngle
            xVel = self.calcXVal(angVel)
            dt = 1.0 / self.params.numPts * self.params.horizonTime

            x = [self.x]  # x,y,th
            u = [xVel, angVel]

            for n in range(self.params.numPts):
                x.append(self.motion(x[-1], u, dt))

            trajectories.append(x)
            inputs.append(u)

        return trajectories, inputs

    def removeCollidingPaths(self, trajectories, inputs):
        newTrajectories = []
        newInputs = []

        obstacles = self.obstacles
        threshold = self.params.bubbleRadius ** 2

        # For each trajectory, check if any obstacles are within the robotradius
        for i in range(len(inputs)):
            traj = trajectories[i]
            input = inputs[i]
            cull = False
            for pt in traj[self.params.numPts/3:self.params.numPts]:
                dists = [(o[0] - pt[0]) ** 2 + (o[1] - pt[1]) ** 2 for o in obstacles]
                if min(dists) < threshold:
                    cull = True
                    break
            if not cull:
                newTrajectories.append(traj)
                newInputs.append(input)

        return newTrajectories, newInputs

    def calcXVal(self, angVel, forward=True):
        # Calculate the forward speed based on constant wheelspeed limit and angular velocity
        wheelSpeed = float(self.params.wheelbaseWidth) / 2 * angVel
        xSpeed = max(self.params.maxSpeed - abs(wheelSpeed), 0)
        return xSpeed if forward else -xSpeed

    def motion(self, x, u, dt):
        res = [0, 0, 0]
        res[0] = x[0] + u[0] * cos(x[2]) * dt
        res[1] = x[1] + u[0] * sin(x[2]) * dt
        res[2] = x[2] + u[1] * dt
        return res

    def moveCarrot(self):
        xp = PointStamped()
        xp.header.frame_id = self.odom_frame
        xp.point= Point(self.x[0], self.x[1], 0)
        try:
            xgp = self.tfr.transformPoint(xp, "map")
        except:
            rospy.logwarn("Error transforming point!")
            return

        xg = [xgp.point.x, xgp.point.y]

        waypoints = self.path2traj(self.globalPath)

        if self.cx is None:
            self.cx = waypoints[0]

        while (xg[0] - self.cx[0]) ** 2 + \
                (xg[1] - self.cx[1]) ** 2 < self.params.carrotDist ** 2:

            if self.carrotIdx >= len(waypoints) - 1:
                break

            pt0 = np.array(waypoints[self.carrotIdx])
            pt1 = np.array(waypoints[self.carrotIdx + 1])
            cpt = np.array(self.cx)
            v = pt1 - pt0
            v /= np.linalg.norm(v)
            v *= 0.01  # Advance carrot 5cm along path
            cpt += v
            self.cx = list(cpt)

            if np.linalg.norm(pt1 - cpt) < 0.015:
                self.cx = list(pt1)
                self.carrotIdx += 1

    def getRewards(self, trajectories):
        rewards = []

        cxp = PointStamped()
        cxp.header.frame_id = self.odom_frame
        cxp.point = Point(self.x[0], self.x[1], 0)
        try:
            cxop = self.tfr.transformPoint(cxp, "odom")
        except:
            rospy.logwarn("Error transforming point!")
            return []

        cxo = [cxop.point.x, cxop.point.y]

        for traj in trajectories:
            pt = np.array(traj[-1][0:2])
            cpt = np.array(cxo)
            d2 = np.linalg.norm(pt - cpt)
            rewards.append(-d2)
        return rewards

    def update(self, timerEvent):
        if not self.followPath:
            return

        # Update Robot State:
        try:
            trans = self.tfb.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0))
        except:
            rospy.logwarn("Error looking up transform!")
            return

        q = trans.transform.rotation
        rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        pos = trans.transform.translation
        self.x[0] = pos.x
        self.x[1] = pos.y
        self.x[2] = rpy[2]

        # Compute trajectories and cull colliding paths:
        (trajectories, inputs) = self.computeTrajectories()
        self.publishTrajectories(trajectories)
        (trajectories, inputs) = self.removeCollidingPaths(trajectories, inputs)

        # If no solutions, sit still
        if len(trajectories) == 0:
            rospy.loginfo("No valid local trajectories found! Waiting!")
            self.cmdVelPub.publish(input2twist([0, 0]))
            self.localPlanPub.publish(traj2path([self.x]))
            return

        # Move the carrot (sliding goal) and calculate rewards
        self.moveCarrot()

        if self.cx is None:
            rospy.loginfo("Carrot does not yet exist! Is there a global plan?")
            return

        rewards = self.getRewards(trajectories)

        lowestDist = max(rewards)
        best = argmax(rewards)

        if len(rewards) > 0 and abs(lowestDist) < self.satisfaction_threshold**2 \
            and self.carrotIdx >= len(self.globalPath.poses):
            self.followPath = False
            self.cx = None
            self.globalPath = Path()
            rospy.loginfo("Satisfaction Achieved!")

        # Publish results
        self.localPlanPub.publish(traj2path(trajectories[best]))
        self.globalPlanPub.publish(self.globalPath)
        self.carrotPub.publish(pt2point(self.cx))
        self.cmdVelPub.publish(input2twist(inputs[best]))

    def publishTrajectories(self, trajectories):
        for i in range(len(trajectories)):
            self.trajPubs[i].publish(traj2path(trajectories[i]))

    def updateObstacles(self, cloud):
        # Transform to odom frame:
        try:
            trans = self.tfb.lookup_transform(self.odom_frame, cloud.header.frame_id, rospy.Time(0))
        except:
            rospy.logwarn("Error looking up transform!")
            return
        cloud = tf2sm.do_transform_cloud(cloud, trans)

        # Extract obstacle locations:
        self.obstacles = pc2.read_points_list(cloud, field_names=["x", "y"], skip_nans=True)

    def path2traj(self, path):
        pts = []
        for ps in path.poses:
            try:
                ps = self.tfr.transformPose(ps, "map")
            except:
                rospy.logwarn("Error looking up transform!")
            pts.append([ps.point.x, ps.point.y])
        return pts

    def getGlobalTraj(self, pos):
        try:
            pos = self.tfr.transformPose(pos, 'map')
        except:
            rospy.logwarn("Error looking up transform!")
            return False


        try:
            trans = self.tfb.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0))
        except:
            rospy.logwarn("Error looking up transform!")
            return

        req = GetPlan()
        h = Header()
        h.stamp = rospy.Time.now()
        req.start = PoseStamped()
        req.end = PoseStamped()
        req.start.header = h
        req.start.pose.position = Point(trans.x, trans.y, 0)
        req.end = pos
        req.end.header = h
        res = self.globalPlannerSrv()
        if len(res.plan.poses) == 0:
            rospy.logwarn("No valid global trajectory was found!")
            return None
        else:
            return res.plan

    def startPathing(self, gotoGoal):
        globalPath = self.getGlobalTraj(gotoGoal.goal)

        if globalPath is None:
            self.followPath = False
            self.cx = None
            self.globalPath = Path()
            return GotoGoalResponse(False)

        self.followPath = True
        self.globalPath = globalPath
        return GotoGoalResponse(True)


    def start(self):
        rospy.Timer(rospy.Duration(1.0 / self.params.frequency), self.update)
        rospy.spin()


def argmax(iterable):
    return max(enumerate(iterable), key=lambda x: x[1])[0]


def traj2path(trajectory):
    path = Path()
    h = Header()
    path.header = h
    h.frame_id = 'odom'
    h.stamp = rospy.Time.now()
    for pt in trajectory:
        p = PoseStamped()
        p.header = h
        p.pose = Pose()
        p.pose.position = Point(pt[0], pt[1], 0)
        if len(pt) == 3:
            q = tf.transformations.quaternion_from_euler(0, 0, pt[2])
            p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        path.poses.append(p)
    return path


def input2twist(input):
    twist = Twist()
    twist.linear = Vector3(input[0], 0, 0)
    twist.angular = Vector3(0, 0, input[1])
    return twist


def pt2point(input):
    if input is None:
        return
    p = PointStamped()
    p.header = Header()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = 'odom'
    p.point = Point()
    p.point.x = input[0]
    p.point.y = input[1]
    p.point.z = 0
    return p


def main():
    rospy.init_node('sampling_planner')
    p = SamplingPlanner()
    p.start()

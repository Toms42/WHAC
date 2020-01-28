import rospy

from std_msgs.msg import Header
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from nav_msgs.srv import GetPlan, GetPlanResponse
import tf

from astar import a_star_search


class AstarNode:
    def __init__(self):
        self.mapSub = rospy.Subscriber(rospy.get_param('~map_topic', '/map'), OccupancyGrid, self.mapCallback)
        self.map = None
        self.planSrv = rospy.Service('~get_plan', GetPlan, self.getPlan)
        self.occupied_threshold = 0.5

    def mapCallback(self, map):
        self.map = map

    def getPlan(self, request):
        if self.map is None:
            rospy.logwarn("Plan requested before map was made available! Please try again later!")
            return GetPlanResponse()

        w = self.map.info.width
        h = self.map.info.height
        d = self.map.data

        res = self.map.info.resolution

        o = self.map.info.origin.position

        rospy.loginfo("Map metadata: {}".format(self.map.info))

        rospy.loginfo("Origin of occupancy grid: ({},{})".format(o.x, o.y))

        gx = int(float(request.goal.pose.position.x - o.x) / res)
        gy = int(float(request.goal.pose.position.y - o.y) / res)
        sx = int(float(request.start.pose.position.x - o.x) / res)
        sy = int(float(request.start.pose.position.y - o.y) / res)

        rospy.loginfo("Planning path from ({},{}) to ({},{}) across {} points with resolution {}".format(
            sx, sy, gx, gy, w * h, res))

        grid = [[0 for i in range(w)] for j in range(h)]

        num_ones = 0
        num_zeros = 0

        for r in range(h):
            for c in range(w):
                v = d[r * w + c]
                grid[r][c] = 1 if v > self.occupied_threshold * 100 or v < 0 else 0
                if grid[r][c] == 1:
                    num_ones += 1
                else:
                    num_zeros += 1

        rospy.loginfo("Thresholding complete! {} ones, {} zeros.".format(num_ones, num_zeros))
        rospy.loginfo("value at goal: {}".format(grid[sy][sx]))
        traj = a_star_search(grid, (sx, sy), (gx, gy))

        if traj is None:
            rospy.logwarn("No valid path found to goal! Returning")
            return GetPlanResponse()

        traj = [[pt[0] * res, pt[1] * res] for pt in traj]

        rospy.loginfo("...Found path of length {} points".format(len(traj)))

        response = GetPlanResponse()
        response.plan = traj2path(traj)
        return response


def traj2path(trajectory):
    path = Path()
    h = Header()
    path.header = h
    h.frame_id = 'map'
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


def main():
    rospy.init_node('astar_planner')
    a = AstarNode()
    rospy.spin()

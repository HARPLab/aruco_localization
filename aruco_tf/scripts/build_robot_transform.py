#!/usr/bin/env python


import os
import rospy
import numpy as np
import tf.transformations as transform
import yaml
from rospy_message_converter import message_converter

import harpmath.tf.rotations as rot

import std_msgs.msg
import geometry_msgs.msg
import aruco_detection.msg

class StaticOffsetCollector:
    def __init__(self, name):
        self.name = name
        self.translations = []
        self.quaternions = []
        
    def get_pose(self):
        if len(self.translations) == 0:
            return None
        
        trans = np.array(self.translations)
        trans_m = np.mean(trans, axis=0)
        trans_cov = np.std(trans, axis=0)
        
        quat = np.array(self.quaternions)
        quat_m = rot.average_quaternion(quat)
        quat_cov = rot.quaternion_covariance(quat, avg=quat_m)
        
        cov_matrix = np.eye(6)
        cov_matrix[[0,1,2],[0,1,2]] = trans_cov
        cov_matrix[[3,4,5],[3,4,5]] = quat_cov
        
        return geometry_msgs.msg.PoseWithCovarianceStamped(
            header = std_msgs.msg.Header(frame_id = self.name),
            pose = geometry_msgs.msg.PoseWithCovariance(
                pose = geometry_msgs.msg.Pose(
                    position = geometry_msgs.msg.Point(*trans_m.tolist()),
                    orientation = geometry_msgs.msg.Quaternion(*quat_m.tolist())
                ),
                covariance = cov_matrix.ravel().tolist()
            ))
        
    def do_update(self, pose, ref_pose):
        t = [ pose.pose.position.x, pose.pose.position.y, pose.pose.position.z ]
        q = [ pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z ]
        
        T_pose = transform.quaternion_matrix(q)
        T_pose[0:3,3] = t
        
        rospy.logdebug('{}: \n\tTref: {},\n\t Tpose: {}'.format(self.name, ref_pose, T_pose))
        T_ttl = np.dot(np.linalg.inv(ref_pose), T_pose)
        
        self.translations.append(T_ttl[0:3,3].tolist())
        self.quaternions.append(transform.quaternion_from_matrix(T_ttl).tolist())
        
        

class NumPointsEndCondition:
    def __init__(self, max_pts):
        self.num_pts = max_pts
    def update(self, _):
        self.num_pts -= 1
    def check(self):
        return (self.num_pts <= 0)
    
class TimeGapEndCondition:
    def __init__(self, gap):
        self.gap = gap
        self.last_tm = None
    def update(self, msg):
        self.last_tm = rospy.Time.now()
    def check(self):
        if self.last_tm is None:
            return False
        else:
            return (rospy.Time.now() - self.last_tm) >= self.gap
            

class OffsetCollector:
    def __init__(self, topic, ref_frame, ref_pose, end_condition=TimeGapEndCondition(rospy.Duration.from_sec(5.))):
        self.ref_frame = ref_frame
        self.ref_pose = ref_pose
        self.offset_collectors = {}
        self.end_condition = end_condition
        self.subscriber = rospy.Subscriber(topic, aruco_detection.msg.Boards, self.board_callback)
        
    def board_callback(self, msg):
        
        for board in msg.boards:
            if board.num_inliers > 0 and board.board_name != self.ref_frame:
                if board.board_name in self.offset_collectors:
                    self.offset_collectors[board.board_name].do_update(board.pose, self.ref_pose)
                else:
                    coll = StaticOffsetCollector(board.board_name)
                    self.offset_collectors[board.board_name] = coll
                    coll.do_update(board.pose, self.ref_pose)
        self.end_condition.update(msg)
                
    def is_finished(self):
        return self.end_condition.check()
    
    def finish(self, d):
        self.subscriber.unregister()
        self.dump_to_files(d)
    
    def dump_to_files(self, d):
        import datetime
        d = os.path.join(d, datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') )
        os.mkdir(d)
        
        raw_data = { c.name: { 'translation': c.translations, 'rotation': c.quaternions} for c in self.offset_collectors.values() }
        with open(os.path.join(d, 'raw_data.yaml'), 'wb') as f:
            yaml.dump(raw_data, f)
        
        pose = { c.name: c.get_pose() for c in self.offset_collectors.values() }
        pose = { k: v for k,v in pose.items() if v is not None } # filter nones here
        pose_raw = { k: message_converter.convert_ros_message_to_dictionary(pose[k]) for k in pose.keys() }
        with open(os.path.join(d, 'pose.yaml'), 'wb') as f:
            yaml.dump(pose_raw, f)
            
        with open(os.path.join(d, 'static_board.launch'), 'w') as f:
            f.write('<launch>\n')
            for name, p in pose.items():
                f.write('  <node pkg="tf2_ros" type="static_transform_publisher" name="{}_broadcaster" args="{} {} {} {} {} {} {} {} {}" />\n'.format(
                        name,
                        p.pose.pose.position.x,
                        p.pose.pose.position.y,
                        p.pose.pose.position.z,
                        p.pose.pose.orientation.x,
                        p.pose.pose.orientation.y,
                        p.pose.pose.orientation.z,
                        p.pose.pose.orientation.w,
                        self.ref_frame,
                        name
                    ))
            f.write('</launch>\n')
        rospy.loginfo('Wrote data for {} frames.'.format(len(self.offset_collectors.values()[0].translations)) )
        
    
def main():
    rospy.init_node('aruco_tf_build_robot')
    ref_frame = rospy.get_param('~robot_frame')
    ref_pose_raw = rospy.get_param('~ref_pose')
    topic = rospy.get_param('~topic')
    out_dir = rospy.get_param('~output_dir')
    R = np.array(ref_pose_raw['Rotation'])
    ref_tf = np.eye(4)
    ref_tf[0:3,0:3] = R
    ref_tf[0:3,3] = np.array(ref_pose_raw['Translation']).ravel()
    
    
    col = OffsetCollector(topic, ref_frame, ref_tf)
    
    while not rospy.is_shutdown() and not col.is_finished():
        rospy.sleep(0.5)
        
    col.finish(out_dir)
        
if __name__ == '__main__':
    main()
    


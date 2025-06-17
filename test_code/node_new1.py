#!/usr/bin/env python3
import os, json
from threading import Thread, Event
from datetime import datetime
import rospy
import numpy as np
from ros_ht_msg.msg import gps_filtered, ht_control

# 单例基类
class SingletonMixin:
    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

# 导航状态常量
NAV_STATUS_IDLE = '闲置'
NAV_STATUS_RUNNING = '导航中'
NAV_STATUS_DONE = '已完成'
# 最大path长度
maxlen = 30

class PathRecorder(SingletonMixin):
    def __init__(self, listname):
        if hasattr(self, 'points'):
            return
        self.listname = listname
        self.path_dir = './paths'
        os.makedirs(self.path_dir, exist_ok=True)
        self.path_file = os.path.join(self.path_dir, f'{listname}.json')
        self.points = []
        self.current_gps = None
        if os.path.exists(self.path_file):
            self.points = json.load(open(self.path_file))
        rospy.Subscriber('/gps/filtered', gps_filtered, self.gps_callback)

    @classmethod
    def get_instance(cls, listname=None):
        inst = cls(listname)
        if listname:
            inst.__init__(listname)
        return inst

    def gps_callback(self, msg):
        self.current_gps = msg

    def _get_real_gps(self, timeout=5.0):
        start = rospy.Time.now()
        rospy.sleep(1)
        while not self.current_gps and (rospy.Time.now()-start).to_sec()<timeout:
            rospy.sleep(1)
        return self.current_gps

    def _mock_gps(self):
        gps= self._get_real_gps()
        if gps:
            return {
                'lat':gps.latitude, 'lng':gps.longitude, 'yaw':gps.yaw,
                'x':gps.x, 'y':gps.y, 'angle':gps.angle,
                'time':datetime.now().strftime('%H:%M:%S')
            }
        rospy.logwarn('无法获取GPS')
        return None

    def add_point(self, comment=''):
        p=self._mock_gps()
        if p:
            p['comment']=comment
            self.points.append(p)
            self._save()

    def insert_point(self, index, comment=''):
        p=self._mock_gps()
        if p and 0<=int(index)<=len(self.points):
            p['comment']=comment
            self.points.insert(int(index),p)
            self._save()

    def delete_point(self, index):
        if 0<=int(index)<len(self.points):
            self.points.pop(int(index))
            self._save()

    def load(self, listname):
        self.__init__(listname)
        return self

    def capture_points(self, name=None):
        # 依旧在 main 中交互实现，此处保持占位
        return self

    def _save(self):
        with open(self.path_file,'w') as f:
            json.dump(self.points,f,indent=2)

class BasePlateCommand:
    def __init__(self, seq, V=100, L=174):
        self.pub = rospy.Publisher('/HT_Control', ht_control, queue_size=10)
        self.seq, self.V, self.L = seq, V, L

    def rotate(self, omega):
        ctrl=ht_control(); ctrl.mode=1; ctrl.x=0; ctrl.y=self.L if omega>0 else -self.L; ctrl.stop=0
        self.pub.publish(ctrl)
        duration=abs(omega)/self.L
        for _ in range(int(duration)):
            self.pub.publish(ctrl); rospy.sleep(1)
        rospy.sleep(duration%1); self.stop()

    def move_straight(self, d):
        ctrl=ht_control(); ctrl.mode=1; ctrl.x=self.V; ctrl.y=0; ctrl.stop=0
        self.pub.publish(ctrl)
        duration=d/(self.V/1000)
        for _ in range(int(duration)):
            self.pub.publish(ctrl); rospy.sleep(1)
        rospy.sleep(duration%1); self.stop()

    def stop(self):
        ctrl=ht_control(); ctrl.mode=1; ctrl.x=0; ctrl.y=0; ctrl.stop=1
        self.pub.publish(ctrl)

class Naver(Thread, SingletonMixin):
    def __init__(self, dir, listname, mvto_point=None):
        if hasattr(self, 'points'):
            return
        Thread.__init__(self)
        self.recorder=PathRecorder.get_instance(listname)
        self.points=self.recorder.points
        self.i=mvto_point or 0
        self.stop=Event(); self.status=NAV_STATUS_IDLE

    @classmethod
    def get_instance(cls, dir=None, listname=None, at_point=None):
        return cls(dir, listname, mvto_point=at_point)

    def run(self):
        self.status=NAV_STATUS_RUNNING
        for idx in range(self.i, len(self.points)-1):
            if self.stop.is_set(): break
            s,t=self.points[idx],self.points[idx+1]
            omega,dist=self.compute_motion_params(s,t)
            BasePlateCommand([(omega,dist)]).run()
        self.status=NAV_STATUS_DONE

    def start_nav(self, dir, listname):
        self.__init__(dir,listname)
        self.start()

    def mvto(self, at_point):
        self.__init__('forward',self.recorder.listname, mvto_point=int(at_point))
        self.start()

    def stop_nav(self):
        self.stop.set()

    def auto_nav(self, listname):
        self.start_nav('forward', listname)

    def compute_motion_params(self, start, target, V=100, Kp=1.0):
        lat1,lon1,yaw=float(start['lat']),float(start['lng']),float(start.get('yaw',0))
        lat2,lon2=float(target['lat']),float(target['lng'])
        lat1,lon1,lat2,lon2=np.radians(lat1),np.radians(lon1),np.radians(lat2),np.radians(lon2)
        dlat,dlon=lat2-lat1,lon2-lon1
        angle=np.arctan2(dlon,dlat)
        err=(angle-yaw+np.pi)%(2*np.pi)-np.pi
        return Kp*err, np.sqrt((dlat*6371000)**2+(dlon*6371000)**2)
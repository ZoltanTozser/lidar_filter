#!/usr/bin/env python3

# A programhoz használt library-k.
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

# A programhoz használt globális változók.
min_X = 0
max_X = 30
min_Y = -10
max_Y = 10
min_Z = -2
max_Z = -1

fixedFrame ='left_os1/os1_lidar' 
topicName = 'left_os1/os1_cloud_node/points'


def lidarFiltered(msg):
        
        boxpoints = []
        
        # Amire felíratkozunk topicra, abból kiolvassuk a pontfelhő point xyz adataid. 
        for point in point_cloud2.read_points(msg, skip_nans=True):
                
                # A beállított globális változók szerint kiválogatja azokat a pontokat, amik megfelelnek a feltételnek.
                # Egy téglatestet kell elképzelni, amiben a pontok vannak. Ezeket a pontokat egy listába tesszük. 
                if(point[0] >= min_X and point[0] <= max_X 
                and point[1] >= min_Y and point[1] <= max_Y 
                and point[2] >= min_Z and point[2] <= max_Z):
                        
                        pt = [point[0], point[1], point[2]]
                        boxpoints.append(pt)

        # A téglatestben található pontok listáját numpy többé konvertáljuk. Ez egy kétdimenziós tömb. 
        boxpoints = np.array(boxpoints, dtype=np.float32)
        
        # A tömbben található elemek darabszáma. 
        piece = len(boxpoints)

        # Egy pont origótól mért távolsága. Ez lesz a 2D tömb 4. oszlopa.
        origodistance = np.array([])

        # Egy pont szögfelbontása.  Ez lesz a 2D tömb 5. oszlopa. 
        alpha = np.array([])
        
        # Egy adott pillanatban végigmegy az összes vizsgált ponton. 
        for i in range(0,piece):
        
                # Feltölti a 4. oszlopot. 
                origodistance = np.append(origodistance, np.sqrt(np.power(boxpoints[i,0],2) + np.power(boxpoints[i,1],2) + np.power(boxpoints[i,2],2))) 

                # Feltölti az 5. oszlopot (alpha).
                bracket = np.abs(boxpoints[i, 2]) / (origodistance[i])

                if(bracket < -1):
                        bracket = -1
                elif(bracket > 1):
                        bracket = 1

                if(boxpoints[i,2] < 0):
                        alpha = np.append(alpha, np.arccos(bracket) * 180 / np.pi)
                elif(boxpoints[i,2] >= 0):
                        alpha = np.append(alpha, (np.arcsin(bracket) * 180 / np.pi) + 90)
                
                
        # Mivel az 4. és az 5. oszlopok egydimenzió listák, ezért átalakítjuk 1 oszlopos 2D-s tömbbé mindkettő tömböt. 
        origodistance = origodistance.reshape((-1, 1))
        alpha = alpha.reshape((-1, 1))
        
        # Megjelenítjük console-on az 5 oszlop adatait, tehát a kialakított 2D-s tömböt.
        # print(np.hstack((boxpoints,origodistance, alpha)))
        
        # Header és Pointfield adatok megadása a létrehozandó pontfelhőhöz. 
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = fixedFrame

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1), 
                  PointField('z', 8, PointField.FLOAT32, 1)]

        # Létrehozzuk a pontfelhőt, majd publikáljuk. 
        testPointcloud = point_cloud2.create_cloud(header, fields, boxpoints)
        testpub.publish(testPointcloud)              

                   
if __name__ == '__main__':

        # ROS node létrehozása. 
        rospy.init_node('filter')
        rospy.loginfo("Node Initializing.")

        # Felíratkozunk a topicra. 
        lidar_sub = rospy.Subscriber(topicName, PointCloud2, lidarFiltered, queue_size=1)
        rospy.loginfo("Subscribe to the topic: %s.", topicName)

        # Létrehozunk egy Publisher-t, aminek a neve 'testponts', a típusa PointCloud2                
        testpub = rospy.Publisher('testpoints', PointCloud2, queue_size=1)
        rospy.loginfo("Publish to the topic: testpoints.")

        rospy.spin()

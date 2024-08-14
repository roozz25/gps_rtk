#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import PoseStamped
from pyproj import Proj
import numpy as np
from pyubx2 import UBXReader
from scipy.spatial.transform import Rotation as R


#Fonction convertissant  les angles d'euler en quaternion 
def get_quaternion_from_euler(roll, pitch, yaw):
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    return rotation.as_quat()
    
#Fonction qui calcule l'orientation du robot par rapport au nord 
def calculate_heading(x, y):
    u = np.array([1, 0, 0]) + 1e-10
    v = np.array([x, y, 0]) + 1e-10
    cros = np.cross(u, v)
    s, c = np.linalg.norm(cros)/np.linalg.norm(v), x/np.linalg.norm(v) 
    rad_angle = np.arctan2(s, c)
    angle = np.rad2deg(rad_angle)
    if(y >  0):
        angle = -angle
    return  angle

#Fonction qui convertit les coordonnées gps latitude et longitude en coordonnées X,Y en metre
#Toulouse se trouve dans la zone UTM 31, Zone à ajuster en fonction du lieu d'utilisation
def convert_coordinates(lat,lon):
    my_proj = Proj(proj='utm', zone=31, ellps='WGS84', datum='WGS84', units='m', no_defs=True)
    est, nord = my_proj(lon, lat)
    return nord , est

#fonction principale 
def talker():
    cpt = 0    # compteur qui servira pour l'initialisation de des coordonnées initiales du robot
    x_base = 0  #variables qui stockeront les coordonnées intiales
    y_base = 0
    x_tmp = 0 #variables qui stockeront les coordonnées globales actuelles du robot
    y_tmp = 0

    
    #création du publisher et du node pour la publication d'un message de type PoseStamped
    pub = rospy.Publisher('gps_rover/fix_try', PoseStamped, queue_size=10)
    rospy.init_node('GPS_RTK', anonymous=True)
    rate = rospy.Rate(10)

    #acces au gps via lien dynamique 
    # dans le fichier /etc/udev/rules.d/50-ardusimple.rules
    #ajouter la ligne KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_Ardusimple_Rover", GROUP="dialout", MODE="0666"
    ser = serial.Serial('/dev/tty_Ardusimple_Rover' , 115200, timeout =1)
    reader = UBXReader(ser)

    #creation et initialisation du message ROS
    msg = PoseStamped() 
    msg.header.frame_id = "rover"
    msg.pose.position.x = 0 
    msg.pose.position.y = 0 
    msg.pose.position.z = 0
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 1


    # "main" du script lecture et fromatage des données pour publication du message
    while not rospy.is_shutdown():
        try:
            if(rospy.is_shutdown()): 
                break
            # variables de test pour s'assurer qu'un message contiendra bien la maj de la position et de l'orientation
            pos = True
            ori = True
            while(pos or ori):
                if(rospy.is_shutdown()): 
                    break
                #lecture des données du gps
                (raw_data, parsed_data) = reader.read()

                # Moyenne des 25 premières position pour créer l'origine du repère du gps
                if (cpt < 25):
                    if(parsed_data.identity == "NAV-PVT"): #la position se trouve dans les messages PVT du gps
                        utmx, utmy = convert_coordinates(parsed_data.lat,parsed_data.lon)
                        x_base += utmx
                        y_base += utmy
                        cpt += 1
                        if(cpt == 25):
                            x_base = round(x_base/25, 2)
                            y_base = round(y_base/25, 2)
                            print(f"base configurée \nX_base : {x_base}  Y_base : {y_base}")
                else:


                    #recupération des données de position 
                    if(parsed_data.identity == "NAV-PVT" and pos):
                    
                        tmpx, tmpy = convert_coordinates(parsed_data.lat,parsed_data.lon)
                        x = tmpx - x_base
                        y = tmpy - y_base
                        msg.pose.position.x = x
                        msg.pose.position.y = -y
                        pos = False

                    #recupération des données d'orientation dans les messages RELPOSNED 
                    #relPosN et relPosE donne respectivement la distance par rapport au nord et a l'est entre les deux antennes
                    elif(parsed_data.identity == "NAV-RELPOSNED" and ori):
                        nord = - parsed_data.relPosN
                        est = - parsed_data.relPosE
                        heading = calculate_heading(nord,est )
                        orientation = get_quaternion_from_euler(0, 0, heading) 
                        msg.pose.orientation.x = orientation[0]
                        msg.pose.orientation.y = orientation[1]
                        msg.pose.orientation.z = orientation[2]
                        msg.pose.orientation.w = orientation[3]
                        
                        ori = False

            msg.header.stamp = rospy.Time.now()
            #publication du message
            pub.publish(msg)
            rate.sleep()
        except Exception as e:
            rospy.logerr(f"Erreur : {e}")



if __name__ == '__main__':
    
    talker()
    
               

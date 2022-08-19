#from scipy.signal import savgol_filter
#import matplotlib.pyplot as plt
import sqlite3
#from math import pi,pow,sqrt
import numpy as np
yaw = 0
meas = 1
odom = 2
imu = 3
imu_int = 4
meas_x = 5
meas_y = 6
odom_x = 7
odom_y = 8
goal_x= 9
goal_y = 10
#dbfile = "/home/kotname/ros_ws/src/Beleg/localization/data_eval/Messungen.db"

dbfile = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db"
# Prozessfehler zw. messung & Zielrotation
# restliche Berechnung mit Messungsgrundlage
def calc_mean(table,tab_length):
    proc_mean = sum(1 -table[:,meas]/table[:,yaw]) / tab_length
    odom_mean = sum(1 - table[:,meas]/table[:,odom] ) / tab_length
    imu_mean = sum(1  - table[:,meas]/table[:, imu] ) / tab_length
    imu_int_mean = sum(1 - table[:,meas]/ table[:, imu_int]) / tab_length
    return {"proc_mean": proc_mean, "odom_mean": odom_mean,"imu_mean": imu_mean, "imu_int_mean":imu_int_mean}


# meas
def calc_variance(table,mean,tab_len):
    proc_var = sum((1 - table[:,meas]/table[:,yaw] - mean["proc_mean"])**2)/tab_len
    odom_var = sum((1 - table[:,meas]/table[:,odom] - mean["odom_mean"])**2)/tab_len
    imu_var = sum((1 -table[:,meas]/table[:,imu] - mean["imu_mean"])**2)/tab_len
    imu_int_var = sum((1 -table[:,meas]/table[:,imu_int] - mean["imu_int_mean"])**2) /tab_len
    return  {"proc_var": proc_var, "odom_var": odom_var,"imu_var": imu_var, "imu_int_var":imu_int_var}

# to calc. the covariance E[X,Y] = (X-Y)**2 but needs to be normalized (by radians) -> ((X-Y)/X)**2
# however we devided X by Y in the SQL-SElect statement (for normalization) already
# -> E[X,Y] = (1 - X)**2 = (1-X/Y)**2 = ((X-Y)/X)**2
def calc_covariance(table,tab_len):
    proc_yaw_cov = sum((1 - table[:,meas])**2) /tab_len
    odom_yaw_cov = sum((1 - table[:,odom])**2)/tab_len
    imu_yaw_cov = sum(( 1 - table[:,imu] )**2)/tab_len
    imu_int_yaw_cov = sum((1 - table[:,imu_int])**2) /tab_len
    return  {"proc_yaw_cov": proc_yaw_cov, "odom_yaw_cov": odom_yaw_cov,"imu_yaw_cov": imu_yaw_cov, "imu_int_yaw_cov":imu_int_yaw_cov}


# ein merkwürdiger wert bei imu_int vel = 1 vlt wegen fehler o.ä.
if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    tables = ["0.25","0.5","0.75","1"]
    for tab in tables:
        query = "SELECT yaw,ang_speed, meas_yaw,odom_yaw,imu_yaw,imu_int_yaw FROM yaw WHERE ang_speed= " + tab
        cursor.execute(query)
        table = np.array(cursor.fetchall())
        tab_len = table.shape[0]
        speed = table[0,1]
        print("Geschwindigkeit: ", speed)
        table = np.delete(table,1,1)
        mean = calc_mean(table,tab_len)
        var = calc_variance(table,mean,tab_len)
        cov = calc_covariance(table,tab_len)
        cursor.execute('''INSERT INTO yaw_eval (ang_speed, proc_mean, proc_var,proc_cov,odom_mean,odom_var,odom_cov, imu_mean, imu_var,imu_cov,imu_int_mean, imu_int_var,imu_int_cov,data_count) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)'''
                       ,(speed,mean["proc_mean"],var["proc_var"], cov["proc_yaw_cov"],mean["odom_mean"],var["odom_var"],cov["odom_yaw_cov"], mean["imu_mean"],var["imu_var"],cov["imu_yaw_cov"],mean["imu_int_mean"],var["imu_int_var"],cov["imu_int_yaw_cov"],tab_len))
        print(mean)
        print(var)
        #print(cov)
    db_connector.commit()
    db_connector.close()

from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import sqlite3
from math import pi,pow,sqrt
import numpy as np
yaw = 0
meas = 1
odom = 2
imu = 3
imu_int = 4

dbfile = "/home/kotname/ros_ws/src/Beleg/localization/data_eval/Messungen.db"
# Prozessfehler zw. messung & Zielrotation
# restliche Berechnung mit Messungsgrundlage
def calc_mean(table,tab_length):
    proc_mean = sum(table[:,meas] - 1) / tab_length
    odom_mean = sum(table[:, odom] -1 ) / tab_length
    imu_mean = sum(table[:, imu] -1 ) / tab_length
    imu_int_mean = sum(table[:, imu_int] -1 ) / tab_length
    return {"proc_mean": proc_mean, "odom_mean": odom_mean,"imu_mean": imu_mean, "imu_int_mean":imu_int_mean}



def calc_variance(table,mean):
    proc_var = sum((table[:,meas] - 1 - mean["proc_mean"])**2)
    odom_var = sum((table[:,odom] - 1 - mean["odom_mean"])**2)
    imu_var = sum((table[:,imu] - 1 - mean["imu_mean"])**2)
    imu_int_var = sum((table[:,imu_int] - 1 - mean["imu_int_mean"])**2)
    return  {"proc_var": proc_var, "odom_var": odom_var,"imu_var": imu_var, "imu_int_var":imu_int_var}

'''def calc_covariance(table):
    proc_cov = sum((table[:,meas] - table[:,yaw])**2)
    odom_cov = sum((table[:,odom] - mean["odom"])**2)
    imu_cov = sum((table[:,imu] - mean["imu"])**2)
    imu_int_cov = sum((table[:,imu_int] - mean["imu_int"])**2)
    return  {"meas": proc_cov, "odom": odom_cov,"imu": imu_cov, "imu_int":imu_int_cov}
'''

# ein merkwürdiger wert bei imu_int vel = 1 vlt wegen fehler o.ä.
if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    tables = ["yaw_vel_0_25","yaw_vel_0_5","yaw_vel_0_75","yaw_vel_1"]
    for tab in tables:
        query = "SELECT yaw,ang_speed, meas_yaw/yaw,odom_yaw/meas_yaw,imu_yaw/meas_yaw,imu_int_yaw/meas_yaw FROM " + tab
        cursor.execute(query)
        table = np.array(cursor.fetchall())
        tab_len = table.shape[0]
        speed = table[0,1]
        print("Geschwindigkeit: ", speed)
        table = np.delete(table,1,1)
        mean = calc_mean(table,tab_len)
        var = calc_variance(table,mean)
        cursor.execute('''INSERT INTO yaw_eval (ang_speed, proc_mean, proc_var,odom_mean,odom_var, imu_mean, imu_var, imu_int_mean, imu_int_var,data_count) VALUES (?,?,?,?,?,?,?,?,?,?)'''
                       ,(speed,mean["proc_mean"],mean["odom_mean"],mean["imu_mean"],mean["imu_int_mean"],var["proc_var"],var["odom_var"], var["imu_var"],var["imu_int_var"],tab_len))
        print(mean)
        print(var)
    db_connector.commit()
    db_connector.close()
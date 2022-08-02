from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import sqlite3
from math import pi,pow,sqrt
import numpy as np
dist = 0
x_meas = 1
y_meas = 2
x_odom = 3
y_odom = 4

dbfile = "/home/kotname/ros_ws/src/Beleg/localization/data_eval/Messungen.db"
# Prozessfehler zw. messung & Zielrotation
# restliche Berechnung mit Messungsgrundlage
def calc_mean(table):
    tab_length = table.shape[0]
    proc_x_mean = sum(table[:,x_meas] - 1) / tab_length
    proc_y_mean = sum(table[:,y_meas] - 1) / tab_length
    odom_x_mean = sum(table[:, x_odom] -1 ) / tab_length
    odom_y_mean = sum(table[:, y_odom] -1 ) / tab_length
    return {"proc_x": proc_x_mean, "proc_y": proc_y_mean, "odom_x": odom_x_mean,"odom_y": odom_y_mean}



def calc_variance(table,mean):
    proc_var_x = sum((table[:,x_meas] - 1 - mean["proc_x"])**2)
    proc_var_y = sum((table[:,y_meas] - 1 - mean["proc_y"])**2)
    odom_var_x = sum((table[:,x_odom] - 1 - mean["odom_x"])**2)
    odom_var_y = sum((table[:,y_odom] - 1 - mean["odom_y"])**2)

    return {"proc_var_x": proc_var_x, "proc_var_y": proc_var_y, "odom_var_x": odom_var_x,"odom_var_y": odom_var_y}


#0.327 hier offset rausgerechnet

# ein merkwürdiger wert bei imu_int vel = 1 vlt wegen fehler o.ä.
if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    tables = ["acc_vel_0_1","acc_vel_0_2","acc_vel_0_4","acc_vel_0_6","acc_vel_0_8"]
    for tab in tables:
        query = "SELECT distance,speed, (x_meas -0.327) /distance,y_meas/distance,x_odom/x_meas,y_odom/y_meas FROM " + tab
        cursor.execute(query)
        table = np.array(cursor.fetchall())
        print("Geschwindigkeit: ", table[0,1])
        table = np.delete(table,1,1)
        mean = calc_mean(table)
        var = calc_variance(table,mean)
        print(mean)
        print(var)

    db_connector.close()
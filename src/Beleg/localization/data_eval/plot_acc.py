#from scipy.signal import savgol_filter
#import matplotlib.pyplot as plt
import sqlite3
#from math import pi,pow,sqrt
import numpy as np
dist = 0
x_meas = 1
y_meas = 2
x_odom = 3
y_odom = 4

dbfile = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db"
# Prozessfehler zw. messung & Zielrotation
# restliche Berechnung mit Messungsgrundlage
def calc_mean(table, tab_length):
    proc_x_mean = sum(table[:,x_meas] - 1) / tab_length
    proc_y_mean = sum(table[:,y_meas] - 1) / tab_length
    odom_x_mean = sum(table[:, x_odom] -1 ) / tab_length
    odom_y_mean = sum(table[:, y_odom] -1 ) / tab_length
    return {"proc_x_mean": proc_x_mean, "proc_y_mean": proc_y_mean, "odom_x_mean": odom_x_mean,"odom_y_mean": odom_y_mean}



def calc_variance(table,mean):
    proc_x_var = sum((table[:,x_meas] - 1 - mean["proc_x_mean"])**2)
    proc_y_var = sum((table[:,y_meas] - 1 - mean["proc_y_mean"])**2)
    odom_x_var = sum((table[:,x_odom] - 1 - mean["odom_x_mean"])**2)
    odom_y_var = sum((table[:,y_odom] - 1 - mean["odom_y_mean"])**2)

    return {"proc_x_var": proc_x_var, "proc_y_var": proc_y_var, "odom_x_var": odom_x_var,"odom_y_var": odom_y_var}


#0.327 hier offset rausgerechnet

# ein merkwürdiger wert bei imu_int vel = 1 vlt wegen fehler o.ä.
if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    tables = ["acc_vel_0_1","acc_vel_0_2","acc_vel_0_4","acc_vel_0_6","acc_vel_0_8"]
    for tab in tables:
        query = "SELECT distance,speed, (x_meas -0.327) /distance,y_meas/distance,x_odom/(x_meas -0.327),y_odom/y_meas FROM " + tab
        cursor.execute(query)
        table = np.array(cursor.fetchall())
        speed = table[0,1]
        tab_len = table.shape[0]
        print("Geschwindigkeit: ", speed)
        table = np.delete(table,1,1)
        mean = calc_mean(table,tab_len)
        var = calc_variance(table,mean)
        cursor.execute('''INSERT INTO acc_eval (speed,proc_x_mean,proc_y_mean,proc_x_var,proc_y_var, odom_x_mean, odom_y_mean, odom_x_var, odom_y_var, data_count) VALUES (?,?,?,?,?,?,?,?,?,?)''',
                       (speed,mean["proc_x_mean"],mean["proc_y_mean"], var["proc_x_var"], var["proc_y_var"], mean["odom_x_mean"],mean["odom_y_mean"], var["odom_x_var"], var["odom_y_var"],tab_len))
        print(mean)
        print(var)
    db_connector.commit()
    db_connector.close()
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
#dbfile = "/home/kotname/ros_ws/src/Beleg/localization/data_eval/Messungen.db"

dbfile = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db"
# Prozessfehler zw. messung & Zielrotation
# restliche Berechnung mit Messungsgrundlage
def calc_mean(table, tab_length):
    proc_x_mean = sum( 1 - table[:,x_meas]/table[:,dist]) / tab_length #1 = distance to target
    proc_y_mean = sum(table[:,y_meas]) / tab_length # target is 0
    odom_x_mean = sum(1 - table[:,x_meas]/table[:, x_odom]) / tab_length
    odom_y_mean = sum(table[:,y_odom] - table[:, y_meas] ) / tab_length
    return {"proc_x_mean": proc_x_mean, "proc_y_mean": proc_y_mean, "odom_x_mean": odom_x_mean,"odom_y_mean": odom_y_mean}



def calc_variance(table,mean,tab_len):
    proc_x_var = sum(( 1 - table[:,x_meas] - mean["proc_x_mean"])**2) /tab_len
    proc_y_var = sum((table[:,y_meas] - mean["proc_y_mean"])**2) /tab_len
    odom_x_var = sum((1 - table[:,x_meas] / table[:,x_odom] - mean["odom_x_mean"])**2) /tab_len
    odom_y_var = sum((table[:,y_odom] - table[:, y_meas] - mean["odom_y_mean"])**2) /tab_len

    return {"proc_x_var": proc_x_var, "proc_y_var": proc_y_var, "odom_x_var": odom_x_var,"odom_y_var": odom_y_var}

def calc_covariance(table,tab_len):
    proc_x_cov = sum((1 - table[:,x_meas] )**2) /tab_len
    proc_y_cov = sum((table[:,y_meas] )**2)/tab_len
    odom_x_cov = sum((1- table[:,x_odom])**2)/tab_len
    odom_y_cov = sum((table[:,y_meas] - table[:,y_odom])**2)/tab_len
    return  {"odom_x_cov": odom_x_cov, "odom_y_cov": odom_y_cov, "proc_x_cov":proc_x_cov,"proc_y_cov":proc_y_cov}


#0.327 hier offset rausgerechnet

# ein merkwürdiger wert bei imu_int vel = 1 vlt wegen fehler o.ä.
if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    tables = ["0.1","0.2","0.4","0.6","0.8"]
    for tab in tables:
        query = "SELECT distance,speed, (x_meas -0.327),(y_meas-0.284),x_odom,y_odom FROM acc WHERE speed=" + tab
        cursor.execute(query)
        table = np.array(cursor.fetchall())
        speed = table[0,1]
        tab_len = table.shape[0]
        print("Geschwindigkeit: ", speed)
        table = np.delete(table,1,1)
        mean = calc_mean(table,tab_len)
        var = calc_variance(table,mean,tab_len)
        cov = calc_covariance(table,tab_len)
        cursor.execute('''INSERT INTO acc_eval (speed,proc_x_mean,proc_y_mean,proc_x_var,proc_y_var, odom_x_mean, odom_y_mean, odom_x_var, odom_y_var, data_count) VALUES (?,?,?,?,?,?,?,?,?,?)''',
                       (speed,mean["proc_x_mean"],mean["proc_y_mean"], var["proc_x_var"], var["proc_y_var"], mean["odom_x_mean"],mean["odom_y_mean"], var["odom_x_var"], var["odom_y_var"],tab_len))
        print(mean)
        print(var)
        #print(cov)
    db_connector.commit()
    db_connector.close()

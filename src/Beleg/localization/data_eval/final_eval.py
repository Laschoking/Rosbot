#from scipy.signal import savgol_filter
#import matplotlib.pyplot as plt
import sqlite3
#from math import pi,pow,sqrt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
dbfile = "/home/kotname/ros_ws/src/Beleg/localization/data_eval/Messungen.db"

#dbfile = "/home/husarion/husarion_ws/src/Beleg/localization/data_eval/Messungen.db"


'''    #combs = []
    #for (i in range(len(combs)))
    min_len = cursor.execute("SELECT COUNT(*) as m from acc group by speed ORDER BY m ASC LIMIT 1;").fetchone()
    print(min_len[0])
    #(x_meas -0.327),(y_meas-0.284)
    query = "SELECT speed from evaluation group by speed;"
    res = pd.read_sql_query(query, db_connector)
    df_x = pd.DataFrame()
    df_y = pd.DataFrame()
    i =0
    for row in res.itertuples():
        id = str(row.speed)
        query = "SELECT x_meas - 1.327 as \"x_"+ id +  "\", y_meas-0.284 as \"y_"+id+ "\" FROM acc where speed=" + str(row.speed) +  ";"
        r = pd.read_sql_query(query,db_connector)

        df_x = pd.concat([df_x,r["x_" + id]],axis=1)
        df_y = pd.concat([df_y,r["y_" + id]],ignore_index=True,axis=1)

        print("nummer: " + str(i) +" "+ id)
        i += 1
    #plt.boxplot(df_x)
    #plt.boxplot(df_y)

    #ax.imshow(I)
    labels = [0,0.2,0.4,0.6,0.8]
    #x_labels += list(df_x.columns.values)
    df_x.plot(kind='box',title="X-Abweichung").set_xticks(ticks = range(len(labels)),labels=labels) # NAN enhalten daher nicht boxplot()

    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/X-Abweichung.png")
    df_y.plot(kind='box',title="Y-Abweichung").set_xticks(ticks = range(len(labels)),labels=labels) # NAN enhalten daher nicht boxplot()
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/Y-Abweichung.png")
    plt.show()'''
def evalHit(cursor,db_connector):
    query = "SELECT speed,ang_vel,place FROM evaluation GROUP BY speed,ang_vel,place"
    res = pd.read_sql_query(query, db_connector)
    for row in res.itertuples():
        cursor.execute("SELECT COUNT(*) FROM evaluation where place=\""+row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        max_len = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM evaluation WHERE sqrt(pow(amcl_x - goal_x,2) + pow(amcl_y- goal_y,2)) <= 0.07 AND sqrt(pow(meas_x - goal_x,2) + pow(meas_y- goal_y,2)) <= 0.07 AND place=\""+row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        tp = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM evaluation WHERE sqrt(pow(amcl_x - goal_x,2) + pow(amcl_y- goal_y,2)) > 0.07 AND sqrt(pow(meas_x - goal_x,2) + pow(meas_y- goal_y,2)) > 0.07 AND place=\""+row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        tn = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM evaluation WHERE sqrt(pow(amcl_x - goal_x,2) + pow(amcl_y- goal_y,2)) <= 0.07 AND sqrt(pow(meas_x - goal_x,2) + pow(meas_y- goal_y,2)) > 0.07 AND place=\""+row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        fp = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM evaluation WHERE sqrt(pow(amcl_x - goal_x,2) + pow(amcl_y- goal_y,2)) > 0.07 AND sqrt(pow(meas_x - goal_x,2) + pow(meas_y- goal_y,2)) <= 0.07 AND place=\""+row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        fn = cursor.fetchone()[0]
        print(row)
        print(tp/max_len)
        cursor.execute("UPDATE phit SET TP="+str(round(tp/max_len,3)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        cursor.execute("UPDATE phit SET TN="+str(round(tn/max_len,3)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        cursor.execute("UPDATE phit SET FP="+str(round(fp/max_len,3)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")
        cursor.execute("UPDATE phit SET FN="+str(round(fn/max_len,3)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;")

        #cursor.execute("INSERT INTO phit VALUES(?,?,?,?,?,?,?,?)",(row.place,row.speed,row.ang_vel,round(tp/max_len,3),round(tn/max_len,3),round(fp/max_len,3),round(fn/max_len,3),max_len))
    db_connector.commit()

def plotAMCLDist2(db_connector):
    #query = "SELECT goal_nr,place FROM evaluation GROUP BY goal_nr,place"
    df = pd.DataFrame()

    #ps= [(0,0),(0.5, 1.5),(1.5,0.5),(2,-1),(1,0.5),(0.5,-0.5),(0,0.3)]
    ps= [(0,0),(3.5,0.2),(3.6, -3.5),(3.5,0),(6,-0.1),(0.8,0)]
    for i in range(5):

        #print(i)
        query = "SELECT meas_x  - 0.0 as x_diff,meas_y  - 0.0 as y_diff,amcl_x  - 0.0 as x_amcl,amcl_y  - 0.0 as y_amcl, goal_nr,place FROM evaluation where  goal_nr =" + str(i) + " and place like 'floor%';"
        res = pd.read_sql_query(query,db_connector)
        #res.plot.scatter('x_diff','x_diff',c='goal_nr', colormap='gnuplot')
        #df = pd.concat([df, res], ignore_index=False, axis=1)
        p = ps.pop(0)
        # c=(res['place']=='floor_5cm'),cmap='PiYG',

        if i == 0:
            f = res.groupby('place')
            for key,group in f:
                if key == 'floor_5cm':
                    plt.scatter(group['x_diff'],group['y_diff'],label='Messposition,5cm Raster', marker='o',c='indigo',cmap='PiYG',s=6)
                else:
                    plt.scatter(group['x_diff'],group['y_diff'],label='Messposition,10cm Raster', marker='o',c='darkgreen',cmap='PiYG',s=6)

            plt.plot([p[0],ps[0][0]],[p[1],ps[0][1]],c ='black', label='Pfad',marker='o',markersize=4,linewidth=0.7)
        else:

            f = res.groupby('place')
            for key,group in f:
                if key == 'floor_5cm':
                    plt.scatter(group['x_diff'],group['y_diff'], marker='o',c='indigo',cmap='PiYG',s=6)
                else:
                    plt.scatter(group['x_diff'],group['y_diff'], marker='o',c='darkgreen',cmap='PiYG',s=6)

            plt.plot([p[0],ps[0][0]],[p[1],ps[0][1]],c ='black',marker='o',markersize=4,linewidth=0.7)


            #plt.scatter(res['y_diff'], res['x_diff'], marker='o', c=(res['place']=='floor_5cm'),cmap='PiYG',s=6)
            #plt.plot([p[1], ps[0][1]], [p[0], ps[0][0]], c='black', marker='o', markersize=4,
            #         linewidth=0.5)


        #df.plot('x_diff1','y_diff1',kind='scatter',c='g')
        #df.plot(x='x_diff',y='y_diff',c='b',kind='scatter')
    #Y als Y-aches und umgekehrte, da X vorwärts
    plt.xlim(7,-1)
    plt.ylim(0.7,-4.3)
    #handles = ['b','darkorange','black']
    #plt.legend(['meas-pose','amcl-pose','path'])
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    #plt.figlegend()
    #plt.title("Alle 6 Zielpunkte")
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/AMCL-AbweichungFlur_gedreht.eps")

    plt.show()

def plotAMCLDist(db_connector):
    #query = "SELECT goal_nr,place FROM evaluation GROUP BY goal_nr,place"
    color = ['r','b','y','g','c','b']
    df = pd.DataFrame()

    ps= [(0,0),(0.5, 1.5),(1.5,0.5),(2,-1),(1,0.5),(0.5,-0.5),(0,0.3)]
    #ps= [(0,0),(3.5,0.2),(3.6, -3.5),(3.5,0),(6,-0.1),(0.8,0)]
    for i in range(6):
        #print(i)
        query = "SELECT meas_x  - 0.0 as x_diff,meas_y  - 0.0 as y_diff,amcl_x  - 0.0 as x_amcl,amcl_y  - 0.0 as y_amcl, goal_nr,place FROM evaluation where  goal_nr =" + str(i) + " and place like 'E53%';"
        res = pd.read_sql_query(query,db_connector)

        p = ps.pop(0)
        # c=(res['place']=='floor_5cm'),cmap='PiYG',

        if i == 0:
            plt.scatter(res['y_diff'],res['x_diff'],label='Messposition', marker='o',c='b',s=6)
            plt.scatter(res['y_amcl'],res['x_amcl'],label='AMCL-Position', marker='o',c='darkorange',s=6)

            plt.plot([p[1],ps[0][1]],[p[0],ps[0][0]],c ='black', label='Pfad',marker='o',markersize=4,linewidth=0.7)
        else:

            plt.scatter(res['y_diff'],res['x_diff'], marker='o',c='b',s=6)
            plt.scatter(res['y_amcl'],res['x_amcl'], marker='o',c='darkorange',s=6)
            plt.plot([p[1],ps[0][1]],[p[0],ps[0][0]],c ='black',marker='o',markersize=4,linewidth=0.7)


            #plt.scatter(res['y_diff'], res['x_diff'], marker='o', c=(res['place']=='floor_5cm'),cmap='PiYG',s=6)
            #plt.plot([p[1], ps[0][1]], [p[0], ps[0][0]], c='black', marker='o', markersize=4,
            #         linewidth=0.5)


        #df.plot('x_diff1','y_diff1',kind='scatter',c='g')
        #df.plot(x='x_diff',y='y_diff',c='b',kind='scatter')
    #Y als Y-aches und umgekehrte, da X vorwärts
    plt.xlim(1.7,-1.2)
    #handles = ['b','darkorange','black']
    #plt.legend(['meas-pose','amcl-pose','path'])
    plt.legend()
    plt.xlabel('Y')
    plt.ylabel('X')
    #plt.figlegend()
    #plt.title("Alle 6 Zielpunkte")
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/AMCL-AbweichungvonFE53_2.png")

    plt.show()

def stdVar(cursor,db_connector):
    query = "SELECT speed,ang_vel,place FROM evaluation GROUP BY speed,ang_vel,place"
    res = pd.read_sql_query(query, db_connector)
    for row in res.itertuples():
        sql = "SELECT sqrt(pow(amcl_x-meas_x,2) + pow(amcl_y-meas_y,2)) as amcl_dev, sqrt(pow(goal_x-meas_x,2)+ pow(goal_y-meas_y,2)) as  goal_dev from evaluation where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        df_f = pd.read_sql_query(sql, db_connector)
        #qmax = df.quantile(0.97)
        #qmin = df.quantile(0.03)
        #df_f = df[(df < qmax) & (df > qmin)]
        dev = df_f.std()
        var =df_f.var()
        mean = df_f.mean()
        print(dev)
        print(dev[0])
        print(dev[1])
        sql = "UPDATE phit SET goal_dev = "+str(round(dev[1],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)
        sql = "UPDATE phit SET amcl_dev = "+ str(round(dev[0],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)

        sql = "UPDATE phit SET goal_var = "+str(round(var[1],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)
        sql = "UPDATE phit SET amcl_var = "+ str(round(var[0],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)
        sql = "UPDATE phit SET goal_mean = "+str(round(mean[1],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)
        sql = "UPDATE phit SET amcl_mean = "+ str(round(mean[0],5)) + " where place=\""+ row.place+"\" AND speed="+ str(row.speed)+ " AND ang_vel=" + str(row.ang_vel)+ " ;"
        cursor.execute(sql)
    db_connector.commit()

def evalDist(cursor,db_connector):
    query = "SELECT sqrt(pow(amcl_x-meas_x,2) + pow(amcl_y-meas_y,2)) as amcl_dist, sqrt(pow(goal_x-meas_x,2)+ pow(goal_y-meas_y,2)) as  goal_dist from evaluation WHERE place like 'E53%' ORDER BY amcl_dist ASC;"
    res = pd.read_sql_query(query,db_connector)
    #df = res["goal_dev"]

    res.plot(x='amcl_dist',y='goal_dist',kind='line')
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/plot-goal-amcl-relation.eps")
    plt.show()
def bpNodes(cursor,db_connector):
    # combs = []
    # for (i in range(len(combs)))
    query = "SELECT goal_nr from evaluation WHERE place LIKE \'E53%\' group by goal_nr  ORDER BY goal_nr ASC;"
    res = pd.read_sql_query(query, db_connector)
    df = pd.DataFrame()
    i = 0
    #xlabels = [""]
    for row in res.itertuples():
        id = "node"+str(row.goal_nr +2)
        query = "SELECT sqrt(pow(meas_x-amcl_x,2)+ pow(meas_y-amcl_y,2)) as \"" + id + "\" FROM evaluation where place LIKE\"E53_%\" AND goal_nr=" + str(
            row.goal_nr) + ";"
        r = pd.read_sql_query(query, db_connector)[id]

        df = pd.concat([df, r], ignore_index=False, axis=1)
        print("nummer: " + str(i) + " " + id)
        #xlabels.append("s="+str(row.speed)+"rv="+str(row.ang_vel))
        i += 1
    #plt.xticks(range(len(xlabels) ), xlabels,rotation=90)
    #,xticks=range(len(xlabels)),xlabel=xlabels
    df.plot(kind='box',rot=0)  # NAN enhalten daher nicht boxplot()
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/amcl_meas_dist_nodes_E53.eps")
    plt.show()


def bpEvaluation(cursor,db_connector):
    # combs = []
    # for (i in range(len(combs)))
    max_len = cursor.execute(
        "SELECT COUNT(*) as m from evaluation group by place,speed,ang_vel ORDER BY m DESC LIMIT 1;").fetchone()
    print(max_len[0]) #WHERE place = \'E53_5cm\'
    query = "SELECT place,speed, ang_vel from evaluation WHERE place = \"E53_5cm\" group by place,speed,ang_vel  ORDER BY speed,ang_vel,place;"
    res = pd.read_sql_query(query, db_connector)
    df = pd.DataFrame()
    i = 0
    xlabels = [""]
    for row in res.itertuples():
        id = "v"+str(row.speed)+"r"+str(row.ang_vel)# +"p"+str(row.place)#row.place + str(row.speed) + str(row.ang_vel)
        query = "SELECT sqrt(pow(meas_x-goal_x,2)+ pow(meas_y-goal_y,2)) as \"" + id + "\" FROM evaluation where place=\"" + row.place + "\" AND speed=" + str(
            row.speed) + " AND ang_vel=" + str(row.ang_vel) + " AND place=\'" + str(row.place) + "\';"
        r = pd.read_sql_query(query, db_connector)[id]

        df = pd.concat([df, r], ignore_index=False, axis=1)
        print("nummer: " + str(i) + " " + id)
        xlabels.append("s="+str(row.speed)+"rv="+str(row.ang_vel))
        i += 1
    #plt.xticks(range(len(xlabels) ), xlabels,rotation=25)
    #,xticks=range(len(xlabels)),xlabel=xlabels
    #df.plot(kind='box',rot=25)  # NAN enhalten daher nicht boxplot()
    #plt.show()
    df.plot(kind='kde')
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/goal_meas_dist_kde_"+ row.place +".eps")

    plt.show()
def bpAcc(cursor,db_connector):
    #combs = []
    #for (i in range(len(combs)))
    mm = cursor.execute('''
    SELECT max(m) as mmax ,min(m) as mmin FROM (
    SELECT x_meas - 1.327 as m FROM acc
    UNION SELECT y_meas-0.284 as m FROM acc
    UNION SELECT  x_odom+0.327 - x_meas as m FROM acc
    UNION SELECT y_odom + 0.284 - y_meas as m FROM acc)
    ;
    ''').fetchone()
    print(mm)
    #(x_meas -0.327),(y_meas-0.284)
    query = "SELECT speed from evaluation group by speed;"
    res = pd.read_sql_query(query, db_connector)
    df_x = pd.DataFrame()
    df_y = pd.DataFrame()
    i =0
    bot,top = plt.ylim()

    for row in res.itertuples():
        plt.ylim(mm[1], mm[0])
        l = np.flip(np.arange(mm[1]-0.005, mm[0] + 0.005, step=0.02))
        plt.yticks(l)
        id = str(row.speed)
        query = "SELECT x_meas - 1.327 as \"proc_x_"+ id +  "\", y_meas-0.284 as \"proc_y_"+id+ "\",x_odom+0.327 - x_meas as \"odom_x_" + id +"\", y_odom + 0.284 - y_meas as \"odom_y_" + id + "\"FROM acc where speed=" + str(row.speed) +  ";"
        r = pd.read_sql_query(query,db_connector)
        label = ['', 'proc_x', 'proc_y', 'odom_x', 'odom_y']
        #plt.title("X,Y-Messungen für Geschwindigkeit " + str(row.speed) + "m/s")
        locs, labels = plt.xticks()

        plt.boxplot(r)
        plt.xticks(range(len(label)), label)

        print("nummer: " + str(i) +" "+ id)
        i += 1
        id = id.replace('.',"_")
        plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/XY-Abweichungen" + id + ".eps",format='eps')
        plt.show()


    #plt.boxplot(df_x)
    #zu wenig punkte, keine klare Linie erkennbar

def corGoalTp(db_connector):
    #phit.amcl_mean as amcl_mean, phit.amcl_dev as amcl_dev,
    #SUM(CASE WHEN sqrt(power(meas_x-goal_x,2)+power(goal_y-meas_y,2)) < 0.07 and sqrt(power(amcl_x-goal_x,2)+power(amcl_y-meas_y,2)) < 0.07 THEN 1 ELSE 0 END)*1.0/Count(*) as TP,Count(*) as count_points,floor(AVG(cpu)) as  where place like 'E53%' GROUP BY iteration
    query = "SELECT  phit.goal_dev as goal_dev, phit.goal_mean as goal_mean, phit.TP as TP  FROM phit JOIN eval_compressed as ev ON ev.place = phit.place and ev.speed=phit.speed  and ev.ang_vel=phit.ang_vel   where ev.place like 'E53%' ORDER BY phit.goal_mean ASC;"
    res = pd.read_sql_query(query, db_connector)
    #fig, ax = plt.subplots()
    #ax.set_prop_cycle(color=['red', 'blue'])
    #df_plot = res.set_index('goal_mean')
    #df_plot.plot()
    res.plot('goal_mean','TP',c='red')
    res.plot('goal_mean','goal_dev',c='blue')

    #df_plot.plot(figsize=(10,6), xticks=range(0, 8)).legend(title='age', bbox_to_anchor=(1, 1))
    #plot(x='cpu',y='dist',kind='scatter',title="alle punkte speed05dist E53")
    #plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/corr_goal_tp2.eps")
    plt.show()

def corCpuTp(cursor,db_connector):
    query = "SELECT speed FROM eval_compressed  GROUP BY speed ORDER BY speed ASC"
    ressp = pd.read_sql_query(query, db_connector)
    nodes = [1,2,3,16,17,5,4,6,8,0,18,0,10,9,11,12,14,13,15]
    color = ['darkgreen','teal','darkblue','darkviolet']
    c = 0
    for row in ressp.itertuples():
        #        ev.place = \'" + row.place+ "\'  and and ev.speed= " + str(row.speed) +
        query = "SELECT phit.place,phit.speed, phit.ang_vel, phit.TP as TP,ev.cpu as cpu FROM phit JOIN eval_compressed as ev ON ev.place = phit.place and ev.speed=phit.speed  and ev.ang_vel=phit.ang_vel WHERE ev.speed= " + str(row.speed) + " and ev.place like \'E53%\' ORDER BY ev.cpu ASC;"
        #print(query)
        col = color.pop(0)

        res = pd.read_sql_query(query,db_connector)
        #print(np.array(res['cpu'].values))
        #print(np.array(res['TP'].values))
        c += res.shape[0]
        print(res)
        if (res.shape[0] > 1):
            plt.plot(res['cpu'].values,res['TP'].values, color=col, marker='o', fillstyle='full',mec=col,mfc='w', linestyle='-',linewidth=2, markersize=15)
            for z in res.itertuples():
                i = nodes.pop(0)
                if (i> 0):
                    if (i>=10):
                        plt.text(z.cpu-0.59, z.TP-0.009, str(i), weight='semibold',color=col, fontsize=10)
                    else:
                        plt.text(z.cpu-0.3, z.TP-0.009, str(i), weight='semibold',color=col, fontsize=10)


        query = "SELECT phit.place,phit.speed, phit.ang_vel, phit.TP as TP,ev.cpu as cpu FROM phit JOIN eval_compressed as ev ON ev.place = phit.place and ev.speed=phit.speed  and ev.ang_vel=phit.ang_vel WHERE ev.speed= " + str(row.speed) + " and ev.place like \'floor%\' ORDER BY ev.cpu ASC;"
        res1 = pd.read_sql_query(query, db_connector)
        # print(np.array(res['cpu'].values))
        # print(np.array(res['TP'].values))
        if (res1.shape[0] > 1):
            plt.plot(res1['cpu'].values, res1['TP'].values, color=col, marker='o', fillstyle='full',mec=col, mfc='w',linestyle='-', linewidth=2,
                     markersize=16)
            for x, y in zip(res1['cpu'].values, res1['TP'].values):
                i = nodes.pop(0)
                if (i > 0):
                    plt.text(x-0.59, y-0.009, str(i), weight='semibold',color=col, fontsize=10)

    print(c)
    plt.ylabel('TP')
    plt.xlabel('cpu')
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/plot-cpu-tp-relationALL.eps")

    plt.show()

def bpYaw(cursor,db_connector):
    #combs = []
    #for (i in range(len(combs)))
    mm = cursor.execute('''
    SELECT max(m) as mmax ,min(m) as mmin FROM (
    SELECT meas_yaw/yaw -1 as m FROM yaw
    UNION SELECT odom_yaw/meas_yaw -1 as m FROM yaw
    UNION SELECT  imu_yaw/meas_yaw -1 as m FROM yaw
    UNION SELECT imu_int_yaw/meas_yaw -1 as m FROM yaw WHERE ROWID != 166)
    ;
    ''').fetchone()
    print(mm)

    #(x_meas -0.327),(y_meas-0.284)
    query = "SELECT ang_speed,yaw from yaw group by ang_speed,abs(yaw) ORDER BY ang_speed ASC;"
    res = pd.read_sql_query(query, db_connector)
    df = pd.DataFrame()
    i =0
    bot,top = plt.ylim()
    for row in res.itertuples():
        plt.ylim(round(mm[1],1) -0.02, round(mm[0],1) +0.02)
        l = np.flip(np.arange(round(mm[1]-0.02,1), round(mm[0]+ 0.02,1) , step=0.1))
        plt.yticks(l)
        id = str(row.ang_speed) + ":" + str(row.yaw)
        #1 punkt ausgelassen
        query = "SELECT meas_yaw/yaw -1 as \"proc_" + str(id)+ "\", odom_yaw/meas_yaw -1 as \"odom_"+id+ "\", imu_yaw/meas_yaw -1 as \"imu_"+id+ "\", imu_int_yaw/meas_yaw -1 as \"imu_int_"+id+ "\" FROM yaw where ang_speed=" + str(row.ang_speed) +  " AND imu_int_yaw != -0.488216 AND (yaw = "+ str(row.yaw) + " or yaw = " + str(-row.yaw) + ") ;"
        r = pd.read_sql_query(query,db_connector)
        label = ['', 'proc','odom','imu','imu_int']
        plt.title("Winkelmessungen für Rotationsgeschwindigkeit " + id)
        locs, labels = plt.xticks()

        plt.boxplot(r)
        plt.xticks(range(len(label)),label)
        #df_x = pd.concat([df_x,r["x_" + id]],axis=1)

        print("nummer: " + str(i) +" "+ id)
        i += 1
        plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/Yaw_sep" + id + ".eps",format='eps')
        plt.show()

    #plt.boxplot(df_x)
    #plt.boxplot(df_y)

    #ax.imshow(I)
    '''
    labels = [0,0.2,0.4,0.6,0.8]
    #x_labels += list(df_x.columns.values)
    df_x.plot(kind='box',title="X-Abweichung").set_xticks(ticks = range(len(labels)),labels=labels) # NAN enhalten daher nicht boxplot()

    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/X-Abweichung.png")
    df_y.plot(kind='box',title="Y-Abweichung").set_xticks(ticks = range(len(labels)),labels=labels) # NAN enhalten daher nicht boxplot()
    plt.savefig("/home/kotname/ros_ws/src/Beleg/localization/data_eval/sqlitebrowser_Bilder/Y-Abweichung.png")
    '''


if __name__=="__main__":
    db_connector = sqlite3.connect(dbfile)
    cursor = db_connector.cursor()
    plotAMCLDist2(db_connector)
    #corGoalTp(db_connector)
    #corCpuTp(cursor,db_connector)
    #evalDist(cursor,db_connector)
    #stdVar(cursor,db_connector)
    #evalHit(cursor,db_connector)
    #bpNodes(cursor,db_connector)
    #bpEvaluation(cursor,db_connector)
    #bpAcc(cursor,db_connector)
    #plt.boxplot(filtered_data)
    #bpYaw(cursor,db_connector)
    #plt.show()
    db_connector.commit()
    db_connector.close()

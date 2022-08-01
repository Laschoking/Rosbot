#include <stdio.h>
#include <sqlite3.h>



int main(int argc,char **argv) {
    sqlite3* db ;
    db_file = "~/ros_ws/src/Beleg/Messungen.sqlite";
    cursor = sqlite3_open(db_file, &db);
    sql = "SELECT FROM yaw "


    sqlite3_close(db);


}

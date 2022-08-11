#ifndef ROS_WS_SQLITE_LIB_H
#define ROS_WS_SQLITE_LIB_H
#include <sqlite3.h>
#include <string>

double getSQLiteOut(sqlite3* db, std::string* sql);
int insertSQLite(sqlite3* db, std::string* sql);



#endif
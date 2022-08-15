#include "sqlite_lib.h"
#include <iostream>


double getSQLiteOut(sqlite3* db, std::string* sql){
    sqlite3_stmt* stmt;
    double res;
    if(sqlite3_prepare_v2(db,sql->c_str(),-1, &stmt, NULL) != SQLITE_OK){
        printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return 0;
    }
    while(sqlite3_step(stmt) == SQLITE_ROW){
        res = (double) sqlite3_column_double(stmt,0);
    }
    sqlite3_finalize(stmt);
    return res;
}

int insertSQLite(sqlite3* db, std::string* sql){
    sqlite3_stmt* stmt;
    if(sqlite3_prepare_v2(db,sql->c_str(),-1, &stmt, NULL) != SQLITE_OK){
        printf("ERROR: while executing sql: %s\n", sqlite3_errmsg(db));
        std::cout << *sql << "\n";
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return 0;
    }else{
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        return 1;
    }
}
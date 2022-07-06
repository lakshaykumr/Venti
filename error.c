#include <syslog.h>
#include <stdio.h>
#include "error.h"

int syslog_function(char *error_no)                                      // Print your Messages in system log
{
        openlog("LOG" , LOG_CONS ,LOG_USER);                            // open a connection to system logger
        syslog(LOG_DEBUG ,error_no);                                    // genrated a log message
        closelog();                                                     // close connection
        return 0;
}

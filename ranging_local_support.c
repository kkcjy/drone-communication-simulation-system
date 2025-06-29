#include "ranging_local_support.h"


void DEBUG_PRINT(const char *format, ...) {
    static bool first_call = true;
    va_list args;

    // print to console 
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    
    // print to file
    FILE *log_file = first_call ? fopen("data/dataLog.txt", "w") : fopen("data/dataLog.txt", "a");
    
    if (log_file != NULL) {
        va_start(args, format);  
        vfprintf(log_file, format, args);
        va_end(args);
        fclose(log_file);
        
        first_call = false;

    } else {
        printf("Warning: Could not open dataLog.txt for writing\n");
    }
}
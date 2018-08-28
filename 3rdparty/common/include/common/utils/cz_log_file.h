#ifndef CZ_COMMON_LOG_FILE_H
#define CZ_COMMON_LOG_FILE_H
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "common/utils/cz_def.h"

namespace cz {

class CZ_EXPORTS LogFile {
public:
    ~LogFile();
    
    static void init(int argc, char** argv);
    static void init(std::string path = "");
    static LogFile& getInstance();
    template <class T>
	inline LogFile& operator << (const T& t) {
		if (getInstance()._out_log.is_open()) {
			getInstance()._out_log << getTimeStr() << t << std::endl;
        }
		return getInstance();
    }
    void Separator(const std::string& info = "SEPARATOR", const char& ch = '*', int num = 20) {
		if (getInstance()._out_log.is_open()) {
            std::string bef; bef.resize(num, ch);
		    std::string end; end.resize(num, ch);
		    getInstance()._out_log << getTimeStr() << bef << info << end << std::endl;
        }
	}
private:
    LogFile();
    std::string getTimeStr();
private:
    std::ofstream _out_log;
};

#define cLog LogFile::getInstance()
}
#endif 
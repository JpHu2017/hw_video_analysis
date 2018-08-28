#ifndef CZ_COMMON_LOG_STREAM_H
#define CZ_COMMON_LOG_STREAM_H
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <ctime>
//#include "direct.h"
#ifdef WIN_VS
#include <windows.h>
//#include <io.h>
//#include <direct.h> 

#else 
#include <unistd.h>
//#include <sys/types.h>  
//#include <sys/stat.h>
#endif

namespace cz {
class LogStream {
public:
	enum LogType { INFO_LOG, WARNING_LOG, ERROR_LOG, DEBUG_LOG };
#ifndef WIN_VS
    const std::string g_RESET   = "\033[0m" ;
    const std::string g_BLACK   = "\033[30m";      /* Black */
    const std::string g_RED     = "\033[31m";      /* Red */
    const std::string g_GREEN   = "\033[32m";      /* Green */
    const std::string g_YELLOW  = "\033[33m";      /* Yellow */
    const std::string g_BLUE    = "\033[34m";      /* Blue */
    const std::string g_MAGENTA = "\033[35m";      /* Magenta */
    const std::string g_CYAN    = "\033[36m";      /* Cyan */
    const std::string g_WHITE   = "\033[37m";      /* White */
    const std::string g_BOLDBLACK   = "\033[1m\033[30m";      /* Bold Black */
    const std::string g_BOLDRED     = "\033[1m\033[31m";      /* Bold Red */
    const std::string g_BOLDGREEN   = "\033[1m\033[32m";      /* Bold Green */
    const std::string g_BOLDYELLOW  = "\033[1m\033[33m";      /* Bold Yellow */
    const std::string g_BOLDBLUE    = "\033[1m\033[34m";      /* Bold Blue */
    const std::string g_BOLDMAGENTA = "\033[1m\033[35m";      /* Bold Magenta */
    const std::string g_BOLDCYAN    = "\033[1m\033[36m";      /* Bold Cyan */
    const std::string g_BOLDWHITE   = "\033[1m\033[37m";      /* Bold White */
#endif
    explicit LogStream(LogType type) :
        _type(type), _debug_str("") {

    }
    LogStream(LogType type, const char *file, const char *func, int line) : _type(type) {
        _debug_str = "Function: " + std::string(func) + " in file: "
                   + std::string(file) + " at line: " + std::to_string(line);
    }
    ~LogStream() {
        sendMsg();
    }
    inline LogStream &space() {_ss << ' '; return *this;}

    template <class T>
    inline LogStream &operator<<(const std::vector<T>& ts) {
        for (int i = 0; i < ts.size(); ++i) {
            _ss << ts[i] << " ";
        }
        return space();
    }
    template <class T>
    inline LogStream&operator <<(const T& t) {_ss << t; return space();}

	template <class T>
	inline LogStream& operator ()(const T t) {_ss << t; return space(); }

    template <class T1, class T2>
	inline LogStream& operator ()(const T1& key, const T2& value) { _ss << key << ": " << value; return space(); }

	void Separator(const std::string& info = "SEPARATOR", const char& ch = '*', int num = 20) {
		std::string bef; bef.resize(num, ch);
		std::string end; end.resize(num, ch);
		_ss << bef << info << end;
	}
private:
    void sendMsg() {
        switch(_type) {
		case INFO_LOG:
		{
#ifdef WIN_VS
			HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
			SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
			std::cout << _ss.str();
			SetConsoleTextAttribute(hStdout, FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
			std::cout << g_BOLDGREEN << _ss.str() << g_RESET;
#endif
			break;
		}
		case WARNING_LOG:
		{
#ifdef WIN_VS
			HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
			SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
			std::cout << _ss.str();
			SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
			std::cout << g_BOLDYELLOW << _ss.str() << g_RESET;
#endif
			break;
		}
		case ERROR_LOG:
		{
#ifdef WIN_VS
			HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
			SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_INTENSITY);
			std::cout << _ss.str();
			SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
			std::cout << g_BOLDRED << _ss.str() << g_RESET;
#endif
			break;
		}
		case DEBUG_LOG:
		{
#ifdef DEBUG
#ifdef WIN_VS
			HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
			SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
			_ss << "; " << _debug_str;
			std::cout << _ss.str();
			SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
			_ss << "; " << _debug_str;
			std::cout << g_BOLDGREEN << _ss.str() << g_RESET;
#endif
#endif
			break;
		}
        default:
            return;
        }
        std::cout << std::endl;
    }

private:
    LogType _type;
    std::string _debug_str;
    std::stringstream _ss;
};

#define cInfo  (LogStream(LogStream::INFO_LOG))
#define cWarn  (LogStream(LogStream::WARNING_LOG))
#define cErr   (LogStream(LogStream::ERROR_LOG))
#define cDebug (LogStream(LogStream::DEBUG_LOG, __FILE__, __FUNCTION__, __LINE__))

}

#endif //

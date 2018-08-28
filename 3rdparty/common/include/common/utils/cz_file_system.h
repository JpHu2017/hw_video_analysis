#ifndef CZ_UTILS_FILE_SYSTEM_H
#define CZ_UTILS_FILE_SYSTEM_H
#include <string>
#include <vector>
#include "common/utils/cz_def.h"

namespace cz {

class CZ_EXPORTS FileSystem {
public:
    FileSystem();
    ~FileSystem();
public:
	static bool isPath(const std::string& pathname);
    static bool isExsit(const std::string& filename);
	static std::string getCurrentPath();

    //filename = c:/data/filename.txt 
	static std::string getSuffix(const std::string& filename); // return txt 
	static std::string getFilePath(const std::string& filename); // return c:/data
    static std::string getFileName(const std::string& filename); // return filename
	static std::string getAbsFileName(const std::string& filename); // return filename.txt

	static bool mkdir(const std::string& path, const std::string& folder);
	static bool mkdir(const std::string& pathname);
	static bool rmdir(const std::string& pathname);
	static bool rmdirAll(const std::string& pathname);

	static std::vector<std::string> findFiles(const std::string& pathname, const std::string& sub_str, bool sort_up = true); // if sub_str = ""; then find all
    static std::vector<std::string> findFiles(const std::string& pathname, const std::string& sub_str, const std::string& exclude_str, bool sort_up = true);
    static std::vector<std::string> findFiles(const std::string& pathname, const std::vector<std::string>& sub_str_vec, const std::vector<std::string>& exclude_str_vec = std::vector<std::string>(), bool sort_up = true);
};

}

#endif

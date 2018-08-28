#ifndef __CZ_COMMON_UTILS_H_
#define __CZ_COMMON_UTILS_H_
#include <string>
#include <common/utils/cz_def.h>
namespace cz {
class CZ_EXPORTS String {
public:
    String() {}
    ~String() {}

    static std::string toUpper(const std::string& str);
    static std::string toLower(const std::string& str);
};
}

#endif
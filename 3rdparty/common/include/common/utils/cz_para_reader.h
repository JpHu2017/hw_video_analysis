#ifndef _CZ_COMMON_PARAREADER_H
#define _CZ_COMMON_PARAREADER_H
#include "common/utils/cz_def.h"
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace cz {
    
class ParaReaderPrivate;
class CZ_EXPORTS ParaReader {
public:
    typedef boost::shared_ptr<ParaReader> Ptr;
    explicit ParaReader(const std::string para_file = "", bool case_sensitive = false);
    ParaReader(int argc, char** argv);
    ~ParaReader();
    bool loadParam(const std::string para_file, bool append = false);
    void setKeyCaseSensitive(bool case_sensitive);

    bool has(std::string key) const;

    template <class T>
    bool getValueVec(std::string key, std::vector<T>& values) {
        if (!has(key)) {return false;}
        values.clear();
        std::stringstream ss (this->getStringValue(key));
        T v;
        while (ss >> v) {values.push_back(v);}
        return true;
    }
    template <class T>
    bool getValue(std::string key, T& value) {
        if (!has(key)) {return false;}
        std::stringstream ss (this->getStringValue(key));
        ss >> value;
        return true;
    }
private:
    std::string getStringValue(std::string key);
private:
    ParaReaderPrivate* _ptr;    
};

}


#endif
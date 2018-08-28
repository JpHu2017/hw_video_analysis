/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-6-11
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function
*/
#include <iostream>
#include <string>
#include <vector>

#ifndef CZ_TOOLS_UTIL_FUNCS_H
#define CZ_TOOLS_UTIL_FUNCS_H
namespace cz {
namespace lt {

enum TrimMode {Ends, All}; //两端，全部
// 字符串去除空格(首尾两段，或者全部)
std::string trim(const std::string &s, const TrimMode& mode = Ends);
// 按照特定子字符串，进行字符分割
void split(const std::string& s,const std::string& delim, std::vector<std::string>& ret);
// 读取文件夹下所有文件（无递归吗，仅返回文件名）
std::vector<std::string> readFileList(const std::string& base_path);
// 递归创建文件夹
void createFolder(const std::string& dir);
}
}


#endif //CZ_TOOLS_UTIL_FUNCS_H

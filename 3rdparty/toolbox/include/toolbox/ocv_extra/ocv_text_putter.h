/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-5-31
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function
*/
#include <memory>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifndef CZ_TOOLS_OCV_TEXT_PUTTER_H
#define CZ_TOOLS_OCV_TEXT_PUTTER_H
namespace cz {
namespace lt {

class TextPutterPrivate;
class TextPutter {
public:
	std::shared_ptr<TextPutter> SharedPtr;
	TextPutter();
	// 装载字库文件
	TextPutter(const std::string& font_type, const int& font_scale = 15);
	virtual ~TextPutter();
	void setFont(const std::string& font_type = "",const int& font_scale = 15);
	void putText(cv::Mat &frame, const std::string& text, const cv::Point& pos,
				 const cv::Scalar& color, const int& thickness = 2);
private:
	std::shared_ptr<TextPutterPrivate> _private_ptr;
};
}
}
#endif //CZ_TOOLS_OCV_TEXT_PUTTER_H
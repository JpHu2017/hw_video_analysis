/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-20
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#ifndef HUNGARIANALG_H
#define HUNGARIANALG_H

#include <memory>
#include <vector>
namespace cz {
namespace ht {
class AssignmentProblemSolver;
// 单例模式，在实现中考虑到线程安全，详情见cpp
class Hungarian {
public:
	/* 采用引用，而不采用指针方式返回的好处是
	 * 既能实现线程安全的单例模式，同时将boost::thread_specific_ptr放置在cpp中
	 * 似的在头文件中无需引入boost头文件，当此模块作为第三方库提供时，头文件更为简练
	 * */
	static Hungarian& createInstance(); //创建实例
	~Hungarian();
	double solve(const std::vector<std::vector<double> >& cost_matrix,std::vector<std::pair<int,int> >& assignment);
private:
	Hungarian();
	std::shared_ptr<AssignmentProblemSolver> _solver_ptr;
};
}
}

#endif //HUNGARIANALG_H
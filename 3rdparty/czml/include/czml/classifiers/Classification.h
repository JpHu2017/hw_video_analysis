#ifndef _CZ_ML_Classification_H
#define _CZ_ML_Classification_H
#include <common/core.h>
#include <opencv2/opencv.hpp>

namespace cz {
namespace ml {

struct classPrediction {
	classPrediction() {}
    float score;
    std::string type;
    int task_id;
	classPrediction(std::string para_type, float para_score, int para_task_id = 0)
            : score(para_score), type(para_type), task_id(para_task_id){}
};
//typedef std::vector<classPrediction> ScoreVec;
//typedef std::vector<std::vector<classPrediction> > ScoreVecs; // dim:3 - img,task,result

class CZ_EXPORTS Classification {
public:
    Classification(int top_n = 1, int tasks = 1);
    ~Classification();


	bool isGood() { return !_predictions_vec.empty(); }

    std::vector<classPrediction>& getPredictions() {return _predictions_vec;}
    const std::vector<classPrediction>& getPredictions() const {return _predictions_vec;}

    std::vector<std::vector<classPrediction> > getMultiTasksPredictions();
    const std::vector<std::vector<classPrediction> > getMultiTasksPredictions() const;

    void setTasks(const int tasks) {_tasks = tasks;}
    int getTasks() {return _tasks;}
    void setTopN(const int n) {_top_n = n;}
    int getTopN() {return _top_n;}

private:
    std::vector<classPrediction> _predictions_vec;
    int _tasks;
    int _top_n;
};
}
}
#endif
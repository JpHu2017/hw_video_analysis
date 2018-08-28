#ifndef __CZ_ML_MODEL_PARAMETER_H_
#define __CZ_ML_MODEL_PARAMETER_H_
#include <common/core.h>

namespace cz {
namespace ml {
class ModelParameterPrivate;
class CZ_EXPORTS ModelParameter {
public:
    ModelParameter();
    virtual ~ModelParameter();

    void setParam(std::string net_config, std::string net_weights, std::string class_file,
                  bool use_gpu = false, int gpu_id = 0);
    void setParam(const std::string& pb_path, bool use_gpu = false, int gpu_id = 0);
    bool loadModel(const std::string& filename);
    static void resetComputeMode(bool use_gpu = false, int gpu_id = 0);

    std::string getNetConfig() const;
    std::string getNetWeights() const;
    std::string getClassFile() const;
    std::string getPbPath() const;
    bool useGPU() const;
    int gpuID() const;
    
    bool getClassLabels(std::vector<std::string>& class_labels) const;
 private:
    ModelParameterPrivate *_ptr;
};
}
}


#endif
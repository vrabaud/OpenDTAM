#include "Cost.h"

#define COST_CPP_SUBPARTS
#include "updateCost.part.cpp"
#include "min.part.cpp"
#undef COST_CPP_SUBPARTS
#include "updateCost.part.hpp"
#define COST_CPP_DATA_MIN .1


Cost::Cost(const cv::Mat& baseImage, int layers, const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr):
baseImage(baseImage),
rows(baseImage.rows),
cols(baseImage.cols),
layers(layers),
depth(generateDepths(layers)),
cameraMatrix(cameraMatrix),
pose(convertPose(R,Tr))
{
    data = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*layers,1);
    std::cout << "coin1 " << int(data.empty()) << std::endl;
    hit = cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*layers,1);

    init();
}


Cost::Cost(const cv::Mat& baseImage, int layers, const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose):
baseImage(baseImage),
rows(baseImage.rows),
cols(baseImage.cols),
layers(layers),
depth(generateDepths(layers)),
cameraMatrix(cameraMatrix),
pose(cameraPose)
{
    data = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*layers,1);
    std::cout << "coin2 " << int(data.empty()) << std::endl;
    hit = cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*layers,1);

    init();
}


Cost::Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr):
baseImage(baseImage),
rows(baseImage.rows),
cols(baseImage.cols),
depth(depth),
layers(depth.size()),
cameraMatrix(cameraMatrix),
pose(convertPose(R,Tr))
{
    data = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*depth.size(),1);
    std::cout << "coin3 " << int(data.empty()) << std::endl;
    hit = cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*depth.size(),1);

    init();
}


Cost::Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose):
baseImage(baseImage),
rows(baseImage.rows),
cols(baseImage.cols),
layers(depth.size()),
depth(depth),
cameraMatrix(cameraMatrix),
pose(cameraPose)
{
    data = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*depth.size(),1);
    std::cout << "coin4 " << int(data.empty()) << std::endl;
    hit = cv::Mat_<float>::ones(baseImage.rows*baseImage.cols*depth.size(),1);

    init();
}

const cv::Mat Cost::depthMap(){
    //Returns the best available depth map
    // Code should not rely on the particular mapping of true 
    // internal data to true inverse depth, as this may change.
    // Currently depth is just a constant multiple of the index, so
    // infinite depth is always represented. This is likely to change.
    if(stableDepth.data){
        exit(0);
        return stableDepth*depthStep;
    }
    return _d*depthStep;
}



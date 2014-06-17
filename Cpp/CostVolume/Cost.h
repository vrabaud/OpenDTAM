#ifndef COST_H
#define COST_H
#include <opencv2/core/core.hpp>
#include <vector>
// The cost volume. Conceptually arranged as an image plane, corresponding
// to the keyframe, lying on top of the actual cost volume, a 3D two channel matrix storing
// the total cost of all rays that have passed through a voxel, and the number of rays that
// have hit that voxel.
//
// There is also the depth array, which stores the inverse depths of each plane of the cost.
//
// For efficiency, the accumulated cost and hit count are seperate arrays even though
// conceptually they are just different channels of the same image.
//
// The cost volume doesn't support updating by a different camera than the one that took the
// keyframe, because that would violate a bunch of assumptions for DTAM
#define COST_H_DEFAULT_NEAR .01
#define COST_CPP_DATA_MIN .1

class Cost{
public:
    cv::Mat rayHits;// number of times a ray has been hit(not implemented)
    cv::Mat_<cv::Vec3f> baseImage;
    cv::Mat_<float> lo;
    cv::Mat_<float> hi;
    const int rows;
    int cols;
    int layers;
    std::vector<float> depth;
    float depthStep;
    cv::Matx33d cameraMatrix;
    cv::Matx44d pose;//the affine transform representing the world -> camera frame transformation
    cv::Mat_<float> data;// stores the [rows][cols][layers] array of sum of costs so far
    cv::Mat_<float> hit;//stores the number of times each cell has been hit by a ray
    int imageNum;

    //Cost(){};
    Cost(const cv::Mat& baseImage, int layers,                  const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr);// autogenerate default depths
    Cost(const cv::Mat& baseImage, int layers,                  const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose);// autogenerate default depths
    Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Mat& R, const cv::Mat& Tr);//use given depths
    Cost(const cv::Mat& baseImage, const std::vector<float>& depth, const cv::Mat& cameraMatrix, const cv::Matx44d& cameraPose);//use given depths

    void updateCostL1(const cv::Mat& image, const cv::Matx44d& currentCameraPose);
    void updateCostL1(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr);
    void updateCostL2(const cv::Mat& image, const cv::Matx44d& currentCameraPose);
    void updateCostL2(const cv::Mat& image, const cv::Mat& R, const cv::Mat& Tr);
    void optimize();
    void initOptimization();
    
    const cv::Mat depthMap(); //return the best available depth map

    const cv::Matx44d convertPose(const cv::Mat& R, const cv::Mat& Tr){
        cv::Mat pose=cv::Mat::eye(4,4, CV_64F);
        R.copyTo(pose(cv::Range(0,3),cv::Range(0,3)));
        Tr.copyTo(pose(cv::Range(0,3),cv::Range(3,4)));
        return cv::Matx44d(pose);
    }


private:
    //Initializer functions
    void init(){
        depthStep=((depth.back()-depth[0])/layers);
        lo = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows,baseImage.cols);
        hi = COST_CPP_DATA_MIN*cv::Mat_<float>::ones(baseImage.rows,baseImage.cols);
        imageNum=0;
        QDruncount=0;
        Aruncount=0;
        thetaStart=1000.0;
        thetaMin=0.01;
        running=false;
        initOptimization();
    }
    std::vector<float> generateDepths(int layers){
        std::vector<float> depths;
        for(float n=0; n<layers; n++){
                depths.push_back(n/(layers-1)*COST_H_DEFAULT_NEAR);
            }
        return depths;// generate the depth list, the signature will probably change
    }

    
    //Utility Functions
    void minv(uchar*/*(float*)*/,cv::Mat& minIndex,cv::Mat& minValue);
    void minv(float*            ,cv::Mat& minIndex,cv::Mat& minValue);
    void maxv(float*/*(float*)*/,cv::Mat& maxIndex,cv::Mat& maxValue);   
    void minmax();    

    //Optimizer functions and data
    public:cv::Mat _qx,_qy,_d,_a,_g,_gu,_gd,_gl,_gr,_gbig;private:
    cv::Mat stableDepth;
    float theta,thetaStart,thetaMin,epsilon,lambda,sigma_d,sigma_q;
    
    void computeSigmas();
    void cacheGValues();
    
        //Q update
    public: void optimizeQD();private://NOT PUBLIC!!! JUST NEED TO ACCESS FROM A STATIC CALL. DO NOT USE!
        //A update
    float aBasic(float* data,float l,float ds,float d);
    public: void optimizeA();private://NOT PUBLIC!!! JUST NEED TO ACCESS FROM A STATIC CALL. DO NOT USE!
    
    
    //Instrumentation
    int QDruncount;
    int Aruncount;
    
    //Thread management
    bool running;
};


#endif

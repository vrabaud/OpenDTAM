// Free for non-commercial, non-military, and non-critical
// use unless incorporated in OpenCV.
// Inherits OpenCV Licence if in OpenCV.

//This file does Q and D optimization steps on the GPU
#include "Optimizer.hpp"
#include "Optimizer.cuh"
#include <opencv2/gpu/stream_accessor.hpp>
#include <iostream>

using namespace std;
void Optimizer::computeSigmas(){
    //This function is my best guess of what was meant by the line:
    //"Gradient ascent/descent time-steps sigma_q , sigma_d are set optimally
    //for the update scheme provided as detailed in [3]."
    // Where [3] is :
    //A. Chambolle and T. Pock. A first-order primal-dual 
    //algorithm for convex problems with applications to imaging.
    //Journal of Mathematical Imaging and Vision, 40(1):120–
    //145, 2011. 3, 4, 6
    //
    // I converted these mechanically to the best of my ability, but no 
    // explaination is given in [3] as to how they came up with these, just 
    // some proofs beyond my ability.
    //
    // Explainations below are speculation, but I think good ones:
    //
    // L appears to be a bound on the largest vector length that can be 
    // produced by the linear operator from a unit vector. In this case the 
    // linear operator is the differentiation matrix with G weighting 
    // (written AG in the DTAM paper,(but I use GA because we want to weight 
    // the springs rather than the pixels)). Since G has each row sum < 1 and 
    // A is a forward difference matrix (which has each input affecting at most
    // 2 outputs via pairs of +-1 weights) the value is bounded by 4.0.
    //
    // So in a sense, L is an upper bound on the magnification of our step size.
    // 
    // Lambda and alpha are called convexity parameters. They come from the 
    // Huber norm and the (d-a)^2 terms. The convexity parameter of a function 
    // is defined as follows: 
    //  Choose a point on the function and construct a parabola of convexity 
    //    c tangent at that point. Call the point c-convex if the parabola is 
    //    above the function at all other points. 
    //  The smallest c such that the function is c-convex everywhere is the 
    //      convexity parameter.
    //  We can think of this as a measure of the bluntest tip that can trace the 
    //     entire function.
    // This is important because any gradient descent step that would not 
    // cause divergence on the tangent parabola is guaranteed not to diverge 
    // on the base function (since the parabola is always higher(i.e. worse)).

    
        
    float lambda, alpha,gamma,delta,mu,rho,sigma;
    float L=4.0;//lower is better(longer steps), but in theory only >=4 is guaranteed to converge
    
    lambda=1.0/theta;
    alpha=epsilon;
    
    gamma=lambda;
    delta=alpha;
    
    mu=2.0*sqrt(gamma*delta)/L;

    rho= mu/(2.0*gamma);
    sigma=mu/(2.0*delta);
    
    sigma_d = rho;
    sigma_q = sigma;
}
#define FLATALLOC(n) n.create(1,cv.rows*cv.cols, CV_32FC1);n=n.reshape(0,cv.rows)
void Optimizer::cacheGValues(){
    using namespace cv::gpu::device::dtam_optimizer;
    localStream = cv::gpu::StreamAccessor::getStream(cvStream);
    if(cachedG)
        return;//already cached
    int layerStep = cv.rows * cv.cols;
    float* d = (float*) _d.data;
    float* a = (float*) _a.data;
    // Call the gpu function for caching g's
    
    loadConstants(cv.rows, cv.cols, cv.layers, layerStep, a, d, cv.data, (float*)cv.lo.data,
            (float*)cv.hi.data, (float*)cv.loInd.data);
    assert(_g1.isContinuous());
    float* pp = (float*) cv.baseImageGray.data;//TODO: write a color version.
    float* g1p = (float*)_g1.data;
    float* gxp = (float*)_gx.data;
    float* gyp = (float*)_gy.data;
    computeGCaller(pp,  g1p,  gxp,  gyp,  lambda,  cv.cols);
    cachedG=1;
}

bool Optimizer::optimizeQD(){
    using namespace cv::gpu::device::dtam_optimizer;
    localStream = cv::gpu::StreamAccessor::getStream(cvStream);

    bool doneOptimizing = theta <= thetaMin;
    int layerStep = cv.rows * cv.cols;
    float* d = (float*) _d.data;
    float* a = (float*) _a.data;
    float* g1pt = (float*)_g1.data;
    float* gxpt = (float*)_gx.data;
    float* gypt = (float*)_gy.data;
    float* gqxpt = (float*)_qx.data;
    float* gqypt = (float*)_qy.data;

//    loadConstants(cv.rows, cv.cols, cv.layers, layerStep, a, d, cv.data, (float*)cv.lo.data,
//            (float*)cv.hi.data, (float*)cv.loInd.data);
    updateQDCaller  ( gqxpt, gqypt, d, a,
            gxpt, gypt, sigma_q, sigma_d, epsilon, theta);
    return doneOptimizing;
}

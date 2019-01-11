//
// Created by phyorch on 10/12/18.
//

#ifndef PROJECT_STEREOGC_H
#define PROJECT_STEREOGC_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/core/internal.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <vector>

using namespace std;

#undef INFINITY
#define INFINITY 10000
#define OCCLUSION_PENALTY 10000
#define OCCLUSION_PENALTY2 1000
#define DENOMINATOR 16
#undef OCCLUDED
#define OCCLUDED SHRT_MAX
#define CUTOFF 1000
#define IS_BLOCKED(d1, d2) ((d1) > (d2))

typedef struct GCVtx
{
    GCVtx *next;
    int parent;
    int first;
    int ts;
    int dist;
    short weight;
    uchar t;
}
        GCVtx;

typedef struct GCEdge
{
    GCVtx* dst;
    int next;
    int weight;
}
        GCEdge;

typedef struct CvStereoGCState2
{
    int Ithreshold, interactionRadius;
    int lambda, lambda1, lambda2, K;
    int dataCostFuncTab[CUTOFF+1];
    int smoothnessR[CUTOFF*2+1];
    int smoothnessGrayDiff[512];
    GCVtx** orphans;
    int maxOrphans;
}
        CvStereoGCState2;

typedef struct CvStereoGCState
{
    int Ithreshold;
    int interactionRadius;
    float K, lambda, lambda1, lambda2;
    int occlusionCost;
    int minDisparity;
    int numberOfDisparities;
    int maxIters;

    CvMat* left;
    CvMat* right;
    CvMat* dispLeft;
    CvMat* dispRight;
    CvMat* ptrLeft;
    CvMat* ptrRight;
    CvMat* vtxBuf;
    CvMat* edgeBuf;
} CvStereoGCState;


static void icvInitStereoConstTabs();

static void icvInitStereoTabs( CvStereoGCState2* state2 );

static int icvGCResizeOrphansBuf( GCVtx**& orphans, int norphans );

static int64 icvGCMaxFlow( GCVtx* vtx, int nvtx, GCEdge* edges, GCVtx**& _orphans, int& _maxOrphans );

CvStereoGCState* cvCreateStereoGCState( int numberOfDisparities, int maxIters );

void cvReleaseStereoGCState( CvStereoGCState** _state );

int icvDataCostFuncGraySubpix( const uchar* a, const uchar* b );

static int icvSmoothnessCostFunc( int da, int db, int maxR, const int* stabR, int scale );

static void icvInitGraySubpix( const CvMat* left, const CvMat* right, CvMat* left3, CvMat* right3 );

static float icvComputeK( CvStereoGCState* state );

static int64 icvComputeEnergy( const CvStereoGCState* state, const CvStereoGCState2* state2, bool allOccluded );

static void icvAddEdge( GCVtx *x, GCVtx* y, GCEdge* edgeBuf, int nedges, int w, int rw );

static int icvAddTWeights( GCVtx* vtx, int sourceWeight, int sinkWeight );

static int icvAddTerm( GCVtx* x, GCVtx* y, int A, int B, int C, int D, GCEdge* edgeBuf, int& nedges );

static int64 icvAlphaExpand( int64 Eprev, int alpha, CvStereoGCState* state, CvStereoGCState2* state2 );

void cvFindStereoCorrespondenceGC( const CvArr* _left, const CvArr* _right, CvArr* _dispLeft, CvArr* _dispRight, CvStereoGCState* state, int useDisparityGuess );



#endif //PROJECT_STEREOGC_H

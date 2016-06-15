/**
 * File: BinaryIndex.h
 * Date: April 2015
 * Author: Emilio Garcia-Fidalgo 
 * License: see the LICENSE file.
 */

#ifndef BINARYINDEX_H_
#define BINARYINDEX_H_

#include <cfloat>
#include <cmath>
#include <iostream>
#include <map>

#include <flann/flann.hpp>
#include <opencv2/opencv.hpp>

#include "obindex/Timer.h"

namespace obindex
{

typedef flann::Matrix<unsigned char> FMat;

enum MeanPointPolicy
{
    MEANP_POLICY_AND,
    MEANP_POLICY_OR,
};

enum ScoringMethod
{
    SCORING_DIRECT,
    SCORING_TFIDF,
    SCORING_DIST
};

struct BinaryIndexParams
{
    BinaryIndexParams() :
        branching(4),
        trees(4),
        leaf_size(200),
        rebuild_thresh(4),
        desc_bytes(32),
        max_descriptors(3000000),
        meanp_policy(MEANP_POLICY_AND),
        scoring_method(SCORING_TFIDF)
    {}

    int branching;
    int trees;
    int leaf_size;
    float rebuild_thresh;
    int desc_bytes;    
    int max_descriptors;
    MeanPointPolicy meanp_policy;
    ScoringMethod scoring_method;
};

struct InvertedIndexEntry
{
    InvertedIndexEntry() :
        image_id(-1),
        distance(FLT_MAX),
        coords(0.0f, 0.0f),
        orig_feat_id(-1)
    {}
    InvertedIndexEntry(const int image_id, const float distance, const cv::Point2f coords, const int orig_image_id) :
        image_id(image_id),
        distance(distance),
        coords(coords),
        orig_feat_id(orig_image_id)
    {}

    int image_id;
    float distance;
    cv::Point2f coords;
    int orig_feat_id;
};

struct ImageMatch
{
    ImageMatch() :
        image_id(-1),
        score(0.0)
    {}

    ImageMatch(const int _image_id) :
        image_id(_image_id),
        score(0.0)
    {}

    int image_id;
    double score;

    bool operator<(const ImageMatch &lcr) const { return score > lcr.score; }
};

class BinaryIndex
{
public:
    BinaryIndex(const BinaryIndexParams params = BinaryIndexParams());
    virtual ~BinaryIndex();

    void add(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs);
    void update(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs, const std::vector<cv::DMatch>& matches);
    void addToInvertedIndex(const int image_id, const std::vector<cv::KeyPoint>& kps, const std::vector<cv::DMatch>& matches);
    double search(const cv::Mat& qdescs, std::vector<std::vector<cv::DMatch> >& matches, const int knn = 2);
    double getSimilarImages(const cv::Mat& qimage, const std::vector<cv::DMatch>& gmatches, std::vector<ImageMatch>& img_matches);
    void remove(const unsigned int desc_id);
    void rebuild();
    unsigned size();

private:
    bool _init;
    int _branching;
    int _trees;
    int _leaf_size;
    float _rebuild_thresh;
    int _desc_bytes;
    int _total_images;
    MeanPointPolicy _meanp_policy;
    ScoringMethod _scoring_method;
    flann::Index<flann::Hamming<unsigned char> >* _feat_index;
    std::map<int, std::vector<InvertedIndexEntry> > _inv_index;
    cv::Mat _descriptors;
    int _next_desc_pos;

    FMat _toFlannMat(const cv::Mat& cvmat);
    cv::Mat _toCvMat(const FMat& flannmat, int type = 0);
    void _initIndex( const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs);
    void _updateDescriptors(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs, const std::vector<cv::DMatch>& matches);
    void _copyDescriptors(const cv::Mat& descs);
};

}

#endif /* BINARYINDEX_H_ */

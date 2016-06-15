/**
 * File: BinaryIndex.cpp
 * Date: April 2015
 * Author: Emilio Garcia-Fidalgo 
 * License: see the LICENSE file.
 */

#include "obindex/BinaryIndex.h"

namespace obindex
{

BinaryIndex::BinaryIndex(const BinaryIndexParams params) :
    _init(false),
    _branching(params.branching),
    _trees(params.trees),
    _leaf_size(params.leaf_size),
    _rebuild_thresh(params.rebuild_thresh),
    _desc_bytes(params.desc_bytes),
    _total_images(0),    
    _meanp_policy(params.meanp_policy),
    _scoring_method(params.scoring_method),
    _feat_index(0),
    _next_desc_pos(0)
{
    _descriptors = cv::Mat::zeros(params.max_descriptors, _desc_bytes, CV_8U);
}

BinaryIndex::~BinaryIndex()
{
    _descriptors.release();

    if (_init)
    {
        delete _feat_index;
    }
}

void BinaryIndex::add(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs)
{
    if (!_init)
    {
        _initIndex(image_id, kps, descs);
        _init = true;
    }
    else
    {
        // Copying the new descriptors
        int prev_desc_id = _next_desc_pos;
        _copyDescriptors(descs);
        int post_desc_id = _next_desc_pos;

        // Adding new points to the index.
        FMat flann_mat = _toFlannMat(_descriptors.rowRange(prev_desc_id, post_desc_id));
        size_t prev_size = _feat_index->size();
        _feat_index->addPoints(flann_mat, _rebuild_thresh);
        size_t post_size = _feat_index->size();

        // Associate these descriptors to the image in the inverse index.
        for (size_t desc_ind = prev_size, qimage_index = 0; desc_ind < post_size; desc_ind++, qimage_index++)
        {
            InvertedIndexEntry entry;
            entry.image_id = image_id;
            entry.distance = 0.0f;
            entry.coords = kps[qimage_index].pt;
            entry.orig_feat_id = qimage_index;
            _inv_index[desc_ind].push_back(entry);
        }
    }
    _total_images++;
}

void BinaryIndex::addToInvertedIndex(const int image_id, const std::vector<cv::KeyPoint>& kps, const std::vector<cv::DMatch>& matches)
{
	for (size_t match_ind = 0; match_ind < matches.size(); match_ind++)
	{   
		int qindex = matches[match_ind].queryIdx;
		int tindex = matches[match_ind].trainIdx;

		// Adding an entry to the inverse index.
		InvertedIndexEntry inv_entry;
		inv_entry.image_id = image_id;
		inv_entry.distance = matches[match_ind].distance;
		inv_entry.coords = kps[qindex].pt;
		inv_entry.orig_feat_id = qindex;
		_inv_index[tindex].push_back(inv_entry);
	}
	_total_images++;
}

void BinaryIndex::update(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs, const std::vector<cv::DMatch>& matches)
{
	// Updating the visual descriptors found in the index according to the matches
	_updateDescriptors(image_id, kps, descs, matches);

	// All features.
	std::set<int> points;
	for (size_t feat_ind = 0; feat_ind < kps.size(); feat_ind++)
	{
		points.insert(feat_ind);
	}

	// Matched features.
	std::set<int> matched_points;
	for (size_t match_ind = 0; match_ind < matches.size(); match_ind++)
	{
		matched_points.insert(matches[match_ind].queryIdx);
	}

    // Computing the difference.
    std::set<int> diff;
    std::set_difference(points.begin(), points.end(), matched_points.begin(), matched_points.end(), std::inserter(diff, diff.end()));

    // Inserting new features into the index.
    for (std::set<int>::iterator it = diff.begin(); it != diff.end(); ++it)
    {
        int index = *it;
        // Copy the descriptor
        descs.row(index).copyTo(_descriptors.row(_next_desc_pos));

        // Adding the point to the index
        _feat_index->addPoints(_toFlannMat(_descriptors.row(_next_desc_pos)), _rebuild_thresh);

        _next_desc_pos++;

        InvertedIndexEntry entry;
        entry.image_id = image_id;
        entry.distance = 0.0f;
        entry.coords = kps[index].pt;
        entry.orig_feat_id = index;
        _inv_index[_feat_index->size() - 1].push_back(entry);
    }
    _total_images++;
}

double BinaryIndex::search(const cv::Mat& qdescs, std::vector<std::vector<cv::DMatch> >& matches, const int knn)
{
    FMat flann_descs = _toFlannMat(qdescs);
	std::vector<std::vector<int> > indices;
	indices.reserve(qdescs.rows);
	std::vector<std::vector<unsigned int> > dists;
	dists.reserve(qdescs.rows);
	// Searching the index.
    Timer t;
    t.start();
    _feat_index->knnSearch(flann_descs, indices, dists, knn, flann::SearchParams());
    t.stop();
    double search_time = t.getInterval();

	// Translating the matches to CV structures.
	matches.clear();
	for (size_t qindex = 0; qindex < indices.size(); qindex++)
	{
		std::vector<cv::DMatch> des_match;
		for (size_t tindex = 0; tindex < indices[qindex].size(); tindex++)
		{
			cv::DMatch match;
			match.queryIdx = qindex;
			match.trainIdx = indices[qindex][tindex];
            match.imgIdx = _inv_index[match.trainIdx][0].image_id; // Image ID where this word was originally created.
			match.distance = static_cast<float>(dists[qindex][tindex]);
			des_match.push_back(match);
		}
		matches.push_back(des_match);
	}

    return search_time;
}

double BinaryIndex::getSimilarImages(const cv::Mat& qimage, const std::vector<cv::DMatch>& gmatches, std::vector<ImageMatch>& img_matches)
{
    // Initializing the resulting structure
    img_matches.resize(_total_images);
    for (int i = 0; i < _total_images; i++)
    {
        img_matches[i].image_id = i;
    }

    // if tf-idf, we count the number of occurrences of a word in an image.
    std::map<int, int> nwi_map;
    if (_scoring_method == SCORING_TFIDF)
    {
        for (unsigned match_index = 0; match_index < gmatches.size(); match_index++)
        {
            int train_idx = gmatches[match_index].trainIdx;
            // Updating nwi_map, number of occurrences of a word in an image.
            if (nwi_map.count(train_idx))
            {
                nwi_map[train_idx]++;
            }
            else
            {
                nwi_map[train_idx] = 1;
            }
        }
    }

    // We process all the matchings again to increase the scores.
    for (unsigned match_index = 0; match_index < gmatches.size(); match_index++)
    {        
        int train_idx = gmatches[match_index].trainIdx;
        float distance = gmatches[match_index].distance;
        float score_inc = 1.0;

        if (_scoring_method == SCORING_TFIDF)
        {
            // Computing the TF term.
            double tf = static_cast<double>(nwi_map[train_idx]) / qimage.rows;

            // Computing the IDF term.
            std::set<int> nw;
            for (unsigned i = 0; i < _inv_index[train_idx].size(); i++)
            {
                nw.insert(_inv_index[train_idx][i].image_id);
            }
            double idf = log(static_cast<double>(_total_images) / nw.size());

            // Computing the final TF-IDF weighting term.
            double tf_idf = tf * idf;
            score_inc = tf_idf;
        }

        for (unsigned invind_index = 0; invind_index < _inv_index[train_idx].size(); invind_index++)
        {
            if (_scoring_method == SCORING_DIST)
            {
                score_inc = 1.0 / (_inv_index[train_idx][invind_index].distance + 1);
            }

            int im = _inv_index[train_idx][invind_index].image_id;
            img_matches[im].score += score_inc;
        }
    }

    std::sort(img_matches.begin(), img_matches.end());
}

void BinaryIndex::remove(const unsigned int desc_id)
{
    if (desc_id < _feat_index->size() && desc_id >= 0)
    {
        _feat_index->removePoint(desc_id);
        _inv_index[desc_id].clear();
    }
}

void BinaryIndex::rebuild()
{
    _feat_index->buildIndex();
}

unsigned BinaryIndex::size()
{
    return _feat_index->size();
}

FMat BinaryIndex::_toFlannMat(const cv::Mat& cvmat)
{
    FMat flannmat(cvmat.data, cvmat.rows, cvmat.cols);
    return flannmat;
}

cv::Mat BinaryIndex::_toCvMat(const FMat& flannmat, int type)
{
    cv::Mat cvmat(flannmat.rows, flannmat.cols, type, flannmat.ptr());
    return cvmat;
}

void BinaryIndex::_initIndex(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs)
{
    // Copying the descriptors to the merged matrix
    _copyDescriptors(descs);

    // Creating the feature index with the given descriptors.
    flann::HierarchicalClusteringIndexParams index_params(_branching, flann::FLANN_CENTERS_RANDOM, _trees, _leaf_size);
    FMat init_mat = _toFlannMat(_descriptors.rowRange(0, descs.rows));
    _feat_index = new flann::Index< flann::Hamming<unsigned char> >(init_mat, index_params);
    _feat_index->buildIndex();

    // Associate these descriptors to the image in the inverse index.
    for (size_t desc_ind = 0; desc_ind < _feat_index->size(); desc_ind++)
    {
        InvertedIndexEntry entry;
        entry.image_id = image_id;
        entry.distance = 0.0f;
        entry.coords = kps[desc_ind].pt;
        entry.orig_feat_id = desc_ind;
        _inv_index[desc_ind].push_back(entry);
    }
}

void BinaryIndex::_updateDescriptors(const int image_id, const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs, const std::vector<cv::DMatch>& matches)
{
    for (size_t match_ind = 0; match_ind < matches.size(); match_ind++)
    {
        int qindex = matches[match_ind].queryIdx;
        int tindex = matches[match_ind].trainIdx;

        // Updating the descriptor inside the feature index.
        unsigned char new_desc[_desc_bytes];
        const uchar* q_desc = descs.ptr(qindex);
        const uchar* t_desc = _feat_index->getPoint(tindex);
        for (int byte_ind = 0; byte_ind < _desc_bytes; byte_ind++)
        {
            if (_meanp_policy == MEANP_POLICY_OR)
            {
                new_desc[byte_ind] = q_desc[byte_ind] | t_desc[byte_ind];
            }
            else if (_meanp_policy == MEANP_POLICY_AND)
            {
                new_desc[byte_ind] = q_desc[byte_ind] & t_desc[byte_ind];
            }
        }
        memcpy(_feat_index->getPoint(tindex), new_desc, sizeof(uchar) * _desc_bytes);

        // Adding an entry to the inverse index.
        InvertedIndexEntry inv_entry;
        inv_entry.image_id = image_id;
        inv_entry.distance = matches[match_ind].distance;
        inv_entry.coords = kps[qindex].pt;
        inv_entry.orig_feat_id = qindex;
        _inv_index[tindex].push_back(inv_entry);
    }
}

void BinaryIndex::_copyDescriptors(const cv::Mat &descs)
{
    // Copying the descriptors to the merged matrix
    int ndescs = descs.rows;
    for (int i = 0; i < ndescs; i++)
    {
        descs.row(i).copyTo(_descriptors.row(_next_desc_pos));
        _next_desc_pos++;
    }
}

}

/**
 * File: example.cpp
 * Date: April 2015
 * Author: Emilio Garcia-Fidalgo
 * License: see the LICENSE file.
 */


#include <cstdio>
#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "obindex/BinaryIndex.h"

namespace vi = obindex;

void getFilenames(const std::string& directory, std::vector<std::string>& filenames)
{
    using namespace boost::filesystem;

    filenames.clear();
    path dir(directory);

    // Retrieving, sorting and filtering filenames.
    std::vector<path> entries;
    copy(directory_iterator(dir), directory_iterator(), back_inserter(entries));
    sort(entries.begin(), entries.end());
    for (std::vector<path>::const_iterator it(entries.begin()); it != entries.end(); ++it)
    {
        std::string ext = it->extension().c_str();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".jpg" || ext == ".ppm" || ext == ".jpeg")
        {
            filenames.push_back(it->string());
        }
    }
}

int main(int argc, char** argv)
{
	std::cout << "-- Example: obindex for indexing a set of images --" << std::endl;
	std::cout << "-- Copyright (c) 2015 Emilio Garcia-Fidalgo --" << std::endl;

	if (argc != 2)
	{
		std::cout << "USAGE: ./test_bindex DIR_OF_IMAGES" << std::endl;
		exit(0);
	}

    std::cout << "Validating BinaryIndex ..." << std::endl;

    std::string dir = argv[1];
    std::vector<std::string> filenames;
    getFilenames(dir, filenames);
    int nimages = filenames.size();

    // Other binary descriptors could be used.
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();

	// Set the parameters according to your needs. See BinaryIndex.h for more information.
	vi::BinaryIndexParams params;
	params.desc_bytes = 32; // Size in bytes of your binary descriptor.
	params.max_descriptors = 3000000; // Maximum number of descriptors to be stores in the index.

	vi::BinaryIndex index(params);

    // Adding image 0
    // Detecting and describing keypoints.
    std::vector<cv::KeyPoint> kps0;
    cv::Mat dscs0;
    cv::Mat image0 = cv::imread(filenames[0]);
    detector->detect(image0, kps0);
    extractor->compute(image0, kps0, dscs0);
    cv::KeyPointsFilter::retainBest(kps0, 1000);
	// Adding the image to the index.
	index.add(0, kps0, dscs0);

    for (int i = 0; i < nimages; i++)
    {
        std::cout << "Processing image " << i << std::endl;

        // Detecting and describing keypoints.
        std::vector<cv::KeyPoint> kps;
        cv::Mat dscs;
        cv::Mat image = cv::imread(filenames[i]);
        detector->detect(image, kps);
        extractor->compute(image, kps, dscs);
        cv::KeyPointsFilter::retainBest(kps, 1000);

        // Matching the images
        std::vector<std::vector<cv::DMatch> > matches_feats;

		// Searching the query descriptors against the features.
        index.search(dscs, matches_feats, 2);

        // Filtering matches according to the ratio test.
        std::vector<cv::DMatch> matches;
        for (unsigned m = 0; m < matches_feats.size(); m++)
        {
            if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8)
            {
                matches.push_back(matches_feats[m][0]);
            }
        }

        std::vector<vi::ImageMatch> image_matches;
		// We look for similar images according to the good matches found in the index.
        index.getSimilarImages(dscs, matches, image_matches);

        // Showing results
        for (int j = 0; j < std::min(15, int(image_matches.size())); j++)
        {
            std::cout << "Cand: " << image_matches[j].image_id << ", Score: " << image_matches[j].score << std::endl;
        }

        std::cout << "Total features found in the image: " << kps.size() << std::endl;
        std::cout << "Total matches found against the index: " << matches.size() << std::endl;
        std::cout << "Total index size BEFORE UPDATE: " << index.size() << std::endl;
		// Updating the index. Matched descriptors are used to update the index and the remaining ones are added as a new descriptors.
        index.update(i, kps, dscs, matches);
        std::cout << "Total index size AFTER UPDATE: " << index.size() << std::endl;
    }
}

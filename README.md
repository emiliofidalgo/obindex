# OBINDEX

OBINDEX is an open source C++ library for indexing images. It implements a hierarchical indexing scheme to match binary descriptors, which is used as an incremental Bag-Of-Words (BoW) visual vocabulary. This index is combined with an inverted file in order to provide an image database, that can be queried for finding similar images. This online binary visual vocabulary tries to solve the main drawbacks that classical BoW approaches present, avoiding the training step and adapting the visual dictionary to the operating environment.

OBINDEX is released as a ROS package, and relies on OpenCV, FLANN and Boost Filesystem. It can be used with any binary descriptor computed using the OpenCV format.

Note that OBINDEX is research code. The authors are not responsible for any errors it may contain. Use it at your own risk!

## Related Publication

Our approach is detailed in:

**On the Use of Binary Feature Descriptors for Loop Closure Detection**     
Emilio Garcia-Fidalgo and Alberto Ortiz     
IEEE International Conference on Emerging Technologies and Factory Automation (ETFA)     
Barcelona (Spain), 2014

The paper can be downloaded from: http://emiliofidalgo.github.io/static/papers/conf_ETFA_Garcia2014.pdf

If you use this software in an academic work, please cite:

	@INPROCEEDINGS{GarciaFidalgoETFA14,
		author={Garcia-Fidalgo, E. and Ortiz, A.}, 
		booktitle={Emerging Technology and Factory Automation (ETFA), 2014 IEEE},
		title={On the use of binary feature descriptors for loop closure detection},
		year={2014},
		month={Sept},
		pages={1-8},
		doi={10.1109/ETFA.2014.7005121}
	}

## License

OBINDEX is released under a BSD license. Please, read the *LICENSE* file distributed with this package for further details.

## Installation

## Usage

For an example of use, see the demo file (`src/demo/example.cpp`).

## Known Limitations

Currently, the memory allocation to store the descriptors in the index is performed in the class constructor and not dynamically. This implies that you should indicate as a parameter (`max_descriptors`) the approximate number of descriptors that you need to index.

## Contact

If you have problems or questions, please contact the authors (emilio.garcia@uib.es).

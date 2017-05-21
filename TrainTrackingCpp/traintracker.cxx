//opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "opencv2/core.hpp"

//C
#include <iostream>

using namespace std;
using namespace cv;
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>

/* 
 * File:   traintracker.cxx
 * Author: Luke
 *
 * Created on 21 May 2017, 19:13
 */

#include <cstdlib>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    // Read image
    Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );
    return 0;
}


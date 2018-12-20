//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef __H_FILEWRITER
#define __H_FILEWRITER

#include "detection.h"
#include <fstream>
#include <iostream>


/*!
	This class is used to write detections to a file.
*/

class FileWriter {

public:

    /*! Initialise the file to write to*/
    FileWriter(std::string filename, bool txt=false) ;
    /*!
    	Write the detections one by one to the file. It is important that these come image per image
    */
    void writeDetection(Detection *det);
    void writeDetectiontxt(Detection *det);

    /*!
    	Destruct the object. This means closing the file
    */
    ~FileWriter() ;

private:
    std::ofstream m_file;
	bool txt; // to make out if it is XML or  TXT
};
#endif

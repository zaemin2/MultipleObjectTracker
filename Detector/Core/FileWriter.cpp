//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include "FileWriter.h"
/*!
        \author F. De Smedt
        \date 2014
*/
FileWriter::FileWriter(std::string filename, bool txt): txt(txt) {

    /*Open the file*/
    m_file.open(filename.c_str());
if(!txt){
    /*Write header of file*/
    m_file << "<?xml version=\"1.0\"?>" << std::endl;
    m_file << "<Detections>" << std::endl;
}
}


void FileWriter::writeDetectiontxt(Detection *det) {
    static std::string s_oldFileName="NonExist";

     m_file << det->getFilename() << "," << det->getX() << "," << det->getY() << "," << det->getWidth() << "," << det->getHeight() << "," << det->getScore() << std::endl;

}

void FileWriter::writeDetection(Detection *det) {
    static std::string s_oldFileName="NonExist";

    /*Check if this detection belongs to the same file as the previous one*/
    if(det->getFilename() != s_oldFileName) {
        if(s_oldFileName == "NonExist") {
            /*Create first Detection*/
            m_file << "<File>" << std::endl;
        }
        else {
            /*close the previous header*/

            m_file << "</File>" << std::endl;
            m_file << "<File>" << std::endl;
        }
        /*Write Path*/
        m_file << "<Path>" << det->getFilename() << "</Path>" << std::endl;
    }

    s_oldFileName = det->getFilename();

    /*Write the detection*/
    m_file << "<Detection>"  << std::endl;
    m_file << "\t<x>" << det->getX() << "</x>" << std::endl;
    m_file << "\t<y>" << det->getY() << "</y>" << std::endl;
    m_file << "\t<width>" << det->getWidth() << "</width>" << std::endl;
    m_file << "\t<height>" << det->getHeight() << "</height>" << std::endl;
    m_file << "\t<score>" << det->getScore() << "</score>" << std::endl;
    m_file << "</Detection>"  << std::endl;

}


FileWriter::~FileWriter() {
if(!txt){
    /*end the last file*/
    m_file << "</File>" << std::endl;
    m_file << "</Detections>" << std::endl;
    /*Close the file*/
    m_file.close();
}
else{
	std::cout << "CLosing textbased" << std::endl;
}
}


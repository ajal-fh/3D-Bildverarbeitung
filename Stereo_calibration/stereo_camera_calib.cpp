#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

static void help()
{
    cout <<  "This is a stereo-camera calibration sample." << endl;

}

class Settings
{
public:	
    Settings() : goodInput(false) {}
    
    bool goodInput;

    Mat distCoeffs,imagePoints,cameraMatrix;
    
        void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
                  << "distortion_coefficients"  << distCoeffs
                  << "image_points" << imagePoints
                  << "extrinsic_parameters"         << cameraMatrix

           << "}";
    }

    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["distortion_coefficients" ] >> distCoeffs;
        node["image_points" ] >> imagePoints;       
        node["extrinsic_parameters" ] >> cameraMatrix;
       
    }
    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
};

int main(int argc, char* argv[])
{
    help();

    //! [file_read]
    Settings cam1;
    const string inputSettingsFile = "out_camera_data.xml";
    cout << inputSettingsFile << endl;
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }

    fs["Settings"] >> cam1;
    fs.release();                                         // close Settings file
    //! [file_read]

	//input video Capture (0) and (1) videoCapture cam; cam.open(0)
	//monocalibration both cameras same time
	//stereocalibrate


}

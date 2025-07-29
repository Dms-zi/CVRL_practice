#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    // read argv[1] as image file
    cv::Mat image;
    image = cv::imread(argv[1]);

    if(!image.data) 
    {
      cout << "Could not open or find the image!" << endl;
      return -1;
    }

    cout << "Columns: " << image.cols << endl;
    cout << "Rows: " << image.rows << endl;
    cout << "Channels: " << image.channels() << endl;

    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a keystroke in the window

    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) 
    {
        cout << "Image is not grayscale or RGB!" << endl;
        return -1;
    }

    // std::chrono
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    for (size_t y=0; y < image.rows; y++) 
    {
       unsigned char* row_ptr = image.ptr<unsigned char>(y);
       for (size_t x=0; x < image.cols; x++) 
       {
            unsigned char* pixel_ptr = row_ptr + x * image.channels(); //data_ptr
            for(int c=0; c < image.channels(); c++) 
            {
                unsigned char& pixel = pixel_ptr[c];
            }
        }

    }
    
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>(end - start);
    cout << "Time taken to access pixel: " << duration.count() << " seconds" << endl;


    //cv::Mat copy 
    cv::Mat copy = image; // not a deep copy
    copy(cv::Rect(0, 0, 100, 100)).setTo(0);
    cv::imshow("Copy", copy);
    cv::waitKey(0);

    cv::Mat deepCopy = image.clone(); // deep copy
    deepCopy(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("Deep Copy", deepCopy);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
#include <esim/unrealcv_bridge/unrealcv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>

using boost::asio::ip::tcp;
using namespace std;

int main() {
  event_camera_simulator::UnrealCvClient client("localhost", "9000");

  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE );
  cv::namedWindow("Depthmap", cv::WINDOW_AUTOSIZE );

  for(double y = 0.0; y<100.0; y+=10.0)
  {
    event_camera_simulator::CameraData test = {0,
                                               0.0,
                                               0.0,
                                               0.0,
                                               0.0,
                                               y,
                                               100.0};

    client.setCamera(test);
    cv::Mat img = client.getImage(0);
    cv::imshow("Image", img);

    cv::Mat depthmap = client.getDepth(0);
    cv::normalize(depthmap, depthmap, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Depthmap", depthmap);

    cv::waitKey(10);
  }

  return 0;
}

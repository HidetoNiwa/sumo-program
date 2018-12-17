//compile command
// g++ main.cpp -I ./lib -std=c++11 ./lib/control.cpp ./lib/image.cpp ./lib/realtime.cpp `pkg-config --cflags --libs opencv`
#include <unistd.h>
#include "./lib/control.h"
#include "./lib/image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp" // highguiのヘッダーをインクルード
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "box.h"

using namespace cv;
using namespace std;

Mat image, image_blue, image_red, image_green, image3, image_disp_red, image_disp_green, image_disp_blue;

class ImageProcessing : public sumo::Image
{
  public:
    void handleImage(const struct sumo::image *, const uint8_t *buffer, size_t size)
    {
        std::vector<unsigned char> vec;
        for (int i = 0; i < size; i++)
            vec.push_back(buffer[i]);
        Mat mobj(vec, false);
        image = imdecode(mobj, CV_LOAD_IMAGE_COLOR);
    }
};

int main(int argh, char *argv[])
{
    //自作ボックスクラス適用
    box matrix;
    box face;

    // 画像表示用のウィンドウ作成、画像の初期化
    namedWindow("Sumo Camera face", WINDOW_AUTOSIZE);
    namedWindow("Sumo Camera marix", WINDOW_AUTOSIZE);
    image = Mat(640, 480, CV_8UC3, Scalar(0, 0, 255));

    // 顔検出用変数、クラス
    Mat grayImage;                                                       // 白黒画像用変数
    std::vector<Rect> faces;                                             // 顔情報用変数
    CascadeClassifier classifier("haarcascade_frontalface_default.xml"); // 顔検出クラス

    std::vector<KeyPoint> keypoints; // コーナーの情報を扱う変数
    std::vector<Point2f> plots;
    std::vector<Vec3f> r, g, b;
    auto detector = GFTTDetector::create(76, 0.01, 10, 3, true); // 検出クラス

    uint16_t span[2] = {0};
    uint16_t range[2] = {0};

    uint8_t mode = 1; //1でface,2で格子,3で円箱

    face.flag = 0;
    matrix.flag = 0;
    face.sum = 0;
    matrix.sum = 0;
    face.count = 0;
    matrix.count = 0;
    face.ave = 0;
    matrix.ave = 0;
    face.motion = 0;
    matrix.motion = 0;
    uint8_t flag_box = 0;
    int radius;

    Mat channels[3];

    // SUMOの初期化
    sumo::Control sumo(new ImageProcessing);
    if (!sumo.open())
        return EXIT_FAILURE;

    // メインループ
    for (uint16_t i = 0; i < 10000; i++)
    {
        if (!image.empty()) // 画像が空でなければ、画像を処理する
        {
            GaussianBlur(image, image, Size(5, 5), 0);
            if (i < 60)
            {
                if (i < 5)
                {
                    sumo.tap();
                }
                cvtColor(image, grayImage, CV_BGR2GRAY);       // 画像を白黒に変換
                classifier.detectMultiScale(grayImage, faces); // 画像から顔を検出
                if (face.flag == 8)
                {
                    if (face.ave < 640 / 3)
                    {
                        sumo.move(127, (face.ave - 640 / 2) / 130);
                    }
                    else if (face.ave > 640 * 2 / 3)
                    {
                        sumo.move(127, (face.ave - 640 / 2) / 130);
                    }
                    else
                    {
                        sumo.move(127, 0);
                    }

                    if (faces.size() == 0)
                    {
                        face.flag = 0;
                    }
                }
                else
                {
                    face.sum = 0;
                    face.count = 0;
                    face.ave = 0;
                    for (int i = 0; i < faces.size(); i++)
                    {
                        rectangle(image, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(100, 100, 200), 2, 8, 0); // 検出した顔を四角で囲む
                        if (((faces[i].y) > 480 / 4) && ((faces[i].y) < 480 / 4 * 3))
                        {
                            face.sum = face.sum + faces[i].x + faces[i].width / 2;
                            face.count++;
                        }
                    }
                    if (face.count > 0)
                    {
                        face.ave = face.sum / face.count;
                        if (face.ave < 640 / 3)
                        {
                            sumo.move(0, (face.ave - 640 / 2) / 200);
                        }
                        else if (face.ave > 640 * 2 / 3)
                        {
                            sumo.move(0, (face.ave - 640 / 2) / 200);
                        }
                        else
                        {
                            sumo.move(0, 0);
                        }
                        face.flag++;
                    }
                    else
                    {
                        sumo.move(0, 3);
                    }
                }
                imshow("Sumo Camera", image); // 画像と検出結果を表示
            }
            else if ((i < 220) && (i > 59))
            {
                if (i < 62)
                {
                    sumo.swing();
                }
                if (i > 220)
                {
                    i = 0;
                }
                if (matrix.flag == 1)
                {
                    cvtColor(image, grayImage, CV_BGR2GRAY); // 白黒画像に変換
                    detector->detect(grayImage, keypoints);  // 検出

                    matrix.sum = 0;
                    matrix.ave = 0;
                    matrix.count = 0;
                    for (uint8_t k = 0; k < keypoints.size(); k++)
                    {
                        if ((keypoints[k].pt.y < 480 / 3 * 2) && (keypoints[k].pt.y > 480 / 3))
                        {
                            matrix.sum = matrix.sum + keypoints[k].pt.x;
                            matrix.count++;
                        }
                    }

                    if (matrix.count > 0)
                    {
                        matrix.ave = matrix.sum / matrix.count;
                    }
                    else
                    {
                        matrix.ave = 320;
                    }

                    sumo.move(abs((keypoints.size())), (matrix.ave - 320) / 5);
                    drawKeypoints(image, keypoints, grayImage, Scalar(0, 0, 255));

                    // 検出したコーナーを画像に描画する
                    imshow("Sumo Camera", grayImage); // 画像と検出結果を表示する
                    span[1] = span[0];
                }
                else
                {
                    cvtColor(image, grayImage, CV_BGR2GRAY); // 白黒画像に変換
                    detector->detect(grayImage, keypoints);
                    matrix.sum = 0;
                    matrix.count = 0;
                    matrix.ave = 0;
                    for (uint8_t k = 0; k < keypoints.size(); k++)
                    {
                        if ((keypoints[k].pt.y < 480 / 4 * 3) && (keypoints[k].pt.y > 480 / 3))
                        {
                            matrix.sum = matrix.sum + keypoints[k].pt.x;
                            matrix.count++;
                        }
                    }
                    if (matrix.count > 0)
                    {
                        matrix.ave = matrix.sum / matrix.count;
                    }
                    else
                    {
                        matrix.ave = 0;
                    }

                    if ((matrix.ave > 640 / 3) && (matrix.ave < 640 / 3 * 2))
                    {
                        matrix.flag++;
                    }
                    else
                    {
                        sumo.move(0, -20);
                    }
                    drawKeypoints(image, keypoints, grayImage, Scalar(0, 0, 255));
                    imshow("Sumo Camera", grayImage);
                    span[1] = span[0];
                    range[1] = range[0];
                }
            }
            else
            {
                uint8_t count = 0;
                uint8_t centor_x[255] = {0};
                float sum_x = 0;
                float ave = 0;
                if (i > 350)
                {
                    i = 0;
                }
                if (i < 230)
                {
                    sumo.lookLeftAndRight();
                }
                /*
                split(image, channels);
                HoughCircles(channels[2], r, CV_HOUGH_GRADIENT, 1, 50, 100, 75, 0, 300);
                for (auto it = r.begin(); it != r.end(); ++it)
                {
                    centor_x[count] = (*it)[0];
                    count++;
                    radius = (*it)[2];
                    Point centor = Point((*it)[0], (*it)[1]);
                    circle(image, centor, radius, Scalar(0, 0, 255), 2);
                }
                HoughCircles(channels[1], g, CV_HOUGH_GRADIENT, 1, 50, 100, 75, 0, 300);
                for (auto it = r.begin(); it != r.end(); ++it)
                {
                    centor_x[count] = (*it)[0];
                    count++;
                    radius = (*it)[2];
                    Point centor = Point((*it)[0], (*it)[1]);
                    circle(image, centor, radius, Scalar(0, 255, 0), 2);
                }
                HoughCircles(channels[0], b, CV_HOUGH_GRADIENT, 1, 50, 100, 75, 0, 300);
                for (auto it = r.begin(); it != r.end(); ++it)
                {
                    centor_x[count] = (*it)[0];
                    count++;
                    radius = (*it)[2];
                    Point centor = Point((*it)[0], (*it)[1]);
                    circle(image, centor, radius, Scalar(255, 0, 0), 2);
                }
                if (count == 0)
                {
                    sumo.move(0, 10);
                }
                else
                {
                    for (uint8_t k = 0; k < count; k++)
                    {
                        sum_x = centor_x[k] + sum_x;
                    }
                    printf("%d", count);
                    ave = sum_x / count;
                    sumo.move(127, (ave - 320) / 20);
                }
                */
                imshow("Sumo Camera", image); // 画像と検出結果を表示
            }
        }
        if (waitKey(10) == 27)
            break;
    }

    // SUMO終了処理
    printf("Finalize SUMO\n");
    sumo.close();

    // （4）ウィンドウを破棄する
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}

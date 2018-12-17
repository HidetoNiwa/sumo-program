//compile command
// g++ main.cpp -I ./lib -std=c++11  -pthread ./lib/control.cpp ./lib/image.cpp ./lib/realtime.cpp `pkg-config --cflags --libs opencv`
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

int main(void)
{
	// 画像表示用のウィンドウ作成、画像の初期化
	namedWindow("Sumo Camera Red", WINDOW_AUTOSIZE);
	namedWindow("Sumo Camera Green", WINDOW_AUTOSIZE);
	namedWindow("Sumo Camera Blue", WINDOW_AUTOSIZE);
	image = Mat(640, 480, CV_8UC3, Scalar(0, 0, 255));
	Mat *rgb = new Mat[3];
	Mat *rgb2 = new Mat[3];
	Mat *rgb3 = new Mat[3];
	Mat red, green, blue;

	// SUMOの初期化
	sumo::Control sumo(new ImageProcessing);
	if (!sumo.open())
		return EXIT_FAILURE;
	red = 0;
	green = 0;
	blue = 0;

	uint32_t flag = 0;
	bool start_flag = 0;

	// メインループ
	for (uint16_t i = 0; i < 10000; i++)
	{
		if (!image.empty()) // 画像が空でなければ、画像を処理する
		{

			image_red = image.clone();
			image_green = image.clone();
			image_blue = image.clone(); // 画像を image にコピーする
			split(image_red, rgb);
			split(image_green, rgb2);
			split(image_blue, rgb3); // カラー画像をBGRの単色画像に分割
			red = rgb[2] / 2 + (~rgb[1]) / 4 + (~rgb[0]) / 4;
			green = rgb2[1] / 2 + (~rgb2[0]) / 4 + (~rgb2[2]) / 4;
			blue = rgb3[0] / 2 + (~rgb3[1]) / 4 + (~rgb3[2]) / 4; // 青が強調される画像を作成
			threshold(red, red, 150, 255, THRESH_BINARY);
			threshold(green, green, 150, 255, THRESH_BINARY);
			threshold(blue, blue, 150, 255, THRESH_BINARY); // 閾値以下の部分を0にする

			Mat vec1_blue, vec2_blue;														  // 画像の平均値計算用変数
			reduce(blue, vec1_blue, 0, REDUCE_AVG);											  // 列の平均値を表す行ベクトルを求める
			reduce(vec1_blue, vec2_blue, 1, REDUCE_AVG);									  // 行ベクトルの平均値を表すスカラ値を求める
			double mean_blue = vec2_blue.data[0] * 3.0;										  // 平均値を取り出す
			Moments m_blue = moments(blue, 0);												  // 画像モーメントを求める
			double gx_blue = m_blue.m10 / m_blue.m00, gy_blue = m_blue.m01 / m_blue.m00;	  // 色重心の位置を求める
			circle(blue, Point(gx_blue, gy_blue), mean_blue, Scalar(255, 255, 255), 6, 8, 0); // 色重心位置に円を描画する

			Mat vec1_red, vec2_red;														  // 画像の平均値計算用変数
			reduce(red, vec1_red, 0, REDUCE_AVG);										  // 列の平均値を表す行ベクトルを求める
			reduce(vec1_red, vec2_red, 1, REDUCE_AVG);									  // 行ベクトルの平均値を表すスカラ値を求める
			double mean_red = vec2_red.data[0] * 3.0;									  // 平均値を取り出す
			Moments m_red = moments(red, 0);											  // 画像モーメントを求める
			double gx_red = m_red.m10 / m_red.m00, gy_red = m_red.m01 / m_red.m00;		  // 色重心の位置を求める
			circle(red, Point(gx_red, gy_red), mean_red, Scalar(255, 255, 255), 6, 8, 0); // 色重心位置に円を描画する

			Mat vec1_green, vec2_green;															  // 画像の平均値計算用変数
			reduce(green, vec1_green, 0, REDUCE_AVG);											  // 列の平均値を表す行ベクトルを求める
			reduce(vec1_green, vec2_green, 1, REDUCE_AVG);										  // 行ベクトルの平均値を表すスカラ値を求める
			double mean_green = vec2_green.data[0] * 3.0;										  // 平均値を取り出す
			Moments m_green = moments(green, 0);												  // 画像モーメントを求める
			double gx_green = m_green.m10 / m_green.m00, gy_green = m_green.m01 / m_green.m00;	// 色重心の位置を求める
			circle(green, Point(gx_green, gy_green), mean_green, Scalar(255, 255, 255), 6, 8, 0); // 色重心位置に円を描画する

			rgb[0] = 0;								   // 処理済みの画像を青色値とする
			rgb[1] = 0;								   // 緑色は0で塗りつぶす
			rgb[2] = red;							   // 赤色は0で塗りつぶす
			merge(rgb, 3, image_disp_red);			   // 単色画像をカラー画像に結合する
			imshow("Sumo Camera Red", image_disp_red); // 処理済みの画像を表示する

			rgb2[0] = 0;					  // 処理済みの画像を青色値とする
			rgb2[1] = green;				  // 緑色は0で塗りつぶす
			rgb2[2] = 0;					  // 赤色は0で塗りつぶす
			merge(rgb2, 3, image_disp_green); // 単色画像をカラー画像に結合する
			//imshow("Sumo Camera Green", image_disp_green);			// 処理済みの画像を表示する

			rgb3[0] = blue;								 // 処理済みの画像を青色値とする
			rgb3[1] = 0;								 // 緑色は0で塗りつぶす
			rgb3[2] = 0;								 // 赤色は0で塗りつぶす
			merge(rgb3, 3, image_disp_blue);			 // 単色画像をカラー画像に結合する
			imshow("Sumo Camera Blue", image_disp_blue); // 処理済みの画像を表示する
			if (start_flag == 0)
			{
				if (mean_blue > 20 || mean_red > 20)
				{
					start_flag = 1;
				}
				sumo.move(30, -50);
			}
			if (i > 25 && start_flag == 1)
			{
				if ((mean_green > 5) && (mean_red < 10) && (mean_blue < 10))
				{
					sumo.move(0, 0);
					break;
				}
				else
				{
					if ((mean_red - mean_blue) > 45)
					{
						sumo.move(127.0, -(mean_red - mean_blue) / 10); // 色重心の位置、大きさに応じてSUMOを制御する
						flag = 0;
					}
					else if ((mean_red - mean_blue) < -45)
					{
						sumo.move(127.0, -(mean_red - mean_blue) / 10); // 色重心の位置、大きさに応じてSUMOを制御する
						flag = 0;
					}
					else
					{
						if (flag < 200)
						{
							sumo.move(127.0, (-mean_red + mean_blue) / 2);
							if ((mean_red < 5) && (mean_blue < 5))
							{
								sumo.move(-50.0, (-mean_red + mean_blue));
							}
						}
						else
						{
							sumo.move(-20.0, (-mean_red + mean_blue));
						}
						flag++;
						//sumo.move(50.0, -1);					// 青色の領域が小さすぎる場合は、SUMOを停止する
					}
				}
			}
			else
			{
				sumo.move(0, 0);
			}
		}
		if (waitKey(1) == 27)
			break;
	}

	while (1)
	{
	}

	// SUMO終了処理
	printf("Finalize SUMO\n");
	sumo.close();

	// Wiiコントローラ終了処理
	//	wii_controller_end();

	// （4）ウィンドウを破棄する
	cv::destroyAllWindows();

	return EXIT_SUCCESS;
}

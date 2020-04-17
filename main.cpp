#pragma warning(disable: 4099)
#pragma warning(disable: 4819)

#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common\io.h>
#include <pcl/io\pcd_io.h>
#include <pcl/io\vtk_lib_io.h>
#include <pcl/keypoints/harris_3d.h>             // Harris特徴検出器
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>       // 平面検出

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
//using namespace pcl;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGBNormal PointT;

int main()
{
    //return 0;

    string pcdFileName = "data\\AI2_Obj1.pcd";

    ifstream ifs(pcdFileName);
    //ifstream ifs("data\\test.txt");
    if (ifs.fail()) {
        cerr << "Failed to open file." << std::endl;
        return -1;
    }
    ofstream tmp_file;
    tmp_file.open("data\\tmp.pcd");


    //namedWindow("test", 1);

    string str;
    // ヘッダーのSIZE記載の一文だけ書き換える
    while (getline(ifs, str)) {
        if (!str.compare(0, 4, "SIZE")) {
            while (str.find("8") != string::npos) {
                auto pos = str.find("8");
                str.replace(pos, 1, "4");
            }
        }
        tmp_file << str << std::endl;
    }


    pcl::PointCloud<PointT>::Ptr cloudSrc(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudDst(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile<PointT>("data/11.pcd", *cloudSrc);
    pcl::io::loadPCDFile<pcl::Normal>("data/11.pcd", *normals);

    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>("data\\dst_bin.pcd", *cloud);

    //// PCLVisualizerでの描画
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");

    // 色付け
    viewer.setBackgroundColor(0.3, 0.3, 0.3);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloud, 0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);

    //for (std::size_t i = 0; i < cloudSrc->points.size(); ++i)
    //{
    //	cloudSrc->points[i].r = 0;
    //	cloudSrc->points[i].g = 255;
    //	cloudSrc->points[i].b = 0;
    //}

    *cloudDst = *cloudSrc;

    /*for (std::size_t i = 1; i < cloudDst->points.size(); ++i)
    {
        cloudDst->points[i].normal_x = 0;
        cloudDst->points[i].normal_y = 0;
        cloudDst->points[i].normal_z = 0;
    }
*/
// 法線の推定
//pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
//ne.setMaxDepthChangeFactor(0.01);
//ne.setNormalSmoothingSize(5.0);
//ne.setInputCloud(cloudXYZ);
//ne.compute(*cloudNormals);

// 点群の描画
//viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud, single_color, "cloud1");
//viewer.addPointCloudNormals<pcl::Normal>(cloud_normals, 100, 0.002, "normals");
//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, 100, 0.003, "normals");

    viewer.addPointCloud<PointT>(cloudDst, "cloud1");
    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloudDst, normals, 1, 3, "normalsDst2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
    //viewer.addPointCloudNormals<PointT>(cloudDst, 1, 0.01, "normals");






    // 法線推定用描画
    //viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloudXYZ, cloudNormals, 100, 0.01, "normals");

    //// 図形等のオブジェクト描画
    //viewer.addLine<pcl::PointXYZRGBNormal>(cloud->points[0], cloud->points[cloud->size() - 1], "line");
  // addArrow

  //pcl::PointXYZ p0, p1;
  //p0.x = 0.0;
  //p0.y = 0.0;
  //p0.z = 0.0;
  //p1.x = 0.1;
  //p1.y = 0.1;
  //p1.z = 0.1;
  //viewer.addArrow<pcl::PointXYZ>(p0, p1, 0.0, 1.0, 1.0, "arrow", 0);

  //viewer.addSphere<pcl::PointXYZRGBNormal>(cloud->points[0], 0.01, 1.0, 0.5, 0.5, "sphere");
    //viewer.addPointCloudIntensityGradients<pcl::PointXYZRGBNormal>(,);

    //viewer.addPointCloud<pcl::PointXYZ>(inu_cloud, "cloud1");
    //viewer.addPointCloud<pcl::PointXYZRGB>(AI2_hanger, "cloud1");
    //viewer.addText("cloud1", 10, 10, "c1");
//	float nrmx = cloud->points[0].normal_x;
    //viewer.addText3D( to_string(cloudDst->points[0].x), cloudDst->points[0], 0.01, 0.0, 1.0, 1.0, "text3dtest" );
    //viewer.addCoordinateSystem(0.5);
    //viewer.initCameraParameters();
    viewer.resetCamera();
    viewer.spin();

    /*cv::namedWindow("a", 1);
    cv::waitKey(0);
*/
    return 0;
}
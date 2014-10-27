#include "../visualization.h"
#include <string>

void view( PointCloudRGB & cloud ) {
        pcl::visualization::CloudViewer viewer1("Cloud Viewer");
        viewer1.showCloud( cloud.makeShared() );
    while( !viewer1.wasStopped() );

        return;
}


void drawLine(pcl::visualization::PCLVisualizer &vis, PointRGB start_point, PointRGB end_point, std::string line_id, double r, double g, double b){
	
	vis.addLine (start_point, end_point, r, g, b, line_id);
	
	
};


void showCloudsLeft(const PCD cloud_target, const PCD cloud_source, pcl::visualization::PCLVisualizer *v, int vp1)
{
    v->removeAllPointClouds();

    PointCloudColorHandlerCustom<PointRGB> tgt_h (cloud_target.cloud, 0, 255, 0); //verde
    PointCloudColorHandlerCustom<PointRGB> src_h (cloud_source.cloud, 255, 0, 0); //rosso
    v->addPointCloud (cloud_target.cloud, tgt_h, cloud_target.f_name, vp1);
    v->addPointCloud (cloud_source.cloud, src_h, cloud_source.f_name, vp1);

    v->getCloudActorMap()->at(cloud_target.f_name).actor->PickableOff();
    v->getCloudActorMap()->at(cloud_source.f_name).actor->PickableOff();

    PCL_INFO ("Press q to begin the registration.\n");
    v-> spin();
}


void showCloudsRight(const PointCloudNormal::Ptr cloud_target, const PointCloudNormal::Ptr cloud_source, pcl::visualization::PCLVisualizer *v, int vp2)
{
    v->removePointCloud ("source");
    v->removePointCloud ("target");


    PointCloudColorHandlerGenericField<PNormal> tgt_color_handler (cloud_target, "curvature");
    if (!tgt_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PNormal> src_color_handler (cloud_source, "curvature");
    if (!src_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");


    v->addPointCloud (cloud_target, tgt_color_handler, "target", vp2);
    v->addPointCloud (cloud_source, src_color_handler, "source", vp2);

    v->getCloudActorMap()->at("target").actor->PickableOff();
    v->getCloudActorMap()->at("source").actor->PickableOff();

    v->spinOnce();
}

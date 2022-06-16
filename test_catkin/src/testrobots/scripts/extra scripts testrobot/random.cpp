std::vector<Point2D> calcBoundingBoxInWorldCoords(bool visualizeBB, double camera_x, double camera_y, double camera_theta)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = m_postPlaneExtractedCloud;
  
  // Compute principal directions (equivalent to principal components / PCA)
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);


  //Two methods: compute Eigen vectors, or use a PCA object

  //Method 1
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  //The following line is necessary for proper orientation in some cases.
  //The numbers come out the same without it, but the signs are
  //   different and the box doesn't get correctly oriented in some cases.
   eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  


  
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // For the transformation back to world coordinates
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  std::stringstream ss;
  ss <<  "world_step" << printStepCount() << "_PCA.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudSegmented, false);

  std::cout << "PCA:" << std::endl;
  std::cout << eigenVectorsPCA << std::endl;
  std::cout << "Bounding box transform:" << std::endl;
  std::cout << bboxTransform << std::endl;

  Eigen::Vector3f p1(minPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p2(minPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p3(maxPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p4(maxPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p5(minPoint.x, maxPoint.y, minPoint.z);
  Eigen::Vector3f p6(minPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p7(maxPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p8(maxPoint.x, maxPoint.y, minPoint.z);

  std::cout << "Points - untransformed" << std::endl;
  std::cout << "P1" << std::endl << p1  << std::endl;
  std::cout << "P2" << std::endl << p2  << std::endl;
  std::cout << "P3" << std::endl << p3  << std::endl;
  std::cout << "P4" << std::endl << p4  << std::endl;
  std::cout << "P5" << std::endl << p5  << std::endl;
  std::cout << "P6" << std::endl << p6  << std::endl;
  std::cout << "P7" << std::endl << p7  << std::endl;
  std::cout << "P8" << std::endl << p8  << std::endl;

  // Transform back to world coordinates
  Eigen::Vector3f pr1 = bboxQuaternion * p1 + bboxTransform;
  Eigen::Vector3f pr2 = bboxQuaternion * p2 + bboxTransform;
  Eigen::Vector3f pr3 = bboxQuaternion * p3 + bboxTransform;
  Eigen::Vector3f pr4 = bboxQuaternion * p4 + bboxTransform;
  Eigen::Vector3f pr5 = bboxQuaternion * p5 + bboxTransform;
  Eigen::Vector3f pr6 = bboxQuaternion * p6 + bboxTransform;
  Eigen::Vector3f pr7 = bboxQuaternion * p7 + bboxTransform;
  Eigen::Vector3f pr8 = bboxQuaternion * p8 + bboxTransform;

  std::cout << std::endl << std::endl;
  std::cout << "Points with rotation/translation" << std::endl;
  std::cout << "P1" << std::endl << pr1  << std::endl;
  std::cout << "P2" << std::endl << pr2  << std::endl;
  std::cout << "P3" << std::endl << pr3  << std::endl;
  std::cout << "P4" << std::endl << pr4  << std::endl;
  std::cout << "P5" << std::endl << pr5  << std::endl;
  std::cout << "P6" << std::endl << pr6  << std::endl;
  std::cout << "P7" << std::endl << pr7  << std::endl;
  std::cout << "P8" << std::endl << pr8  << std::endl;

  //Project onto flat 2D space (in this case, x-z of the camera view)
  std::vector<Point2D> points;
  points.push_back({pr1[0], pr1[2]});
  points.push_back({pr2[0], pr2[2]});
  points.push_back({pr3[0], pr3[2]});
  points.push_back({pr4[0], pr4[2]});
  points.push_back({pr5[0], pr5[2]});
  points.push_back({pr6[0], pr6[2]});
  points.push_back({pr7[0], pr7[2]});
  points.push_back({pr8[0], pr8[2]});
  std::cout << "Input points are: " << std::endl;
  std::for_each(points.begin(), points.end(),
                [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

  std::vector<Point2D> resultLocal = testrobots::findConvexHull(points);
  std::cout << "Boundary points of convex hull (local) are: "<<endl;
  std::for_each(resultLocal.begin(), resultLocal.end(),
                [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

  //Transform these into global coordinates
  std::vector<Point2D> resultGlobal;
  for(auto pt : resultLocal) {
    //Based on where the camera is and where it is looking. Note that camera local coords are:
    //  X = perpendicular to the right, Y = in direction of view (out the front)
    double delta_x =        sin(camera_theta) * pt.x + cos(camera_theta) * pt.y;
    double delta_y = -1.0 * cos(camera_theta) * pt.x + sin(camera_theta) * pt.y;
    resultGlobal.push_back(Point2D{camera_x + delta_x, camera_y + delta_y});
  }
  assert(resultLocal.size() == resultGlobal.size());
  std::cout << "Boundary points of convex hull (global) are: "<<endl;
  std::for_each(resultGlobal.begin(), resultGlobal.end(),
                [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

  //Just for visulizing / debugging
  // if(visualizeBB) {
  //   // This viewer has 4 windows, but is only showing images in one of them as written here.
  //   int argc = 1;
  //   char** argv;
  //   pcl::visualization::PCLVisualizer *visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
  //   int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
  //   //visu->createViewPort(0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
  //   //visu->createViewPort(0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
  //   //visu->createViewPort(0.0, 0, 0.5, 0.5,  mesh_vp_3);
  //   //visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
  //   //visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
    
  //   visu->createViewPort(0.0, 0.0, 2.0, 2.0, mesh_vp_3);
  //   visu->addPointCloud(cloudSegmented, "bboxedCloud", mesh_vp_3);
  //   visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
    
  //   while(!visu->wasStopped())
  //   {
  //     visu->spinOnce();
  //   }
  // }

  return resultGlobal;
}
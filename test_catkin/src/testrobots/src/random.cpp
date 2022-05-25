// This file contains all the functions that we will need later but not right now


// we don't read from pcd
UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string& baseName,
                                                         const BoundingBox& yoloIdentifiedBoundingBox,
                                                         std::string pcdFilepath)
  : m_pipelineStepCount(0),
    m_baseName(baseName),
    m_boundingBox(yoloIdentifiedBoundingBox),
    m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>)

{
  //Read in the pointcloud from file
  pcl::PCDReader reader;
  reader.read(pcdFilepath, *m_cloud);
}


// *******************************************************************************************


double UNL_Robotics::SegmentationPipeline::calculateDepths() const  //of despair
{
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;

  //Get the Gaussian kernels for gaussian sampling
  UNL_Robotics::kernel_type gaussianKernel1 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS);  //Default mu/sigma values
  double mu = KERNEL_RADIUS;
  double sigma = 0.84089652;
  UNL_Robotics::kernel_type gaussianKernel2 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS, mu, sigma);  //Tighter Gaussian curve

  //Sampling

  //Get all the remaining point-cloud depths
  std::vector<double> PCDepths;
  for(unsigned i=0; i< m_cloud->size(); ++i)
    PCDepths.push_back(m_cloud->operator[](i).z);

  //Apply filter to remove the max and min values (removes n-samples from each side)
  std::vector<double> PCDepths_filtered = truncate_n_return_middle_vals(PCDepths, NUM_GAUSS_SAMPLE_PTS);
  std::vector<double> PCDepths_dblFiltered = truncate_n_return_middle_vals(PCDepths, 2*NUM_GAUSS_SAMPLE_PTS);  // 2x number of samples

  //Raw
  std::pair<double, double> PCDepthMeanStdev = calcMeanAndStdDev(PCDepths);
  cout << "   PCloud raw:" <<
    "   mean = " << PCDepthMeanStdev.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev.second << std::endl;

  //Max-min filtered
  std::pair<double, double> PCDepthMeanStdev_filtered = calcMeanAndStdDev(PCDepths_filtered);
  cout << "   PCloud filter:" <<
    "   mean = " << PCDepthMeanStdev_filtered.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev_filtered.second << std::endl;

  //Max-min filtered with 2x
  std::pair<double, double> PCDepthMeanStdev_dblFiltered = calcMeanAndStdDev(PCDepths_dblFiltered);
  cout << "   PCloud double filter:" <<
    "   mean = " << PCDepthMeanStdev_dblFiltered.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev_dblFiltered.second << std::endl;

  /*
  //Gauss weight 1
  cout << "   PCloud Gauss weight1 :" <<
    "   mean = " << std::accumulate(PCDepths_weighted1.begin(), PCDepths_weighted1.end(), 0.0) << std::endl;

  //Gauss weight 2
  cout << "   PCloud Gauss weight2 :" <<
    "   mean = " << std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0) << std::endl;
  */

  //Using the distance and pose, calculate the position of the object
  //double dist = std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0);
  //double objYaw = m_currentPose.yaw + calculateObjectAngleOffset();     //The object yaw is adjusted for its position within the image

  //Pose objectPose = { m_currentPose.x + dist * cos(objYaw),
  //                    m_currentPose.y + dist * sin(objYaw),
  //                    0.0 };
  //cout << "   Object pose (x,y,yaw) = " << objectPose << std::endl;

  return PCDepthMeanStdev.first;
}
***********************************************************************************************************************************************

********************************************************************************************************************************************
/// the functions below help in calculations in other functions


//*********************************************************


//************** from below not needed right now**********************************************************************************************************
void UNL_Robotics::SegmentationPipeline::removeOutliers(double meanK, double stddevMulThresh)
{
  // Remove outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedOutliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(m_cloud);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*cloud_removedOutliers);

  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount() << "_removedOutliers.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_removedOutliers, false);

  // Make this is our new working cloud
  copyPointCloud(*cloud_removedOutliers, *m_cloud);

  m_pipelineStepCount += 10;
}

void UNL_Robotics::SegmentationPipeline::performEuclideanExtraction()
{
  // Euclidean Cluster Extraction

  // Start by removing NaNs, if they exist
  pcl::PointCloud<pcl::PointXYZ>::Ptr nanlessCloud(new pcl::PointCloud<pcl::PointXYZ>);
  removeNaNs(m_cloud, nanlessCloud);
  copyPointCloud(*nanlessCloud, *m_cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(m_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  std::cout << "Cluster tolearance set to: " << CLUSTER_TOLERANCE << std::endl;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);

  std::cout << "Minimum cluster size set to: " << MIN_CLUSTER_SIZE << std::endl;
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);

  // This is set based on the bounding box size
  unsigned maxClusterSize = (m_boundingBox.xmax - m_boundingBox.xmin) * (m_boundingBox.ymax - m_boundingBox.ymin);
  ec.setMaxClusterSize(maxClusterSize);
  std::cout << "Maximum cluster size set to: " << maxClusterSize << std::endl;
  ec.setSearchMethod(tree);
  std::cout << "Set search method to tree" << std::endl;
  m_cloud->is_dense = false;
  ec.setInputCloud(m_cloud);
  std::cout << "Set input cloud" << std::endl;
  ec.extract(cluster_indices);
  std::cout << "   " << cluster_indices.size() << " cluster" << (cluster_indices.size() != 1 ? "s" : "")
            << " found in pointcloud." << std::endl;

  // For the file name, set the cluster number width (so that numbers are formatted as  07  if there are more than 10)
  unsigned clusterNumberWidth = floor(log10(cluster_indices.size())) + 1;

  // Loop over all the clusters found
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->push_back((*m_cloud)[*pit]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    std::cout << "    - Cluster extracted with " << cloud_cluster->size() << " data points." << std::endl;
    std::stringstream ss;
    ss << m_baseName << "_step" << printStepCount() << "_euclideanCluster_" << std::setfill('0')
       << std::setw(clusterNumberWidth) << std::distance(cluster_indices.begin(), it) << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
  }

  // Make the biggest cluster (cluster_indices[0]) our new working cloud
  if (cluster_indices.size() > 0)
  {
    copyPointCloud(*clusters[0], *m_cloud);

    /* Do this if we ever figure out how to get cluster extraction to work on the NaN-full cloud

      pcl::ExtractIndices<pcl::PointXYZ> clusterExtractor;
      clusterExtractor.setInputCloud(m_cloud);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster_indices[0]));
      clusterExtractor.setIndices(inliers);
      clusterExtractor.setNegative(true);
      //Keep the removed points as NaN values to maintain the structure of the cloud
      clusterExtractor.setKeepOrganized(true);

      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
      clusterExtractor.filter(*clusterCloud);
      copyPointCloud(*clusterCloud, *m_cloud);
    */
  }

  m_pipelineStepCount += 10;
}

//********************************************************************************************************************************************

std::vector<UNL_Robotics::Point2D> UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords(bool visualizeBB, double camera_x, double camera_y, double camera_theta)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = m_cloud;

  // Compute principal directions (equivalent to principal components / PCA)
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);

  // Two methods: compute Eigen vectors, or use a PCA object

  // Method 1
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // The following line is necessary for proper orientation in some cases.
  // The numbers come out the same without it, but the signs are
  //    different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  /*
     //Method 2
     // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCA<pcl::PointXYZ> pca;
     pca.setInputCloud(cloudSegmented);
     pca.project(*cloudSegmented, *cloudPCAprojection);
     Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
  */

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // For the transformation back to world coordinates
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount() << "_PCA.pcd";
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
  std::cout << "P1" << std::endl
            << p1 << std::endl;
  std::cout << "P2" << std::endl
            << p2 << std::endl;
  std::cout << "P3" << std::endl
            << p3 << std::endl;
  std::cout << "P4" << std::endl
            << p4 << std::endl;
  std::cout << "P5" << std::endl
            << p5 << std::endl;
  std::cout << "P6" << std::endl
            << p6 << std::endl;
  std::cout << "P7" << std::endl
            << p7 << std::endl;
  std::cout << "P8" << std::endl
            << p8 << std::endl;

  // Transform back to world coordinates
  Eigen::Vector3f pr1 = bboxQuaternion * p1 + bboxTransform;
  Eigen::Vector3f pr2 = bboxQuaternion * p2 + bboxTransform;
  Eigen::Vector3f pr3 = bboxQuaternion * p3 + bboxTransform;
  Eigen::Vector3f pr4 = bboxQuaternion * p4 + bboxTransform;
  Eigen::Vector3f pr5 = bboxQuaternion * p5 + bboxTransform;
  Eigen::Vector3f pr6 = bboxQuaternion * p6 + bboxTransform;
  Eigen::Vector3f pr7 = bboxQuaternion * p7 + bboxTransform;
  Eigen::Vector3f pr8 = bboxQuaternion * p8 + bboxTransform;

  std::cout << std::endl
            << std::endl;
  std::cout << "Points with rotation/translation" << std::endl;
  std::cout << "P1" << std::endl
            << pr1 << std::endl;
  std::cout << "P2" << std::endl
            << pr2 << std::endl;
  std::cout << "P3" << std::endl
            << pr3 << std::endl;
  std::cout << "P4" << std::endl
            << pr4 << std::endl;
  std::cout << "P5" << std::endl
            << pr5 << std::endl;
  std::cout << "P6" << std::endl
            << pr6 << std::endl;
  std::cout << "P7" << std::endl
            << pr7 << std::endl;
  std::cout << "P8" << std::endl
            << pr8 << std::endl;

  // Project onto flat 2D space (in this case, x-z of the camera view)
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
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  std::vector<Point2D> resultLocal = findConvexHull(points);
  std::cout << "Boundary points of convex hull (local) are: " << endl;
  std::for_each(resultLocal.begin(), resultLocal.end(),
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  // Transform these into global coordinates
  std::vector<Point2D> resultGlobal;
  for (auto pt : resultLocal)
  {
    // Based on where the camera is and where it is looking. Note that camera local coords are:
    //   X = perpendicular to the right, Y = in direction of view (out the front)
    double delta_x = sin(camera_theta) * pt.x + cos(camera_theta) * pt.y;
    double delta_y = -1.0 * cos(camera_theta) * pt.x + sin(camera_theta) * pt.y;
    resultGlobal.push_back(Point2D{camera_x + delta_x, camera_y + delta_y});
  }
  assert(resultLocal.size() == resultGlobal.size());
  std::cout << "Boundary points of convex hull (global) are: " << endl;
  std::for_each(resultGlobal.begin(), resultGlobal.end(),
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  // Just for visulizing / debugging
  if (visualizeBB)
  {
    // This viewer has 4 windows, but is only showing images in one of them as written here.
    int argc = 1;
    char **argv;
    pcl::visualization::PCLVisualizer *visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
    int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    // visu->createViewPort(0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    // visu->createViewPort(0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    // visu->createViewPort(0.0, 0, 0.5, 0.5,  mesh_vp_3);
    // visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
    // visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);

    visu->createViewPort(0.0, 0.0, 2.0, 2.0, mesh_vp_3);
    visu->addPointCloud(cloudSegmented, "bboxedCloud", mesh_vp_3);
    visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);

    while (!visu->wasStopped())
    {
      visu->spinOnce();
    }
  }

  return resultGlobal;
}

void UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta)
{
  typedef pcl::PointXYZ PointType;

  pcl::PointCloud<PointType>::Ptr cloud = m_cloud;

  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // correct vertical between main directions
  eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
  eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

  std::cout << "Eigenvalue va(3x1):\n"
            << eigenValuesPCA << std::endl;
  std::cout << "Feature vector ve(3x3):\n"
            << eigenVectorsPCA << std::endl;
  std::cout << "centroid point (4x1):\n"
            << pcaCentroid << std::endl;
  /*
   // Another way to calculate the eigenvalues ​​and eigenvectors of the point cloud covariance matrix: through the pca interface in PCL as follows
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloudSegmented);
  pca.project(*cloudSegmented, *cloudPCAprojection);
   std::cerr << std::endl << "EigenVectors: "<< pca.getEigenVectors() << std::endl;//Calculate the feature vector
   std::cerr << std::endl << "EigenValues: "<< pca.getEigenValues() << std::endl;//Calculate characteristic values
  */
  Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
  tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();                                     // R.
  tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>()); //  -R*t
  tm_inv = tm.inverse();

  std::cout << "Transformation matrix tm(4x4):\n"
            << tm << std::endl;
  std::cout << "inverter matrix tm'(4x4):\n"
            << tm_inv << std::endl;

  pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*cloud, *transformedCloud, tm);

  PointType min_p1, max_p1;
  Eigen::Vector3f c1, c;
  pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
  c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

  std::cout << "Centre c1(3x1):\n"
            << c1 << std::endl;

  Eigen::Affine3f tm_inv_aff(tm_inv);
  pcl::transformPoint(c1, c, tm_inv_aff);

  Eigen::Vector3f whd, whd1;
  whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
  whd = whd1;
  float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; // The average scale of the point cloud, used to set the size of the main direction arrow

  std::cout << "width1=" << whd1(0) << endl;
  std::cout << "heght1=" << whd1(1) << endl;
  std::cout << "depth1=" << whd1(2) << endl;
  std::cout << "scale1=" << sc1 << endl;

  const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
  const Eigen::Vector3f bboxT1(c1);

  const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
  const Eigen::Vector3f bboxT(c);

  // The main direction of the point cloud transformed to the origin
  PointType op;
  op.x = 0.0;
  op.y = 0.0;
  op.z = 0.0;
  Eigen::Vector3f px, py, pz;
  Eigen::Affine3f tm_aff(tm);
  pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
  pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
  pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
  PointType pcaX;
  pcaX.x = sc1 * px(0);
  pcaX.y = sc1 * px(1);
  pcaX.z = sc1 * px(2);
  PointType pcaY;
  pcaY.x = sc1 * py(0);
  pcaY.y = sc1 * py(1);
  pcaY.z = sc1 * py(2);
  PointType pcaZ;
  pcaZ.x = sc1 * pz(0);
  pcaZ.y = sc1 * pz(1);
  pcaZ.z = sc1 * pz(2);

  // The main direction of the initial point cloud
  PointType cp;
  cp.x = pcaCentroid(0);
  cp.y = pcaCentroid(1);
  cp.z = pcaCentroid(2);
  PointType pcX;
  pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
  pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
  pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
  PointType pcY;
  pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
  pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
  pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
  PointType pcZ;
  pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
  pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
  pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

  // visualization
  pcl::visualization::PCLVisualizer viewer;

  pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); // Point cloud related to the origin
  viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
  viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");

  viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
  viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
  viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

  /*
   pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0); //The initial point cloud input is related
  viewer.addPointCloud(cloud, color_handler, "cloud");
  viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");

  viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
  viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
  viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
  */

  viewer.addCoordinateSystem(0.5f * sc1);
  viewer.setBackgroundColor(1.0, 1.0, 1.0);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }
}



//***********************************************************************************************************************************************



// why is this function used and what does it do ???????
void UNL_Robotics::SegmentationPipeline::resetCloudToPostPlaneExtractionAndBB(const std::string &newBaseName,
                                                                              const BoundingBox &newBoundingBox)
{
  setBaseName(newBaseName);
  setBoundingBox(newBoundingBox);
  resetCloudToPostPlaneExtraction();
}

// why are we using this - well this function is used in depth calculation in and Gaussian Calculation
double UNL_Robotics::SegmentationPipeline::calculateObjectAngleOffset() const
{
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;

  // Calculate the angle offset of the picture relative to the center of the view port
  unsigned x_centerBB = m_boundingBox.xmin + static_cast<unsigned>(x_delta / 2);
  unsigned y_centerBB = m_boundingBox.ymin + static_cast<unsigned>(y_delta / 2);
  int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH / 2) - x_centerBB; // Can be negative! This orientation assumes CCW=+
  double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

  std::cout << "   "
            << "Bounding Box (x,y):"
            << "   Min = (" << m_boundingBox.xmin << ", " << m_boundingBox.ymin << ")"
            << "   Max = (" << m_boundingBox.xmax << ", " << m_boundingBox.ymax << ")"
            << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
  std::cout << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;

  return objectAngleOffset;
}

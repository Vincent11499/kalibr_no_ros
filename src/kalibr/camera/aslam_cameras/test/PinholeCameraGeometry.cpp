// Bring in gtest
#include <gtest/gtest.h>
#include <limits>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

namespace {

aslam::cameras::GridCalibrationTargetObservation makeCircleGridObservation(
    bool partial) {
  using aslam::cameras::GridCalibrationTargetCheckerboard;
  using aslam::cameras::GridCalibrationTargetObservation;

  GridCalibrationTargetCheckerboard::Ptr target(
      new GridCalibrationTargetCheckerboard(8, 8, 0.04, 0.04));
  GridCalibrationTargetObservation observation(
      target, cv::Mat::zeros(1200, 2000, CV_8UC1));

  const cv::Point2d firstVanishingPoint(250.0, 300.0);
  const cv::Point2d secondVanishingPoint(1500.0, 300.0);
  const double centerX =
      0.5 * (firstVanishingPoint.x + secondVanishingPoint.x);
  for (size_t row = 0; row < target->rows(); ++row) {
    const cv::Point2d center(centerX, -700.0 + 200.0 * row);
    const double firstAngle =
        std::atan2(firstVanishingPoint.y - center.y,
                   firstVanishingPoint.x - center.x);
    const double rawSecondAngle =
        std::atan2(secondVanishingPoint.y - center.y,
                   secondVanishingPoint.x - center.x);
    double angleDifference = rawSecondAngle - firstAngle;
    while (angleDifference > M_PI) {
      angleDifference -= 2.0 * M_PI;
    }
    while (angleDifference < -M_PI) {
      angleDifference += 2.0 * M_PI;
    }
    const double radius = cv::norm(firstVanishingPoint - center);
    for (size_t column = 0; column < target->cols(); ++column) {
      if (partial && (column == 0 || column + 1 == target->cols())) {
        continue;
      }
      const double fraction =
          static_cast<double>(column) /
          static_cast<double>(target->cols() - 1);
      const double angle = firstAngle + fraction * angleDifference;
      observation.updateImagePoint(
          target->gridCoordinatesToPoint(row, column),
          Eigen::Vector2d(center.x + radius * std::cos(angle),
                          center.y + radius * std::sin(angle)));
    }
  }
  return observation;
}

}  // namespace

TEST(AslamCamerasTestSuite, testPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<PinholeCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDefaultDistortedPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DistortedPinholeCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedPinholeCameraGeometry)
{
  using namespace aslam::cameras;

  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedPinholeCameraGeometry geometry = DistortedPinholeCameraGeometry::getTestGeometry();
  geometry.projection().distortion() = d;

  CameraGeometryTestHarness<DistortedPinholeCameraGeometry> harness(geometry, 1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testPinholeRsCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<PinholeRsCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDefaultDistortedRsPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DistortedPinholeRsCameraGeometry> harness(2e-2);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedPinholeRsCameraGeometry)
{
  using namespace aslam::cameras;

  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedPinholeRsCameraGeometry geometry = DistortedPinholeRsCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(d);

  CameraGeometryTestHarness<DistortedPinholeRsCameraGeometry> harness(geometry, 2e-2);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testPartialGridPinholeInitialization)
{
  using namespace aslam::cameras;
  GridCalibrationTargetObservation observation =
      makeCircleGridObservation(true);
  std::vector<GridCalibrationTargetObservation> observations(1, observation);

  PinholeProjection<NoDistortion> projection;
  ASSERT_TRUE(projection.initializeIntrinsics(observations, 0.75));

  Eigen::MatrixXd parameters;
  projection.getParameters(parameters);
  const double expectedFocalLength = 1250.0 / M_PI;
  EXPECT_NEAR(expectedFocalLength, parameters(0, 0), 1.0e-8);
  EXPECT_NEAR(expectedFocalLength, parameters(1, 0), 1.0e-8);
  EXPECT_DOUBLE_EQ(999.5, parameters(2, 0));
  EXPECT_DOUBLE_EQ(599.5, parameters(3, 0));
}

TEST(AslamCamerasTestSuite, testVisibleRatioOnePreservesLegacyInitialization)
{
  using namespace aslam::cameras;
  GridCalibrationTargetObservation observation =
      makeCircleGridObservation(false);
  std::vector<GridCalibrationTargetObservation> observations(1, observation);

  PinholeProjection<NoDistortion> legacyProjection;
  PinholeProjection<NoDistortion> ratioProjection;
  ASSERT_TRUE(legacyProjection.initializeIntrinsics(observations));
  ASSERT_TRUE(ratioProjection.initializeIntrinsics(observations, 1.0));

  Eigen::MatrixXd legacyParameters;
  Eigen::MatrixXd ratioParameters;
  legacyProjection.getParameters(legacyParameters);
  ratioProjection.getParameters(ratioParameters);
  ASSERT_EQ(legacyParameters.rows(), ratioParameters.rows());
  ASSERT_EQ(legacyParameters.cols(), ratioParameters.cols());
  for (Eigen::Index row = 0; row < legacyParameters.rows(); ++row) {
    for (Eigen::Index column = 0; column < legacyParameters.cols(); ++column) {
      EXPECT_DOUBLE_EQ(legacyParameters(row, column),
                       ratioParameters(row, column));
    }
  }
}

TEST(AslamCamerasTestSuite, testCheckedCircleRejectsDegenerateLine)
{
  std::vector<cv::Point2d> points;
  for (int index = 0; index < 8; ++index) {
    points.emplace_back(100.0 * index, 50.0 * index + 20.0);
  }

  cv::Point2d center;
  double radius = 0.0;
  EXPECT_FALSE(aslam::cameras::PinholeHelpers::fitCircleChecked(
      points, center, radius));
}

TEST(AslamCamerasTestSuite, testPartialGridRejectsInvalidRatio)
{
  using namespace aslam::cameras;
  GridCalibrationTargetObservation observation =
      makeCircleGridObservation(true);
  std::vector<GridCalibrationTargetObservation> observations(1, observation);
  PinholeProjection<NoDistortion> projection;

  EXPECT_THROW(projection.initializeIntrinsics(observations, 0.0),
               std::runtime_error);
  EXPECT_THROW(projection.initializeIntrinsics(
                   observations,
                   std::numeric_limits<double>::quiet_NaN()),
               std::runtime_error);
}

// TEST(AslamCamerasTestSuite, testDistortedPinholeCameraGeometry) {

//     std::cout << "pinholetest";

//     // test distortion and distored jacobian:
//     aslam::cameras::PinholeCameraGeometry pcg( aslam::cameras::PinholeCameraGeometry::createDistortedTestGeometry() );
//     Eigen::Vector3d p = pcg.createRandomVisiblePoint();
//     Eigen::Vector2d k = pcg.createRandomKeypoint();

//     Eigen::MatrixXd J;
//     Eigen::MatrixXd estJ;

//     Eigen::Vector2d  k1 = pcg.euclideanToKeypoint(p);
//     Eigen::Vector2d  k2 = pcg.euclideanToKeypoint(p, J);
//     sm::eigen::assertEqual(k1,k2, SM_SOURCE_FILE_POS);
//     pcg.euclideanToKeypointFiniteDifference(p,estJ);
//     sm::eigen::assertNear(J,estJ, 1e-5, SM_SOURCE_FILE_POS);

//     // Project to the normalized plane.
//     double mx_u = p[0]/p[2];
//     double my_u = p[1]/p[2];

//     // Apply distortion.
//     double mx_d, my_d;
//     pcg.distortion(mx_u, my_u, &mx_d, &my_d);  
//     // Distortion only puts out deltas.
//     mx_d += mx_u;
//     my_d += my_u;
//     double hat_mx_u, hat_my_u;
//     pcg.undistortGN(mx_d, my_d, &hat_mx_u, &hat_my_u) ;

//     // Check if the error is small...
//     double hat_mx_d, hat_my_d;
//     pcg.distortion(hat_mx_u, hat_my_u, &hat_mx_d, &hat_my_d);  

//     EXPECT_NEAR(hat_mx_u + hat_mx_d, mx_d, 1e-10);
//     EXPECT_NEAR(hat_my_u + hat_my_d, my_d, 1e-10);

//     double ax,bx,cx,dx;
//     pcg.distortion(hat_mx_u, hat_my_u, &hat_mx_d, &hat_my_d, &ax, &bx, &cx, &dx);  

//     EXPECT_NEAR(hat_mx_u + hat_mx_d, mx_d, 1e-10);
//     EXPECT_NEAR(hat_my_u + hat_my_d, my_d, 1e-10);

//     EXPECT_NEAR(hat_mx_u, mx_u, 1e-10);
//     EXPECT_NEAR(hat_my_u, my_u, 1e-10);

// }

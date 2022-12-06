#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct UndistortCalibratorSettings final {
    cv::Size boardSize;           // The size of the board -> Number of items by width and height
    float squareSize;             // The size of a square in your defined unit (point, millimeter,etc).
    bool flipVertical;            // Flip the captured images around the horizontal axis
    int nrFrames;                 // The number of frames to use from the input for calibration
    float aspectRatio;            // The aspect ratio
    bool calibZeroTangentDist;    // Assume zero tangential distortion
    bool calibFixPrincipalPoint;  // Fix the principal point at the center
    bool writePoints;             // Write detected feature points
    bool writeExtrinsics;         // Write extrinsic parameters
    bool writeGrid;               // Write refined 3D target grid points
    bool showUndistorted;         // Show undistorted images after calibration
    bool fixK1;                   // fix K1 distortion coefficient
    bool fixK2;                   // fix K2 distortion coefficient
    bool fixK3;                   // fix K3 distortion coefficient
    bool fixK4;                   // fix K4 distortion coefficient
    bool fixK5;                   // fix K5 distortion coefficient
    int winSize;                  // Half of search window for cornerSubPix
};

template <class Inspector>
bool inspect(Inspector& f, UndistortCalibratorSettings& x) {
    return f.object(x).fields(
        f.field("boardSizeWidth", x.boardSize.width), f.field("boardSizeHeight", x.boardSize.height),
        f.field("squareSize", x.squareSize), f.field("flipVertical", x.flipVertical), f.field("nrFrames", x.nrFrames),
        f.field("aspectRatio", x.aspectRatio), f.field("calibZeroTangentDist", x.calibZeroTangentDist),
        f.field("calibFixPrincipalPoint", x.calibFixPrincipalPoint), f.field("writePoints", x.writePoints),
        f.field("writeExtrinsics", x.writeExtrinsics), f.field("writeGrid", x.writeGrid),
        f.field("showUndistorted", x.showUndistorted), f.field("fixK1", x.fixK1), f.field("fixK2", x.fixK2),
        f.field("fixK3", x.fixK3), f.field("fixK4", x.fixK4), f.field("fixK5", x.fixK5), f.field("winSize", x.winSize));
}

enum class Status { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

class UndistortCalibrator final : public HubHelper<caf::event_based_actor, UndistortCalibratorSettings, image_frame_atom> {
    Identifier mKey;
    int32_t mFlag = 0;
    float mGridWidth;
    bool mReleaseObject = false;
    std::vector<std::vector<cv::Point2f>> mImagePoints;
    cv::Mat mCameraMatrix, mDistCoeffs;
    cv::Size mImageSize;
    Status mMode;

    void initFlag() {
        if(mConfig.calibFixPrincipalPoint)
            mFlag |= cv::CALIB_FIX_PRINCIPAL_POINT;
        if(mConfig.calibZeroTangentDist)
            mFlag |= cv::CALIB_ZERO_TANGENT_DIST;
        if(mConfig.aspectRatio)
            mFlag |= cv::CALIB_FIX_ASPECT_RATIO;
        if(mConfig.fixK1)
            mFlag |= cv::CALIB_FIX_K1;
        if(mConfig.fixK2)
            mFlag |= cv::CALIB_FIX_K2;
        if(mConfig.fixK3)
            mFlag |= cv::CALIB_FIX_K3;
        if(mConfig.fixK4)
            mFlag |= cv::CALIB_FIX_K4;
        if(mConfig.fixK5)
            mFlag |= cv::CALIB_FIX_K5;
    }

    double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                     const std::vector<std::vector<cv::Point2f>>& imagePoints, const std::vector<cv::Mat>& rvecs,
                                     const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                     std::vector<float>& perViewErrors) {
        std::vector<cv::Point2f> imagePoints2;
        size_t totalPoints = 0;
        double totalErr = 0;
        perViewErrors.resize(objectPoints.size());

        for(size_t i = 0; i < objectPoints.size(); ++i) {
            cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
            const double err = cv::norm(imagePoints[i], imagePoints2, cv::NORM_L2);

            const size_t n = objectPoints[i].size();
            perViewErrors[i] = static_cast<float>(std::sqrt(err * err / static_cast<double>(n)));
            totalErr += err * err;
            totalPoints += n;
        }

        return std::sqrt(totalErr / static_cast<double>(totalPoints));
    }

    void calcBoardCornerPositions(const cv::Size boardSize, const float squareSize, std::vector<cv::Point3f>& corners) {
        corners.clear();
        for(int i = 0; i < boardSize.height; ++i) {
            for(int j = 0; j < boardSize.width; ++j)
                corners.emplace_back(static_cast<float>(j) * squareSize, static_cast<float>(i) * squareSize, 0.0f);
        }
    }

    bool runCalibration(std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs,
                        double& totalAvgErr, std::vector<cv::Point3f>& newObjPoints) {
        mCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        if(mFlag & cv::CALIB_FIX_ASPECT_RATIO)
            mCameraMatrix.at<double>(0, 0) = mConfig.aspectRatio;
        mDistCoeffs = cv::Mat::zeros(8, 1, CV_64F);

        std::vector<std::vector<cv::Point3f>> objectPoints(1);
        calcBoardCornerPositions(mConfig.boardSize, mConfig.squareSize, objectPoints[0]);
        objectPoints[0][mConfig.boardSize.width - 1].x = objectPoints[0][0].x + mGridWidth;
        newObjPoints = objectPoints[0];

        objectPoints.resize(mImagePoints.size(), objectPoints[0]);

        int iFixedPoint = -1;
        if(mReleaseObject)
            iFixedPoint = mConfig.boardSize.width - 1;
        double rms = cv::calibrateCameraRO(objectPoints, mImagePoints, mImageSize, iFixedPoint, mCameraMatrix, mDistCoeffs, rvecs,
                                           tvecs, newObjPoints, mFlag | cv::CALIB_USE_LU);

        if(mReleaseObject) {
            //  logInfo("New board corners: ");
            //  logInfo(newObjPoints[0]);
            //  logInfo(newObjPoints[mConfig.boardSize.width - 1]);
            //  logInfo(newObjPoints[mConfig.boardSize.width * (mConfig.boardSize.height - 1)]);
            //  logInfo(newObjPoints.back());
        }

        logInfo("Re-projection error reported by calibrateCamera: " + std::to_string(rms));

        const bool ok = cv::checkRange(mCameraMatrix) && cv::checkRange(mDistCoeffs);

        objectPoints.clear();
        objectPoints.resize(mImagePoints.size(), newObjPoints);
        totalAvgErr = computeReprojectionErrors(objectPoints, mImagePoints, rvecs, tvecs, mCameraMatrix, mDistCoeffs, reprojErrs);

        return ok;
    }

    // Print camera parameters to the output file
    void saveCameraParams(const std::string& identifier, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                          const std::vector<float>& reprojErrs, double totalAvgErr,
                          const std::vector<cv::Point3f>& newObjPoints) {
        const auto outputFileName = "./data/camera_calibration/" + identifier + ".xml";

        cv::FileStorage fs(outputFileName, cv::FileStorage::WRITE);

        if(!rvecs.empty() || !reprojErrs.empty())
            fs << "nr_of_frames" << static_cast<int>(std::max(rvecs.size(), reprojErrs.size()));

        fs << "image_width" << mImageSize.width;
        fs << "image_height" << mImageSize.height;
        fs << "board_width" << mConfig.boardSize.width;
        fs << "board_height" << mConfig.boardSize.height;
        fs << "square_size" << mConfig.squareSize;

        if(mFlag & cv::CALIB_FIX_ASPECT_RATIO)
            fs << "fix_aspect_ratio" << mConfig.aspectRatio;

        fs << "camera_matrix" << mCameraMatrix;
        fs << "distortion_coefficients" << mDistCoeffs;

        fs << "avg_reprojection_error" << totalAvgErr;
        if(mConfig.writeExtrinsics && !reprojErrs.empty())
            fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

        if(mConfig.writeExtrinsics && !rvecs.empty() && !tvecs.empty()) {
            CV_Assert(rvecs[0].type() == tvecs[0].type());
            cv::Mat bigMat(static_cast<int>(rvecs.size()), 6, CV_MAKETYPE(rvecs[0].type(), 1));
            bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
            bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

            for(size_t i = 0; i < rvecs.size(); i++) {
                cv::Mat r = bigMat(cv::Range(static_cast<int>(i), static_cast<int>(i + 1)), cv::Range(0, 3));
                cv::Mat t = bigMat(cv::Range(static_cast<int>(i), static_cast<int>(i + 1)), cv::Range(3, 6));

                if(needReshapeR)
                    rvecs[i].reshape(1, 1).copyTo(r);
                else {
                    //*.t() is MatExpr (not Mat) so we can use assignment operator
                    CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                    r = rvecs[i].t();
                }

                if(needReshapeT)
                    tvecs[i].reshape(1, 1).copyTo(t);
                else {
                    CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                    t = tvecs[i].t();
                }
            }
            fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
            fs << "extrinsic_parameters" << bigMat;
        }

        if(mConfig.writePoints && !mImagePoints.empty()) {
            cv::Mat imagePtMat(static_cast<int>(mImagePoints.size()), static_cast<int>(mImagePoints[0].size()), CV_32FC2);
            for(size_t i = 0; i < mImagePoints.size(); i++) {
                cv::Mat r = imagePtMat.row(static_cast<int>(i)).reshape(2, imagePtMat.cols);
                cv::Mat imgPoint(mImagePoints[i]);
                imgPoint.copyTo(r);
            }
            fs << "image_points" << imagePtMat;
        }

        if(mConfig.writeGrid && !newObjPoints.empty()) {
            fs << "grid_points" << newObjPoints;
        }
    }

    bool runCalibrationAndSave(const std::string& identifier) {
        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<float> reprojErrs;
        double totalAvgErr = 0;
        std::vector<cv::Point3f> newObjPoints;

        const bool ok = runCalibration(rvecs, tvecs, reprojErrs, totalAvgErr, newObjPoints);

        logInfo((ok ? "Calibration succeeded" : "Calibration failed"));
        logInfo("avg re projection error =" + std::to_string(totalAvgErr));

        if(ok)
            saveCameraParams(identifier, rvecs, tvecs, reprojErrs, totalAvgErr, newObjPoints);
        return ok;
    }

public:
    UndistortCalibrator(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mMode(Status::CAPTURING) {
        initFlag();
        mGridWidth = mConfig.squareSize * static_cast<float>(mConfig.boardSize.width - 1);
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     const auto res = BlackBoard::instance().get<CameraFrame>(key).value();

                     //-----  If no more image, or got enough, then stop calibration and show result -------------
                     if(mMode == Status::CAPTURING && mImagePoints.size() >= static_cast<size_t>(mConfig.nrFrames)) {
                         if(runCalibrationAndSave(res.info.identifier))
                             mMode = Status::CALIBRATED;
                         else
                             mMode = Status::DETECTION;
                     }

                     mImageSize = res.frame.size();
                     if(mConfig.flipVertical)
                         cv::flip(res.frame, res.frame, 0);

                     std::vector<cv::Point2f> pointBuf;
                     int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

                     // Find feature points on the input format
                     bool found = cv::findChessboardCorners(res.frame, mConfig.boardSize, pointBuf, chessBoardFlags);

                     //! [pattern_found]
                     if(found) {
                         // improve the found corners' coordinate accuracy for chessboard
                         cv::Mat imgGray;
                         cv::cvtColor(res.frame, imgGray, cv::COLOR_BGR2GRAY);
                         cornerSubPix(imgGray, pointBuf, cv::Size(mConfig.winSize, mConfig.winSize), cv::Size(-1, -1),
                                      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
                         mImagePoints.push_back(pointBuf);
                         // Draw the corners
                         cv::drawChessboardCorners(res.frame, mConfig.boardSize, cv::Mat(pointBuf), found);
                     }

                     //----------------------------- Output Text ------------------------------------------------
                     //! [output_text]
                     std::string msg = (mMode == Status::CAPTURING) ? "100/100" :
                         (mMode == Status::CALIBRATED)              ? "Calibrated" :
                                                                      "Detected";
                     int baseLine = 0;
                     cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
                     cv::Point textOrigin(res.frame.cols - 2 * textSize.width - 10, res.frame.rows - 2 * baseLine - 10);

                     if(mMode == Status::CAPTURING) {
                         if(mConfig.showUndistorted)
                             msg = cv::format("%d/%d Undistort", static_cast<int>(mImagePoints.size()), mConfig.nrFrames);
                         else
                             msg = cv::format("%d/%d", static_cast<int>(mImagePoints.size()), mConfig.nrFrames);
                     }

                     cv::putText(res.frame, msg, textOrigin, 1, 1, cv::Scalar(0, 255, 0));

                     //-------------------------output  undistorted ------------------------------
                     //! [output_undistorted]
                     if(mMode == Status::CALIBRATED && mConfig.showUndistorted) {
                         cv::Mat temp = res.frame.clone();
                         cv::undistort(temp, res.frame, mCameraMatrix, mDistCoeffs);
                     }
                     // For debugging
                     sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(UndistortCalibrator);

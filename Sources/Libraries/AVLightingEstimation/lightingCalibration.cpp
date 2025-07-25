// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVLightingEstimation/lightingCalibration.hpp>
#include <AVLightingEstimation/lightingEstimation.hpp>
#include <AVLightingEstimation/ellipseGeometry.hpp>

#include <AVImage/all.hpp>
#include <AVImage/io.hpp>
#include <AVImage/convolution.hpp>
#include <AVPhotometricStereo/photometricStereo.hpp>
#include <AVPhotometricStereo/photometricDataIO.hpp>

#include <AVSfMDataIO/sfmDataIO.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <filesystem>
#include <math.h>

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace lightingEstimation {

void lightCalibration(const sfmData::SfMData& sfmData,
                      const std::string& inputFile,
                      const std::string& outputPath,
                      const std::string& method,
                      const bool doDebug,
                      const bool saveAsModel,
                      const bool ellipticEstimation)
{
    std::vector<std::string> imageList;
    std::vector<float> focals;

    std::map<std::string, sfmData::View> viewMap;
    for (auto& viewIt : sfmData.getViews())
    {
        std::map<std::string, std::string> currentMetadata = sfmData.getView(viewIt.first).getImage().getMetadata();

        if (currentMetadata.find("Exif:DateTimeDigitized") == currentMetadata.end())
        {
            std::cout << "No metadata case" << std::endl;
            viewMap[sfmData.getView(viewIt.first).getImage().getImagePath()] = sfmData.getView(viewIt.first);
        }
        else
        {
            viewMap[currentMetadata.at("Exif:DateTimeDigitized")] = sfmData.getView(viewIt.first);
        }
    }

    std::vector<std::array<float, 3>> allSpheresParams;
    std::vector<Eigen::Matrix3f> KMatrices;

    int nbCols = 0;
    int nbRows = 0;

    // Main tree
    bpt::ptree fileTree;
    // Read the json file and initialize the tree
    bpt::read_json(inputFile, fileTree);

    for (const auto& [currentId, currentView] : viewMap)
    {
        ALICEVISION_LOG_INFO("View Id: " << currentView.getViewId());
        const fs::path imagePath = fs::path(currentView.getImage().getImagePath());

        if (!boost::algorithm::icontains(imagePath.stem().string(), "ambient"))
        {
            std::string sphereName = std::to_string(currentView.getViewId());
            auto sphereExists = (fileTree.get_child_optional(sphereName)).is_initialized();
            if (sphereExists)
            {
                ALICEVISION_LOG_INFO("  - " << imagePath.string());
                imageList.push_back(imagePath.string());

                std::array<float, 3> currentSphereParams;
                for (auto& currentSphere : fileTree.get_child(sphereName))
                {
                    currentSphereParams[0] = currentSphere.second.get_child("").get("x", 0.0);
                    currentSphereParams[1] = currentSphere.second.get_child("").get("y", 0.0);
                    currentSphereParams[2] = currentSphere.second.get_child("").get("r", 0.0);
                }
                allSpheresParams.push_back(currentSphereParams);

                IndexT intrinsicId = currentView.getIntrinsicId();
                focals.push_back(sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(0));

                float focalPx = sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(0);
                nbCols = sfmData.getIntrinsics().at(intrinsicId)->w();
                nbRows = sfmData.getIntrinsics().at(intrinsicId)->h();
                float x_p = (nbCols) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(2);
                float y_p = (nbRows) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(3);

                Eigen::MatrixXf currentK = Eigen::MatrixXf::Zero(3, 3);
                // Create K matrix
                currentK << focalPx, 0.0, x_p, 0.0, focalPx, y_p, 0.0, 0.0, 1.0;

                KMatrices.push_back(currentK);
            }
            else
            {
                ALICEVISION_LOG_WARNING("No detected sphere found for '" << imagePath << "'.");
            }
        }
    }

    int lightSize = 3;
    if (!method.compare("SH"))
        lightSize = 9;

    Eigen::MatrixXf lightMat(imageList.size(), lightSize);
    std::vector<float> intList;

    for (size_t i = 0; i < imageList.size(); ++i)
    {
        std::string picturePath = imageList.at(i);
        Eigen::VectorXf lightingDirection = Eigen::VectorXf::Zero(lightSize);
        float intensity;

        float focal = focals.at(i);
        std::array<float, 3> sphereParam = allSpheresParams.at(i);
        if (ellipticEstimation)
        {
            Eigen::Matrix3f K = KMatrices.at(i);
            float sphereRadius = 1.0;
            std::array<float, 5> ellipseParam;

            cv::Mat maskCV = cv::Mat::zeros(nbRows, nbCols, CV_8UC1);

            image::Image<float> imageFloat;
            image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);
            getEllipseMaskFromSphereParameters(sphereParam, K, ellipseParam);
            calibrateLightFromRealSphere(imageFloat, maskCV, K, sphereRadius, method, lightingDirection, intensity);
        }
        else
        {
            lightCalibrationOneImage(picturePath, sphereParam, focal, method, lightingDirection, intensity);
        }

        lightMat.row(i) = lightingDirection;
        intList.push_back(intensity);

        if (doDebug)
        {
            int outputSize = 1024;
            std::string outputFileName =
              fs::path(outputPath).parent_path().string() + "/" + fs::path(picturePath).stem().string() + "_" + method + ".png";
            sphereFromLighting(lightingDirection, lightingDirection.norm(), outputFileName, outputSize);
        }
    }

    // Write in JSON file
    writeJSON(outputPath, sfmData, imageList, lightMat, intList, saveAsModel, method);
}

void lightCalibrationOneImage(const std::string& picturePath,
                              const std::array<float, 3>& sphereParam,
                              const float focal,
                              const std::string& method,
                              Eigen::VectorXf& lightingDirection,
                              float& intensity)
{
    // Read picture :
    image::Image<float> imageFloat;
    image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

    // If method = brightest point :
    if (!method.compare("brightestPoint"))
    {
        // Detect brightest point :
        Eigen::Vector2f brigthestPoint;
        detectBrightestPoint(sphereParam, imageFloat, brigthestPoint);

        Eigen::Vector2f brigthestPoint_xy;
        brigthestPoint_xy(0) = brigthestPoint(0) - imageFloat.cols() / 2;
        brigthestPoint_xy(1) = brigthestPoint(1) - imageFloat.rows() / 2;

        Eigen::Vector3f normalBrightestPoint;
        getNormalOnSphere(brigthestPoint_xy(0), brigthestPoint_xy(1), sphereParam, normalBrightestPoint);

        // Observation direction :
        Eigen::Vector3f observationRayPersp;
        Eigen::Vector3f observationRay;

        // orthographic approximation :
        observationRay(0) = 0.0;
        observationRay(1) = 0.0;
        observationRay(2) = -1.0;

        observationRayPersp(0) = brigthestPoint_xy(0) / focal;
        observationRayPersp(1) = brigthestPoint_xy(1) / focal;
        observationRayPersp(2) = 1.0;
        observationRayPersp = -observationRayPersp / observationRayPersp.norm();

        // Evaluate lighting direction :
        // lightingDirection = 2 * normalBrightestPoint.dot(observationRayPersp) * normalBrightestPoint - observationRayPersp;
        // lightingDirection = lightingDirection / lightingDirection.norm();

        lightingDirection = 2 * normalBrightestPoint.dot(observationRay) * normalBrightestPoint - observationRay;

        intensity = 1.0;
    }
    // If method = whiteSphere :
    else if (!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse
        const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
        const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

        const float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        const int nbPixelsPatch = 4 * radius * radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch, 3);

        int currentIndex = 0;

        for (int j = 0; j < patch.cols(); ++j)
        {
            for (int i = 0; i < patch.rows(); ++i)
            {
                const float distanceToCenter = std::sqrt((i - radius) * (i - radius) + (j - radius) * (j - radius));
                if ((distanceToCenter < 0.95 * radius) && (patch(i, j) > 0.2) && (patch(i, j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = patch(i, j);

                    normalSphere(currentIndex, 0) = (float(j) - radius) / radius;
                    normalSphere(currentIndex, 1) = (float(i) - radius) / radius;
                    normalSphere(currentIndex, 2) = -sqrt(1 - normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                          normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1));

                    ++currentIndex;
                }
            }
        }

        Eigen::MatrixXf normalSphereMasked(currentIndex, 3);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex, 3);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);
        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);

        intensity = lightingDirection.norm();
        lightingDirection = lightingDirection / intensity;
    }

    // If method = SH :
    else if (!method.compare("SH"))
    {
        size_t lightSize = lightingDirection.size();

        // Evaluate light direction and intensity by pseudo-inverse
        int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
        int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

        float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        int nbPixelsPatch = 4 * radius * radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch, lightSize);

        int currentIndex = 0;

        for (size_t j = 0; j < patch.cols(); ++j)
        {
            for (size_t i = 0; i < patch.rows(); ++i)
            {
                float distanceToCenter = sqrt((i - radius) * (i - radius) + (j - radius) * (j - radius));
                if (distanceToCenter < 0.95 * radius && (patch(i, j) > 0.2) && (patch(i, j) < 0.8))
                {
                    imSphere(currentIndex) = patch(i, j);

                    normalSphere(currentIndex, 0) = (float(j) - radius) / radius;
                    normalSphere(currentIndex, 1) = (float(i) - radius) / radius;
                    normalSphere(currentIndex, 2) = -sqrt(1 - normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                          normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1));
                    normalSphere(currentIndex, 3) = 1;
                    if (lightSize > 4)
                    {
                        normalSphere(currentIndex, 4) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 1);
                        normalSphere(currentIndex, 5) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 2);
                        normalSphere(currentIndex, 6) = normalSphere(currentIndex, 1) * normalSphere(currentIndex, 2);
                        normalSphere(currentIndex, 7) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                        normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1);
                        normalSphere(currentIndex, 8) = 3 * normalSphere(currentIndex, 2) * normalSphere(currentIndex, 2) - 1;
                    }
                    ++currentIndex;
                }
            }
        }

        Eigen::MatrixXf normalSphereMasked(currentIndex, lightSize);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex, lightSize);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);

        // 1) Directionnal part estimation :
        Eigen::MatrixXf normalOrdre1(currentIndex, 3);
        normalOrdre1 = normalSphereMasked.leftCols(3);
        Eigen::Vector3f directionnalPart = normalOrdre1.colPivHouseholderQr().solve(imSphereMasked);
        intensity = directionnalPart.norm();
        directionnalPart = directionnalPart / intensity;

        // 2) Other order estimation :
        Eigen::VectorXf imSphereModif(currentIndex);
        imSphereModif = imSphereMasked;
        for (size_t i = 0; i < currentIndex; ++i)
        {
            for (size_t k = 0; k < 3; ++k)
            {
                imSphereModif(i) -= normalSphereMasked(i, k) * directionnalPart(k);
            }
        }
        Eigen::VectorXf secondOrder(6);
        secondOrder = normalSphereMasked.rightCols(6).colPivHouseholderQr().solve(imSphereModif);

        lightingDirection << directionnalPart, secondOrder;
    }
}

void calibrateLightFromRealSphere(const image::Image<float>& imageFloat,
                                  const cv::Mat& maskCV,
                                  const Eigen::Matrix3f& K,
                                  const float sphereRadius,
                                  const std::string& method,
                                  Eigen::VectorXf& lightingDirection,
                                  float& intensity)
{
    image::Image<image::RGBfColor> normals(imageFloat.width(), imageFloat.height());
    image::Image<float> newMask(imageFloat.width(), imageFloat.height());

    getRealNormalOnSphere(maskCV, K, sphereRadius, normals, newMask);

    image::Image<image::RGBColor> normalsPNG(maskCV.cols, maskCV.rows);
    aliceVision::photometricStereo::convertNormalMap2png(normals, normalsPNG);

    // If method = brightest point :
    if (!method.compare("brightestPoint"))
    {
        // Detect brightest point :
        Eigen::Vector2f brigthestPoint;
        detectBrightestPoint(newMask, imageFloat, brigthestPoint);

        Eigen::Vector2f brigthestPoint_xy;
        brigthestPoint_xy(0) = brigthestPoint(0) - imageFloat.cols() / 2;
        brigthestPoint_xy(1) = brigthestPoint(1) - imageFloat.rows() / 2;

        Eigen::Vector3f normalBrightestPoint;
        normalBrightestPoint = normals(round(brigthestPoint(1)), round(brigthestPoint(0))).cast<float>();

        // Observation direction :
        Eigen::Vector3f observationRayPersp;

        // orthographic approximation :
        observationRayPersp(0) = brigthestPoint_xy(0) / K(0, 0);
        observationRayPersp(1) = brigthestPoint_xy(1) / K(0, 0);
        observationRayPersp(2) = 1.0;
        observationRayPersp = -observationRayPersp / observationRayPersp.norm();

        // Evaluate lighting direction :
        lightingDirection = 2 * normalBrightestPoint.dot(observationRayPersp) * normalBrightestPoint - observationRayPersp;
        lightingDirection = lightingDirection / lightingDirection.norm();

        intensity = 1.0;
    }

    // If method = whiteSphere :
    else if (!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse

        std::vector<int> indices;
        aliceVision::photometricStereo::getIndMask(newMask, indices);

        Eigen::VectorXf imSphere(indices.size());
        Eigen::MatrixXf normalSphere(indices.size(), 3);

        int currentIndex = 0;

        for (int j = 0; j < newMask.cols(); ++j)
        {
            for (int i = 0; i < newMask.rows(); ++i)
            {
                if (newMask(i, j) > 0.1 && (imageFloat(i, j) > 0.2) && (imageFloat(i, j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = imageFloat(i, j);

                    normalSphere(currentIndex, 0) = normals(i, j)(0);
                    normalSphere(currentIndex, 1) = normals(i, j)(1);
                    normalSphere(currentIndex, 2) = normals(i, j)(2);

                    ++currentIndex;
                }
            }
        }

        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);

        intensity = lightingDirection.norm();
        lightingDirection = lightingDirection / intensity;
    }
}

void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, sphereParam, patch, patchOrigin);

    image::Image<float> convolutedPatch1;
    image::Image<float> convolutedPatch2;

    // Create Kernel
    size_t kernelSize = round(sphereParam[2] / 20);  // arbitrary
    Eigen::VectorXf kernel(2 * kernelSize + 1);
    createTriangleKernel(kernelSize, kernel);

    image::imageVerticalConvolution(patch, kernel, convolutedPatch1);
    image::imageHorizontalConvolution(convolutedPatch1, kernel, convolutedPatch2);

    Eigen::Index maxRow, maxCol;
    static_cast<void>(convolutedPatch2.maxCoeff(&maxRow, &maxCol));

    brigthestPoint(0) = maxCol + patchOrigin[0];
    brigthestPoint(1) = maxRow + patchOrigin[1];
}

void detectBrightestPoint(const image::Image<float> newMask, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, newMask, patch, patchOrigin);

    image::Image<float> convolutedPatch1;
    image::Image<float> convolutedPatch2;

    // Create Kernel
    size_t kernelSize = round(patch.rows() / 40);  // arbitrary
    Eigen::VectorXf kernel(2 * kernelSize + 1);
    createTriangleKernel(kernelSize, kernel);

    image::imageVerticalConvolution(patch, kernel, convolutedPatch1);
    image::imageHorizontalConvolution(convolutedPatch1, kernel, convolutedPatch2);

    Eigen::Index maxRow, maxCol;
    static_cast<void>(convolutedPatch2.maxCoeff(&maxRow, &maxCol));

    brigthestPoint(0) = maxCol + patchOrigin[0];
    brigthestPoint(1) = maxRow + patchOrigin[1];
}

void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel)
{
    for (int i = 0; i < 2 * kernelSize + 1; ++i)
    {
        if (i > kernelSize)
        {
            kernel(i) = (1.0 + kernelSize - (i - kernelSize)) / kernelSize;
        }
        else
        {
            kernel(i) = (1.0 + i) / kernelSize;
        }
    }
}

void getNormalOnSphere(const float xPicture, const float yPicture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal)
{
    currentNormal(0) = (xPicture - sphereParam[0]) / sphereParam[2];
    currentNormal(1) = (yPicture - sphereParam[1]) / sphereParam[2];
    currentNormal(2) = -sqrt(1 - currentNormal(0) * currentNormal(0) - currentNormal(1) * currentNormal(1));
}

void cutImage(const image::Image<float>& imageFloat,
              const std::array<float, 3>& sphereParam,
              image::Image<float>& patch,
              std::array<float, 2>& patchOrigin)
{
    // Absolute position of the patch's top left corner in image
    const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
    const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    const int radius = round(sphereParam[2]);

    patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

    for (int i = 0; i < patch.rows(); ++i)
    {
        for (int j = 0; j < patch.cols(); ++j)
        {
            const float distanceToCenter = (i - patch.rows() / 2) * (i - patch.rows() / 2) + (j - patch.cols() / 2) * (j - patch.cols() / 2);
            if (distanceToCenter > radius * radius + 2)
            {
                patch(i, j) = 0;
            }
        }
    }
}

void cutImage(const image::Image<float>& imageFloat,
              const image::Image<float>& newMask,
              image::Image<float>& patch,
              std::array<float, 2>& patchOrigin)
{
    int minISphere = newMask.rows();
    int minJSphere = newMask.cols();
    int maxISphere = 0;
    int maxJSphere = 0;

    for (int j = 0; j < newMask.cols(); ++j)
    {
        for (int i = 0; i < newMask.rows(); ++i)
        {
            if (newMask(i, j) == 1)
            {
                if (minISphere > i)
                    minISphere = i;

                if (minJSphere > j)
                    minJSphere = j;

                if (maxISphere < i)
                    maxISphere = i;

                if (maxJSphere < j)
                    maxJSphere = j;
            }
        }
    }

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    patch = imageFloat.block(minISphere, minJSphere, maxISphere - minISphere, maxJSphere - minJSphere);
}

void writeJSON(const std::string& fileName,
               const sfmData::SfMData& sfmData,
               const std::vector<std::string>& imageList,
               const Eigen::MatrixXf& lightMat,
               const std::vector<float>& intList,
               const bool saveAsModel,
               const std::string method)
{
    bpt::ptree lightsTree;
    bpt::ptree fileTree;

    int imgCpt = 0;
    std::map<std::string, sfmData::View> viewMap;
    for (auto& viewIt : sfmData.getViews())
    {
        std::map<std::string, std::string> currentMetadata = sfmData.getView(viewIt.first).getImage().getMetadata();

        if (currentMetadata.find("Exif:DateTimeDigitized") == currentMetadata.end())
        {
            ALICEVISION_LOG_INFO("No metadata case (Exif:DateTimeDigitized is missing)");
            viewMap[sfmData.getView(viewIt.first).getImage().getImagePath()] = sfmData.getView(viewIt.first);
        }
        else
        {
            viewMap[currentMetadata.at("Exif:DateTimeDigitized")] = sfmData.getView(viewIt.first);
        }
    }

    for (const auto& [currentId, viewId] : viewMap)
    {
        const fs::path imagePath = fs::path(viewId.getImage().getImagePath());

        // The file may be in the input SfMData but may not have been calibrated: in that case, it is not in imageList
        const bool calibratedFile = (std::find(imageList.begin(), imageList.end(), viewId.getImage().getImagePath()) != imageList.end());

        // Only write images that were actually used for the lighting calibration, instead of all the input images
        if (!boost::algorithm::icontains(imagePath.stem().string(), "ambient") && calibratedFile)
        {
            bpt::ptree lightTree;
            if (saveAsModel)
            {
                lightTree.put("lightId", imgCpt);
            }
            else
            {
                lightTree.put("viewId", viewId.getViewId());
            }
            if (!method.compare("SH"))
                lightTree.put("type", "SH");
            else
                lightTree.put("type", "directional");

            // Light direction
            bpt::ptree directionNode;
            int lightMatSize = lightMat.cols();
            for (int i = 0; i < lightMatSize; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(lightMat(imgCpt, i));
                directionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("direction", directionNode);

            // Light intensity
            bpt::ptree intensityNode;
            for (unsigned int i = 0; i < 3; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(intList.at(imgCpt));
                intensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", intensityNode);
            imgCpt++;

            lightsTree.push_back(std::make_pair("", lightTree));
        }
        else
        {
            ALICEVISION_LOG_INFO("'" << imagePath << "' is in the input SfMData but has not been used for the lighting "
                                     << "calibration or contains 'ambient' in its filename.");
        }
    }

    fileTree.add_child("lights", lightsTree);
    bpt::write_json(fileName, fileTree);
}

void sphereFromLighting(const Eigen::VectorXf& lightVector, const float intensity, const std::string outputFileName, const int outputSize)
{
    float radius = (outputSize * 0.9) / 2;
    image::Image<float> pixelsValues(outputSize, outputSize);

    for (size_t j = 0; j < outputSize; ++j)
    {
        for (size_t i = 0; i < outputSize; ++i)
        {
            float center_xy = outputSize / 2;
            Eigen::VectorXf normalSphere(lightVector.size());
            float distanceToCenter = sqrt((i - center_xy) * (i - center_xy) + (j - center_xy) * (j - center_xy));
            pixelsValues(i, j) = 0;

            if (distanceToCenter < radius)
            {
                normalSphere(0) = (float(j) - center_xy) / radius;
                normalSphere(1) = (float(i) - center_xy) / radius;
                normalSphere(2) = -sqrt(1 - normalSphere(0) * normalSphere(0) - normalSphere(1) * normalSphere(1));
                if (lightVector.size() > 3)
                {
                    normalSphere(3) = 1;
                }
                if (lightVector.size() > 4)
                {
                    normalSphere(4) = normalSphere(0) * normalSphere(1);
                    normalSphere(5) = normalSphere(0) * normalSphere(2);
                    normalSphere(6) = normalSphere(1) * normalSphere(2);
                    normalSphere(7) = normalSphere(0) * normalSphere(0) - normalSphere(1) * normalSphere(1);
                    normalSphere(8) = 3 * normalSphere(2) * normalSphere(2) - 1;
                }

                for (size_t k = 0; k < lightVector.size(); ++k)
                {
                    pixelsValues(i, j) += normalSphere(k) * lightVector(k);
                }
                pixelsValues(i, j) *= intensity;
            }
        }
    }

    image::writeImage(
      outputFileName,
      pixelsValues,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

}  // namespace lightingEstimation
}  // namespace aliceVision

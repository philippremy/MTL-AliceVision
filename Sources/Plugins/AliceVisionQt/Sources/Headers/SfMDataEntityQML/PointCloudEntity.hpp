#pragma once

#include <QEntity>
#include <AVSfMDataIO/sfmDataIO.hpp>

namespace sfmdataentity {

class PointCloudEntity : public Qt3DCore::QEntity
{
    Q_OBJECT

  public:
    explicit PointCloudEntity(Qt3DCore::QNode* = nullptr);
    ~PointCloudEntity() override = default;
    void setData(const aliceVision::sfmData::Landmarks& landmarks);
};

}  // namespace sfmdataentity

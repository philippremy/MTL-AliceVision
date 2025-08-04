#pragma once

#include <AV/types.hpp>
#include <AVFeature/feature.hpp>

#include <QObject>
#include <QRunnable>
#include <QVariantList>
#include <QVariantMap>

#include <map>
#include <vector>
#include <string>

namespace qtAliceVision {

using FeaturesPerViewPerDesc = std::map<std::string, std::map<aliceVision::IndexT, std::vector<aliceVision::feature::PointFeature>>>;

/**
 * @brief QObject wrapper around extracted features.
 *
 * Given a folder containing extracted features,
 * the role of an MFeatures instance is to load the features from disk.
 * Describer types and view IDs to load must also be specified.
 * This task is done asynchronously to avoid freezing the UI.
 *
 * MFeatures objects are accessible from QML
 * and can be manipulated through their properties.
 *
 * Note:
 * for a given describer type and view ID,
 * features are stored in an array-like structure
 * and the ID a a feature corresponds to its index in this array.
 */
class MFeatures : public QObject
{
    Q_OBJECT

    /// Data properties

    // Path to folder containing the features
    Q_PROPERTY(QVariantList featureFolders MEMBER _featureFolders NOTIFY featureFoldersChanged)
    // View IDs to load
    Q_PROPERTY(QVariantList viewIds MEMBER _viewIds NOTIFY viewIdsChanged)
    // Describer types to load
    Q_PROPERTY(QVariantList describerTypes MEMBER _describerTypes NOTIFY describerTypesChanged)

    /// Status

    Q_PROPERTY(Status status READ status NOTIFY statusChanged)

  public:
    /// Status Enum

    enum Status
    {
        None = 0,
        Loading,
        Ready,
        Error
    };
    Q_ENUM(Status)

    /// Slots

    Q_SLOT void load();
    Q_SLOT void onFeaturesReady(FeaturesPerViewPerDesc* featuresPerViewPerDesc);

    /// Signals

    Q_SIGNAL void featureFoldersChanged();
    Q_SIGNAL void describerTypesChanged();
    Q_SIGNAL void viewIdsChanged();
    Q_SIGNAL void featuresChanged();
    Q_SIGNAL void statusChanged(Status status);

    /// Invokables

    Q_INVOKABLE int nbFeatures(QString describerType, int viewId) const;

    /// Public methods

    MFeatures();
    MFeatures(const MFeatures& other) = delete;
    ~MFeatures() override;

    FeaturesPerViewPerDesc& rawData() { return *_featuresPerViewPerDesc; }
    const FeaturesPerViewPerDesc& rawData() const { return *_featuresPerViewPerDesc; }
    const FeaturesPerViewPerDesc* rawDataPtr() const { return _featuresPerViewPerDesc; }

    Status status() const { return _status; }
    void setStatus(Status status);

  private:
    /// Private members

    QVariantList _featureFolders;
    QVariantList _viewIds;
    QVariantList _describerTypes;

    FeaturesPerViewPerDesc* _featuresPerViewPerDesc = nullptr;

    bool _needReload = false;
    Status _status = MFeatures::None;
};

/**
 * @brief QRunnable object dedicated to loading features using AliceVision.
 */
class FeaturesIORunnable : public QObject, public QRunnable
{
    Q_OBJECT

  public:
    FeaturesIORunnable(const std::vector<std::string>& folders,
                       const std::vector<aliceVision::IndexT>& viewIds,
                       const std::vector<std::string>& describerTypes)
      : _folders(folders),
        _viewIds(viewIds),
        _describerTypes(describerTypes)
    {}

    /// Load features based on input parameters
    Q_SLOT void run() override;

    /**
     * @brief  Emitted when features have been loaded and Features objects created.
     * @warning Features objects are not parented - their deletion must be handled manually.
     *
     * @param features the loaded Features list
     */
    Q_SIGNAL void resultReady(FeaturesPerViewPerDesc* featuresPerViewPerDesc);

  private:
    std::vector<std::string> _folders;
    std::vector<aliceVision::IndexT> _viewIds;
    std::vector<std::string> _describerTypes;
};

}  // namespace qtAliceVision

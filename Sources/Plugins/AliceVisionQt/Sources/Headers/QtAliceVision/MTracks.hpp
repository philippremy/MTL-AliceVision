#pragma once

#include <AVTrack/Track.hpp>

#include <QQuickItem>
#include <QRunnable>
#include <QList>
#include <QVariant>

#include <string>
#include <vector>

namespace qtAliceVision {

/**
 * @brief QObject wrapper around Tracks.
 *
 * Given a folder containing feature matches,
 * the role of an MTracks instance is to load the matches from disk
 * and build the corresponding tracks.
 * These tasks are done asynchronously to avoid freezing the UI.
 *
 * MTracks objects are accessible from QML
 * and can be manipulated through their properties.
 */
class MTracks : public QObject
{
    Q_OBJECT

    /// Data properties

    // Path to folder containing the matches
    Q_PROPERTY(QVariantList matchingFolders MEMBER _matchingFolders NOTIFY matchingFoldersChanged)

    /// Path to
    Q_PROPERTY(QUrl tracksFile MEMBER _tracksFile NOTIFY tracksFileChanged)

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

    MTracks();
    MTracks& operator=(const MTracks& other) = delete;
    ~MTracks() override;

  private:
    MTracks(const MTracks& other);

  public:
    /// Slots

    Q_SLOT void load();
    Q_SLOT void loadDirect();
    Q_SLOT void onReady(aliceVision::track::TracksMap* tracks, aliceVision::track::TracksPerView* tracksPerView);

    /// Signals

    Q_SIGNAL void matchingFoldersChanged();
    Q_SIGNAL void tracksFileChanged();
    Q_SIGNAL void tracksChanged();
    Q_SIGNAL void statusChanged(Status status);

    /// Invokables

    Q_INVOKABLE int nbMatches(QString describerType, int viewId) const;

  public:
    const aliceVision::track::TracksMap* tracksPtr() const { return _tracks; }
    const aliceVision::track::TracksMap& tracks() const { return *_tracks; }
    const aliceVision::track::TracksPerView& tracksPerView() const { return *_tracksPerView; }

    Status status() const { return _status; }
    void setStatus(Status status);

  private:
    /// Private members

    QVariantList _matchingFolders;
    QUrl _tracksFile;

    aliceVision::track::TracksMap* _tracks = nullptr;
    aliceVision::track::TracksPerView* _tracksPerView = nullptr;

    bool _needReload = false;
    Status _status = MTracks::None;
};

/**
 * @brief QRunnable object dedicated to loading matches and building tracks using AliceVision.
 */
class TracksIORunnable : public QObject, public QRunnable
{
    Q_OBJECT

  public:
    explicit TracksIORunnable(const std::vector<std::string>& folders)
      : _folders(folders)
    {}

    Q_SLOT void run() override;

    Q_SIGNAL void resultReady(aliceVision::track::TracksMap* tracks, aliceVision::track::TracksPerView* tracksPerView);

  private:
    std::vector<std::string> _folders;
};

/**
 * @brief QRunnable object dedicated to loading tracks using AliceVision.
 */
class TracksDirectIORunnable : public QObject, public QRunnable
{
    Q_OBJECT

  public:
    explicit TracksDirectIORunnable(const std::string& filename)
      : _filename(filename)
    {}

    Q_SLOT void run() override;

    Q_SIGNAL void resultReady(aliceVision::track::TracksMap* tracks, aliceVision::track::TracksPerView* tracksPerView);

  private:
    std::string _filename;
};

}  // namespace qtAliceVision

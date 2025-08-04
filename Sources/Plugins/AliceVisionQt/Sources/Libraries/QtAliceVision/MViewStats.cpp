#include <QtAliceVision/MViewStats.hpp>

#include <QSGGeometryNode>

#include <QPen>
#include <QThreadPool>
#include <algorithm>
#include <cmath>

namespace qtAliceVision {

void MViewStats::fillResidualFullSerie(QXYSeries* residuals)
{
    if (residuals == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualFullSerie: no residuals";
        return;
    }
    residuals->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualFullSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualFullSerie: no valid view: ";
        return;
    }

    std::vector<double> residualHistX = _residualHistogramFull.GetXbinsValue();
    std::vector<size_t> residualHistY = _residualHistogramFull.GetHist();

    if (residualHistX.size() != residualHistY.size())
    {
        throw std::runtime_error("MViewStats::fillResidualFullSerie: residualHistX & residualHistY size mismatch.");
    }
    QPen pen(Qt::red, 1, Qt::DashLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < residualHistX.size(); ++i)
    {
        residuals->append(double(residualHistX[i]), double(residualHistY[i]));
        residuals->setPen(pen);
    }
}

void MViewStats::fillResidualViewSerie(QXYSeries* residuals)
{
    if (residuals == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualViewSerie: no residuals";
        return;
    }
    residuals->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualViewSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillResidualViewSerie: no valid view: ";
        return;
    }

    std::vector<double> residualHistX = _residualHistogramView.GetXbinsValue();
    std::vector<size_t> residualHistY = _residualHistogramView.GetHist();
    if (residualHistX.size() != residualHistY.size())
    {
        throw std::runtime_error("MViewStats::fillResidualViewSerie: residualHistX & residualHistY size mismatch.");
    }
    QPen pen(Qt::darkBlue, 3, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < residualHistX.size(); ++i)
    {
        residuals->append(double(residualHistX[i]), double(residualHistY[i]));
        residuals->setPen(pen);
    }
}

void MViewStats::fillObservationsLengthsFullSerie(QXYSeries* observationsLengths)
{
    if (observationsLengths == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsFullSerie: no observationsLengths";
        return;
    }
    observationsLengths->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsFullSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsFullSerie: no valid view: ";
        return;
    }

    std::vector<double> observationsLengthsHistX = _observationsLengthsHistogramFull.GetXbinsValue();
    std::vector<size_t> observationsLengthsHistY = _observationsLengthsHistogramFull.GetHist();
    if (observationsLengthsHistX.size() != observationsLengthsHistY.size())
    {
        throw std::runtime_error("MViewStats::fillObservationsLengthsFullSerie: observationsLengthsHistX & observationsLengthsHistY size mismatch.");
    }
    QPen pen(Qt::red, 1, Qt::DashLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < observationsLengthsHistX.size(); ++i)
    {
        observationsLengths->append(double(observationsLengthsHistX[i]), double(observationsLengthsHistY[i]));
        observationsLengths->setPen(pen);
    }
}

void MViewStats::fillObservationsLengthsViewSerie(QXYSeries* observationsLengths)
{
    if (observationsLengths == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsViewSerie: no observationsLengths";
        return;
    }
    observationsLengths->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsViewSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsLengthsViewSerie: no valid view: ";
        return;
    }

    std::vector<double> observationsLengthsHistX = _observationsLengthsHistogramView.GetXbinsValue();
    std::vector<size_t> observationsLengthsHistY = _observationsLengthsHistogramView.GetHist();
    if (observationsLengthsHistX.size() != observationsLengthsHistY.size())
    {
        throw std::runtime_error("MViewStats::fillObservationsLengthsViewSerie: observationsLengthsHistX & observationsLengthsHistY size mismatch.");
    }
    QPen pen(Qt::darkBlue, 3, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < observationsLengthsHistX.size(); ++i)
    {
        observationsLengths->append(double(observationsLengthsHistX[i]), double(observationsLengthsHistY[i]));
        observationsLengths->setPen(pen);
    }
}

void MViewStats::fillObservationsScaleFullSerie(QXYSeries* observationsScale)
{
    if (observationsScale == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleFullSerie: no observationsScale";
        return;
    }
    observationsScale->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleFullSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleFullSerie: no valid view: ";
        return;
    }

    std::vector<double> observationsScaleHistX = _observationsScaleHistogramFull.GetXbinsValue();
    std::vector<size_t> observationsScaleHistY = _observationsScaleHistogramFull.GetHist();
    if (observationsScaleHistX.size() != observationsScaleHistY.size())
    {
        throw std::runtime_error("MViewStats::fillObservationsScaleFullSerie: observationsScaleHistX & observationsScaleHistY size mismatch.");
    }
    QPen pen(Qt::red, 1, Qt::DashLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < observationsScaleHistX.size(); ++i)
    {
        observationsScale->append(double(observationsScaleHistX[i]), double(observationsScaleHistY[i]));
        observationsScale->setPen(pen);
    }
}

void MViewStats::fillObservationsScaleViewSerie(QXYSeries* observationsScale)
{
    if (observationsScale == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleViewSerie: no observationsScale";
        return;
    }
    observationsScale->clear();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleViewSerie: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision] MViewStats::fillObservationsScaleViewSerie: no valid view: ";
        return;
    }

    std::vector<double> observationsScaleHistX = _observationsScaleHistogramView.GetXbinsValue();
    std::vector<size_t> observationsScaleHistY = _observationsScaleHistogramView.GetHist();
    if (observationsScaleHistX.size() != observationsScaleHistY.size())
    {
        throw std::runtime_error("MViewStats::fillObservationsScaleViewSerie: observationsScaleHistX & observationsScaleHistY size mismatch.");
    }
    QPen pen(Qt::darkBlue, 3, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);

    for (std::size_t i = 0; i < observationsScaleHistX.size(); ++i)
    {
        observationsScale->append(double(observationsScaleHistX[i]), double(observationsScaleHistY[i]));
        observationsScale->setPen(pen);
    }
}

void MViewStats::computeViewStats()
{
    _residualHistogramFull = aliceVision::utils::Histogram<double>();
    _residualHistogramView = aliceVision::utils::Histogram<double>();

    _observationsLengthsHistogramFull = aliceVision::utils::Histogram<double>();
    _observationsLengthsHistogramView = aliceVision::utils::Histogram<double>();

    _observationsScaleHistogramFull = aliceVision::utils::Histogram<double>();
    _observationsScaleHistogramView = aliceVision::utils::Histogram<double>();

    if (_msfmData == nullptr)
    {
        qInfo() << "[QtAliceVision]  MViewStats::computeViewStats: no SfMData loaded";
        return;
    }
    if (_viewId == aliceVision::UndefinedIndexT)
    {
        qInfo() << "[QtAliceVision]  MViewStats::computeViewStats: no valid view: " << _viewId;
        return;
    }
    using namespace aliceVision;
    // residual histogram
    {
        // Init max values per axis
        _residualMaxAxisX = 0.0;
        _residualMaxAxisY = 0.0;

        {
            // residual histogram of all views
            BoxStats<double> residualFullStats;
            sfm::computeResidualsHistogram(_msfmData->rawData(), residualFullStats, &_residualHistogramFull);

            double nbCameras = double(_msfmData->nbCameras());

            // normalize the histogram to get the average number of observations
            std::vector<size_t>& residualFullHistY = _residualHistogramFull.GetHist();
            for (std::size_t i = 0; i < residualFullHistY.size(); ++i)
            {
                residualFullHistY[i] = static_cast<size_t>(round(static_cast<double>(residualFullHistY[i]) / nbCameras));
            }
            std::vector<double> residualHistX = _residualHistogramFull.GetXbinsValue();

            for (std::size_t i = 0; i < residualHistX.size(); ++i)
            {
                _residualMaxAxisX = round(std::max(_residualMaxAxisX, residualHistX[i]));
            }
            for (std::size_t i = 0; i < residualFullHistY.size(); ++i)
            {
                _residualMaxAxisY = round(std::max(_residualMaxAxisY, double(residualFullHistY[i])));
            }
        }
        {
            // residual histogram of current view
            BoxStats<double> residualViewStats;
            sfm::computeResidualsHistogram(_msfmData->rawData(), residualViewStats, &_residualHistogramView, {_viewId});
            std::vector<size_t>& residualViewHistY = _residualHistogramView.GetHist();
            std::vector<double> residualHistX = _residualHistogramView.GetXbinsValue();

            for (std::size_t i = 0; i < residualHistX.size(); ++i)
            {
                _residualMaxAxisX = round(std::max(_residualMaxAxisX, residualHistX[i]));
            }
            for (std::size_t i = 0; i < residualViewHistY.size(); ++i)
            {
                _residualMaxAxisY = round(std::max(_residualMaxAxisY, double(residualViewHistY[i])));
            }
        }
    }

    _nbObservations = 0;
    {
        _observationsLengthsMaxAxisX = 0.0;
        _observationsLengthsMaxAxisY = 0.0;
        // observationsLengths histogram
        {
            // observationsLengths histogram of all views
            BoxStats<double> observationsLengthsFullStats;
            sfm::computeObservationsLengthsHistogram(
              _msfmData->rawData(), observationsLengthsFullStats, _nbObservations, &_observationsLengthsHistogramFull);

            const double nbCameras = double(_msfmData->nbCameras());
            std::vector<size_t>& observationsLengthsFullHistY = _observationsLengthsHistogramFull.GetHist();

            // normalize the histogram to get the average number of observations
            for (std::size_t i = 0; i < observationsLengthsFullHistY.size(); ++i)
            {
                observationsLengthsFullHistY[i] = static_cast<size_t>(round(static_cast<double>(observationsLengthsFullHistY[i]) / nbCameras));
            }
            std::vector<double> observationsLengthsHistX = _observationsLengthsHistogramFull.GetXbinsValue();

            for (std::size_t i = 0; i < observationsLengthsHistX.size(); ++i)
            {
                _observationsLengthsMaxAxisX = round(std::max(_observationsLengthsMaxAxisX, observationsLengthsHistX[i]));
            }
            for (std::size_t i = 0; i < observationsLengthsFullHistY.size(); ++i)
            {
                _observationsLengthsMaxAxisY = round(std::max(_observationsLengthsMaxAxisY, double(observationsLengthsFullHistY[i])));
            }
        }
        {
            // observationsLengths histogram of current view
            BoxStats<double> observationsLengthsViewStats;
            sfm::computeObservationsLengthsHistogram(
              _msfmData->rawData(), observationsLengthsViewStats, _nbObservations, &_observationsLengthsHistogramView, {_viewId});
            const std::vector<size_t> observationsLengthsViewHistY = _observationsLengthsHistogramView.GetHist();
            const std::vector<double> observationsLengthsHistX = _observationsLengthsHistogramView.GetXbinsValue();

            for (std::size_t i = 0; i < observationsLengthsHistX.size(); ++i)
            {
                _observationsLengthsMaxAxisX = round(std::max(_observationsLengthsMaxAxisX, observationsLengthsHistX[i]));
            }
            for (std::size_t i = 0; i < observationsLengthsViewHistY.size(); ++i)
            {
                _observationsLengthsMaxAxisY = round(std::max(_observationsLengthsMaxAxisY, double(observationsLengthsViewHistY[i])));
            }
        }
    }

    // scale histogram
    {
        // histogram of observations Scale of all views
        BoxStats<double> observationsScaleFullStats;
        sfm::computeScaleHistogram(_msfmData->rawData(), observationsScaleFullStats, &_observationsScaleHistogramFull);

        const double nbCameras = double(_msfmData->nbCameras());

        // normalize the histogram to get the average number of observations
        std::vector<size_t>& observationsScaleFullHistY = _observationsScaleHistogramFull.GetHist();
        for (std::size_t i = 0; i < observationsScaleFullHistY.size(); ++i)
        {
            observationsScaleFullHistY[i] = static_cast<std::size_t>(std::round(static_cast<double>(observationsScaleFullHistY[i]) / nbCameras));
        }
        std::vector<double> observationsScaleHistX = _observationsScaleHistogramFull.GetXbinsValue();

        // histogram of observations Scale of current view
        BoxStats<double> observationsScaleViewStats;
        sfm::computeScaleHistogram(_msfmData->rawData(), observationsScaleViewStats, &_observationsScaleHistogramView, {_viewId});
        const std::vector<size_t> observationsScaleViewHistY = _observationsScaleHistogramView.GetHist();

        _observationsScaleMaxAxisX = 0.0;
        _observationsScaleMaxAxisY = 0.0;

        for (std::size_t i = 0; i < observationsScaleHistX.size(); ++i)
        {
            _observationsScaleMaxAxisX = round(std::max(_observationsScaleMaxAxisX, observationsScaleHistX[i]));
        }
        for (std::size_t i = 0; i < observationsScaleFullHistY.size(); ++i)
        {
            _observationsScaleMaxAxisY = round(std::max(_observationsScaleMaxAxisY, double(observationsScaleFullHistY[i])));
        }
        for (std::size_t i = 0; i < observationsScaleViewHistY.size(); ++i)
        {
            _observationsScaleMaxAxisY = round(std::max(_observationsScaleMaxAxisY, double(observationsScaleViewHistY[i])));
        }
    }

    Q_EMIT viewStatsChanged();
}

void MViewStats::setMSfmData(qtAliceVision::MSfMData* sfmData)
{
    if (_msfmData != nullptr)
    {
        disconnect(_msfmData, SIGNAL(sfmDataChanged()), this, SIGNAL(sfmDataChanged()));
    }
    _msfmData = sfmData;
    if (_msfmData != nullptr)
    {
        connect(_msfmData, SIGNAL(sfmDataChanged()), this, SIGNAL(sfmDataChanged()));
    }
    Q_EMIT sfmDataChanged();
}

}  // namespace qtAliceVision

// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)

#include <AVSfMDataIO/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
namespace sfmDataIO {

class AlembicImporter
{
  public:
    explicit AlembicImporter(const std::string& filename);

    ~AlembicImporter();

    /**
     * @brief populate a SfMData from the alembic file
     * @param[out] sfmData The output SfMData
     * @param[in] flagsPart
     */
    void populateSfM(sfmData::SfMData& sfmdata, ESfMData flagsPart = ESfMData::ALL);

  private:
    struct DataImpl;
    std::unique_ptr<DataImpl> _dataImpl;
};

}  // namespace sfmDataIO
}  // namespace aliceVision

#endif // ALICEVISION_HAVE_ALEMBIC

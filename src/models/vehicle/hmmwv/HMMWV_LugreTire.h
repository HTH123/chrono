// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// HMMWV LuGre tire subsystem
//
// =============================================================================

#ifndef HMMWV_LUGRE_TIRE_H
#define HMMWV_LUGRE_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"

#include "models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_LugreTire : public ChLugreTire {
  public:
    HMMWV_LugreTire(const std::string& name);
    ~HMMWV_LugreTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual int GetNumDiscs() const override { return m_numDiscs; }
    virtual const double* GetDiscLocations() const override { return m_discLocs; }

    virtual double GetNormalStiffness() const override { return m_normalStiffness; }
    virtual double GetNormalDamping() const override { return m_normalDamping; }

    virtual void SetLugreParams() override;

  private:
    static const double m_radius;
    static const int m_numDiscs = 3;
    static const double m_discLocs[m_numDiscs];

    static const double m_normalStiffness;
    static const double m_normalDamping;
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif

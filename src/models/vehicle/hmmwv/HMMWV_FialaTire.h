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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// HMMWV Fiala tire subsystem
//
// =============================================================================

#ifndef HMMWV_FIALA_TIRE_H
#define HMMWV_FIALA_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_FialaTire : public ChFialaTire {
  public:
    HMMWV_FialaTire(const std::string& name);
    ~HMMWV_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual void SetFialaParams();

  private:
    static const double m_normalDamping;
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif

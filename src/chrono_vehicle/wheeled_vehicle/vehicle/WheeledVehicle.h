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
// Wheeled vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef WHEELED_VEHICLE_H
#define WHEELED_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Wheeled vehicle model constructed from a JSON specification file
class CH_VEHICLE_API WheeledVehicle : public ChWheeledVehicle {
  public:
    WheeledVehicle(const std::string& filename,
                   ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI);

    WheeledVehicle(ChSystem* system, const std::string& filename);

    ~WheeledVehicle() {}

    virtual int GetNumberAxles() const override { return m_num_axles; }

    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    virtual void Initialize(const ChCoordsys<>& chassisPos) override;

    bool UseVisualizationMesh() const { return m_chassisUseMesh; }
    const std::string& GetMeshFilename() const { return m_chassisMeshFile; }
    const std::string& GetMeshName() const { return m_chassisMeshName; }

  private:
    void Create(const std::string& filename);

    void LoadSteering(const std::string& filename, int which);
    void LoadDriveline(const std::string& filename);
    void LoadSuspension(const std::string& filename, int axle);
    void LoadAntirollbar(const std::string& filename);
    void LoadWheel(const std::string& filename, int axle, int side);
    void LoadBrake(const std::string& filename, int axle, int side);

  private:
    int m_num_axles;                           // number of axles for this vehicle
    std::vector<ChVector<> > m_suspLocations;  // locations of the suspensions relative to chassis
    std::vector<int> m_suspSteering;           // indexes of steering subsystems (-1 indicates a non-steered suspension)

    std::vector<ChVector<> > m_arbLocations;  // locations of the antirollbar subsystems relative to chassis
    std::vector<int> m_arbSuspension;         // indexes of steering subsystems

    int m_num_strs;                               // number of steering subsystems
    std::vector<ChVector<> > m_strLocations;      // locations of the steering subsystems relative to chassis
    std::vector<ChQuaternion<> > m_strRotations;  // orientations of the steering subsystems relative to chassis

    std::vector<int> m_driven_susp;  // indexes of the driven suspensions

    bool m_chassisUseMesh;          // true if using a mesh for chassis visualization
    std::string m_chassisMeshName;  // name of the chassis visualization mesh
    std::string m_chassisMeshFile;  // name of the Waveform file with the chassis mesh

    double m_chassisMass;         // chassis mass
    ChVector<> m_chassisCOM;      // location of the chassis COM in the chassis reference frame
    ChVector<> m_chassisInertia;  // moments of inertia of the chassis

    ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif

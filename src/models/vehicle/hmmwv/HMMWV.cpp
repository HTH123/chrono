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
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "models/vehicle/hmmwv/HMMWV.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
HMMWV::HMMWV()
    : m_system(NULL),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({NULL, NULL, NULL, NULL}),
      m_contactMethod(ChMaterialSurfaceBase::DVI),
      m_fixed(false),
      m_driveType(AWD),
      m_powertrainType(SHAFTS),
      m_tireType(RIGID),
      m_tire_step_size(-1),
      m_pacejkaParamFile(""),
      m_chassisVis(PRIMITIVES),
      m_wheelVis(PRIMITIVES),
      m_tireVis(false),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)) {}

HMMWV::HMMWV(ChSystem* system)
    : m_system(system),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({NULL, NULL, NULL, NULL}),
      m_contactMethod(ChMaterialSurfaceBase::DVI),
      m_fixed(false),
      m_driveType(AWD),
      m_powertrainType(SHAFTS),
      m_tireType(RIGID),
      m_tire_step_size(-1),
      m_pacejkaParamFile(""),
      m_chassisVis(PRIMITIVES),
      m_wheelVis(PRIMITIVES),
      m_tireVis(false),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)) {}

HMMWV::~HMMWV() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_tires[0];
    delete m_tires[1];
    delete m_tires[2];
    delete m_tires[3];
}

// -----------------------------------------------------------------------------
void HMMWV::Initialize() {
    // Create and initialize the HMMWV vehicle
    m_vehicle = CreateVehicle();
    m_vehicle->Initialize(m_initPos);

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case SHAFTS: {
            HMMWV_Powertrain* ptrain = new HMMWV_Powertrain;
            m_powertrain = ptrain;
            break;
        }
        case SIMPLE: {
            HMMWV_SimplePowertrain* ptrain = new HMMWV_SimplePowertrain;
            m_powertrain = ptrain;
            break;
        }
    }

    m_powertrain->Initialize(m_vehicle->GetChassis(), m_vehicle->GetDriveshaft());

#ifndef CHRONO_FEA
    // If ANCF tire selected but not available, fall back on rigid tire.
    if (m_tireType == ANCF)
        m_tireType = RIGID;
#endif

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case RIGID: {
            HMMWV_RigidTire* tire_FL = new HMMWV_RigidTire("FL");
            HMMWV_RigidTire* tire_FR = new HMMWV_RigidTire("FR");
            HMMWV_RigidTire* tire_RL = new HMMWV_RigidTire("RL");
            HMMWV_RigidTire* tire_RR = new HMMWV_RigidTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case LUGRE: {
            HMMWV_LugreTire* tire_FL = new HMMWV_LugreTire("FL");
            HMMWV_LugreTire* tire_FR = new HMMWV_LugreTire("FR");
            HMMWV_LugreTire* tire_RL = new HMMWV_LugreTire("RL");
            HMMWV_LugreTire* tire_RR = new HMMWV_LugreTire("RR");

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case FIALA: {
            HMMWV_FialaTire* tire_FL = new HMMWV_FialaTire("FL");
            HMMWV_FialaTire* tire_FR = new HMMWV_FialaTire("FR");
            HMMWV_FialaTire* tire_RL = new HMMWV_FialaTire("RL");
            HMMWV_FialaTire* tire_RR = new HMMWV_FialaTire("RR");

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case PACEJKA: {
            if (m_pacejkaParamFile.empty())
                throw ChException("Pacejka parameter file not specified.");

            std::string param_file = vehicle::GetDataFile(m_pacejkaParamFile);

            ChPacejkaTire* tire_FL = new ChPacejkaTire("FL", param_file);
            ChPacejkaTire* tire_FR = new ChPacejkaTire("FR", param_file);
            ChPacejkaTire* tire_RL = new ChPacejkaTire("RL", param_file);
            ChPacejkaTire* tire_RR = new ChPacejkaTire("RR", param_file);

            tire_FL->SetDrivenWheel(false);
            tire_FR->SetDrivenWheel(false);
            tire_RL->SetDrivenWheel(true);
            tire_RR->SetDrivenWheel(true);

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case ANCF: {
#ifdef CHRONO_FEA
            HMMWV_ANCFTire* tire_FL = new HMMWV_ANCFTire("FL");
            HMMWV_ANCFTire* tire_FR = new HMMWV_ANCFTire("FR");
            HMMWV_ANCFTire* tire_RL = new HMMWV_ANCFTire("RL");
            HMMWV_ANCFTire* tire_RR = new HMMWV_ANCFTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
#endif
            break;
        }
        case REISSNER: {
#ifdef CHRONO_FEA
            HMMWV_ReissnerTire* tire_FL = new HMMWV_ReissnerTire("FL");
            HMMWV_ReissnerTire* tire_FR = new HMMWV_ReissnerTire("FR");
            HMMWV_ReissnerTire* tire_RL = new HMMWV_ReissnerTire("RL");
            HMMWV_ReissnerTire* tire_RR = new HMMWV_ReissnerTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
#endif
            break;
        }
    }

    // Enable/disable tire visualization
    m_tires[0]->EnableVisualization(m_tireVis);
    m_tires[1]->EnableVisualization(m_tireVis);
    m_tires[2]->EnableVisualization(m_tireVis);
    m_tires[3]->EnableVisualization(m_tireVis);

    // Initialize the tires.
    m_tires[0]->Initialize(m_vehicle->GetWheelBody(FRONT_LEFT), LEFT);
    m_tires[1]->Initialize(m_vehicle->GetWheelBody(FRONT_RIGHT), RIGHT);
    m_tires[2]->Initialize(m_vehicle->GetWheelBody(REAR_LEFT), LEFT);
    m_tires[3]->Initialize(m_vehicle->GetWheelBody(REAR_RIGHT), RIGHT);
}

// -----------------------------------------------------------------------------
void HMMWV::Synchronize(double time,
                        double steering_input,
                        double braking_input,
                        double throttle_input,
                        const ChTerrain& terrain) {
    TireForces tire_forces(4);
    WheelState wheel_states[4];

    tire_forces[0] = m_tires[0]->GetTireForce();
    tire_forces[1] = m_tires[1]->GetTireForce();
    tire_forces[2] = m_tires[2]->GetTireForce();
    tire_forces[3] = m_tires[3]->GetTireForce();

    wheel_states[0] = m_vehicle->GetWheelState(FRONT_LEFT);
    wheel_states[1] = m_vehicle->GetWheelState(FRONT_RIGHT);
    wheel_states[2] = m_vehicle->GetWheelState(REAR_LEFT);
    wheel_states[3] = m_vehicle->GetWheelState(REAR_RIGHT);

    double powertrain_torque = m_powertrain->GetOutputTorque();

    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();

    m_tires[0]->Synchronize(time, wheel_states[0], terrain);
    m_tires[1]->Synchronize(time, wheel_states[1], terrain);
    m_tires[2]->Synchronize(time, wheel_states[2], terrain);
    m_tires[3]->Synchronize(time, wheel_states[3], terrain);

    m_powertrain->Synchronize(time, throttle_input, driveshaft_speed);

    m_vehicle->Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
}

// -----------------------------------------------------------------------------
void HMMWV::Advance(double step) {
    m_tires[0]->Advance(step);
    m_tires[1]->Advance(step);
    m_tires[2]->Advance(step);
    m_tires[3]->Advance(step);

    m_powertrain->Advance(step);

    m_vehicle->Advance(step);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

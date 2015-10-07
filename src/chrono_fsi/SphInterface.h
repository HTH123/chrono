/*
 * SphInterface.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#ifndef SPHINTERFACE_H_
#define SPHINTERFACE_H_

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "MyStructs.cuh"  //just for SimParams

#include "chrono_parallel/physics/ChSystemParallel.h"
//#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
//#include "chrono_utils/ChUtilsCreators.h"  //Arman: why is this
//#include "chrono_utils/ChUtilsInputOutput.h" //Arman: Why is this
#include "utils/ChUtilsGenerators.h"

// using namespace chrono;
// using namespace chrono::collision;

chrono::ChVector<> ConvertRealToChVector(Real3 p3);
chrono::ChVector<> ConvertRealToChVector(Real4 p4);
chrono::ChQuaternion<> ConvertToChQuaternion(Real4 q4);
Real3 ConvertChVectorToR3(chrono::ChVector<> v3);
Real4 ConvertChVectorToR4(chrono::ChVector<> v3, Real m);
Real4 ConvertChQuaternionToR4(chrono::ChQuaternion<> q4);
Real3 Rotate_By_Quaternion(Real4 q4, Real3 BCE_Pos_local);

void AddSphDataToChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                          int& startIndexSph,
                          const thrust::host_vector<Real3>& posRadH,
                          const thrust::host_vector<Real4>& velMasH,
                          const SimParams& paramsH,
                          const NumberOfObjects& numObjects,
                          int collisionFamilly,
                          Real sphMarkerMass);

void AddHydroForce(chrono::ChSystemParallelDVI& mphysicalSystem, int& startIndexSph, const NumberOfObjects& numObjects);

void UpdateSphDataInChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                             const thrust::host_vector<Real3>& posRadH,
                             const thrust::host_vector<Real4>& velMasH,
                             const NumberOfObjects& numObjects,
                             int startIndexSph);

void AddChSystemForcesToSphForces(thrust::host_vector<Real4>& derivVelRhoChronoH,
                                  const thrust::host_vector<Real4>& velMasH2,
                                  chrono::ChSystemParallelDVI& mphysicalSystem,
                                  const NumberOfObjects& numObjects,
                                  int startIndexSph,
                                  Real dT);

void CountNumContactsPerSph(thrust::host_vector<short int>& numContactsOnAllSph,
                            const chrono::ChSystemParallelDVI& mphysicalSystem,
                            const NumberOfObjects& numObjects,
                            int startIndexSph);

void ClearArraysH(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
                  thrust::host_vector<Real4>& velMasH,
                  thrust::host_vector<Real4>& rhoPresMuH);

void ClearArraysH(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
                  thrust::host_vector<Real4>& velMasH,
                  thrust::host_vector<Real4>& rhoPresMuH,
                  thrust::host_vector<uint>& bodyIndex,
                  thrust::host_vector<int3>& referenceArray);

void CopyD2H(thrust::host_vector<Real4>& derivVelRhoChronoH, const thrust::device_vector<Real4>& derivVelRhoD);

void CopyForceSphToChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                            const NumberOfObjects& numObjects,
                            int startIndexSph,
                            const thrust::device_vector<Real4>& derivVelRhoD,
                            const thrust::host_vector<short int>& numContactsOnAllSph,
                            Real sphMass);

void CopyCustomChSystemPosVel2HostThrust(thrust::host_vector<Real3>& posRadH,
                                         thrust::host_vector<Real4>& velMasH,
                                         chrono::ChSystemParallelDVI& mphysicalSystem,
                                         const NumberOfObjects& numObjects,
                                         int startIndexSph,
                                         const thrust::host_vector<short int>& numContactsOnAllSph);

void CopyH2DPosVel(thrust::device_vector<Real3>& posRadD,
                   thrust::device_vector<Real4>& velMasD,
                   const thrust::host_vector<Real3>& posRadH,
                   const thrust::host_vector<Real4>& velMasH);

void CopyD2HPosVel(thrust::host_vector<Real3>& posRadH,
                   thrust::host_vector<Real4>& velMasH,
                   const thrust::host_vector<Real3>& posRadD,
                   const thrust::host_vector<Real4>& velMasD);

void CopyH2D(thrust::device_vector<Real4>& derivVelRhoD, const thrust::host_vector<Real4>& derivVelRhoChronoH);

void CopySys2D(thrust::device_vector<Real3>& posRadD,
               chrono::ChSystemParallelDVI& mphysicalSystem,
               const NumberOfObjects& numObjects,
               int startIndexSph);

void CopyD2H(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
             thrust::host_vector<Real4>& velMasH,
             thrust::host_vector<Real4>& rhoPresMuH,
             const thrust::device_vector<Real3>& posRadD,
             const thrust::device_vector<Real4>& velMasD,
             const thrust::device_vector<Real4>& rhoPresMuD);







void Add_Rigid_ForceTorques_To_ChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
		const thrust::device_vector<Real3>& rigid_FSI_ForcesD,
		const thrust::device_vector<Real3>& rigid_FSI_TorquesD,
		const thrust::host_vector<int>& mapIndex);

void Update_RigidPosVel_from_ChSystem_H2D(
		thrust::device_vector<Real3>& posRigidD,
		thrust::device_vector<Real4>& qD,
		thrust::device_vector<Real4>& velMassRigidD,
		thrust::device_vector<Real3>& rigidOmegaLRF_D,
		const thrust::host_vector<int>& mapIndex,
		chrono::ChSystemParallelDVI& mphysicalSystem);

void HardSet_PosRot_In_ChSystem_D2H(chrono::ChSystemParallelDVI& mphysicalSystem,
		const thrust::device_vector<Real3>& posRigidD,
		const thrust::device_vector<Real4>& qD,
		const thrust::device_vector<Real4>& velMassRigidD,
		const thrust::device_vector<Real3>& omegaLRF_D,
		const thrust::host_vector<int>& mapIndex);

#endif /* SPHINTERFACE_H_ */

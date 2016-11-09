//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

// Program that creates a terrain file(s) with all positions&rotations&vertexes
// of fragmented hulls, for later using with my_example.exe  

// Change this flag to 1 or 0 to enable/disable Irrlicht 3D viewing window
#define USE_IRRLICHT 1

// Change this flag to 1 or 0 to enable/disable cast shadows in 3D view (warning, may be slower)
#define USE_SHADOWS 1



#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/core/ChDistribution.h"
#include "physics/ChPhysicsItem.h"
#include "chrono/core/ChLog.h"
#include "chrono/physics/ChContactContainerBase.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <string>
#include <typeinfo>
#include <vector>
#include <tuple>
#include <algorithm>
#include <functional> 
#include <cctype>
#include <cmath>
#if USE_IRRLICHT
 #include "chrono_irrlicht/ChIrrApp.h"
#endif 

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;
using namespace std;

// Use the main namespaces of Irrlicht
#if USE_IRRLICHT
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
#endif



/// GLOBAL GENERATION SETTINGS HERE

double  timestep = 0.005;
int     density = 1700;           // density of the particle
int     nparticles = 1000;
double  fall_duration = 2;       // time in seconds for waterfalling all particles
double  fall_height = 0.1;       // [m], height from soil for particles to be ejected
double  fall_raise_vel = 0.2;    // [m/s], height of waterfall can increas in time, to allow high stacking
double  settling_duration = 1;   // time in seconds to allow particles to settle, before ending simulation
double  fall_initial_speed = -0.3; // [m/s] initial speed for particles - better neg than zero to avoid initial overlap
int     nvertexes = 16;
bool    do_spheres = false;
bool    do_hulls = true;

ChMinMaxDistribution distribution_size(0.02,0.06);
//ChZhangDistribution distribution_size(0.4, 0.1) // (average, minimum)
//ChContinuumDistribution  distribution_size(msize, mprob) // (PDF distribution curve sampled by x,y points)

int     created_particles = 0; // counter, auto updated.
std::vector< std::shared_ptr<ChBody> > particles; 


// Helper function that generates a particle waterfall, even progressively in separate timesteps

void generate_particles(
                ChSystem& mphysicalSystem,
                ChStreamOutAsciiFile& file_R, 
                ChStreamOutAsciiFile& file_Verts
                #if USE_IRRLICHT
                , ChIrrApp& application
                #endif
                ) 
{
    int target_particles = floor((mphysicalSystem.GetChTime()/fall_duration) * (double)nparticles); 
    if (target_particles > nparticles) 
        target_particles = nparticles;
    int particles_to_do = target_particles - created_particles; 
    if (particles_to_do <=0) 
        return;

    for (int i = 0; i < particles_to_do; i++) { // to eliminate particles out of the cylinder

        double pos_y = fall_height + fall_raise_vel*mphysicalSystem.GetChTime();
        double pos_x = (ChRandom() * 2.0 - 1.0)* 0.75; // RANDOMIZE fall position
        double pos_z = (ChRandom() * 2.0 - 1.0)* 0.75; // RANDOMIZE fall position

        double rx = pos_x;
        double rz = pos_z;
        double rc = sqrt(rx*rx + rz*rz);
        if (rc<0.75) {
            created_particles++;

            if (do_spheres) {
                // RANDOMIZE radius
                double radius = 0.5*distribution_size.GetRandom();
                file_R << radius << "\n";
                
                auto sphereBody = std::make_shared<ChBodyEasySphere>(radius,
                                                                     density,
                                                                     true,
                                                                     true);
                sphereBody->SetBodyFixed(false);
                sphereBody->SetPos(ChVector<double>(pos_x, pos_y, pos_z));
                sphereBody->SetPos_dt(ChVector<>(0,fall_initial_speed,0));
                sphereBody->GetMaterialSurface()->SetFriction(1.0f);
                mphysicalSystem.Add(sphereBody);
                particles.push_back(sphereBody);
                
                #if USE_IRRLICHT
                    application.AssetBind(sphereBody);
                    application.AssetUpdate(sphereBody);
                    #if USE_SHADOWS
                        application.AddShadow(sphereBody);
                    #endif
                #endif
            }
            if (do_hulls) {
                // Randomize n points as vertexes of a polytope (and save them as x y z triplets on a single line of a file)
                std::vector<ChVector<>> vertexes(16);
                for (int iv = 0; iv<vertexes.size(); ++iv) {
                    // RANDOMIZE chord of polytope
                    double chord = distribution_size.GetRandom();
                    vertexes[iv] = ChVector<>(
                        (ChRandom()-0.5) * chord,
                        (ChRandom()-0.5) * chord,
                        (ChRandom()-0.5) * chord
                        ); 
                    file_Verts << vertexes[iv].x << " " << vertexes[iv].y << " " << vertexes[iv].z << " ";
                }
                file_Verts << "\n"; 

                auto hullBody = std::make_shared<ChBodyEasyConvexHull>(vertexes,
                                                                     density,
                                                                     true,
                                                                     true);
                hullBody->SetBodyFixed(false);
                hullBody->SetPos(ChVector<double>(pos_x, pos_y, pos_z));
                hullBody->SetPos_dt(ChVector<>(0,fall_initial_speed,0));
                hullBody->GetMaterialSurface()->SetFriction(1.0f);
                mphysicalSystem.Add(hullBody);
                particles.push_back(hullBody);
                #if USE_IRRLICHT
                    application.AssetBind(hullBody);
                    application.AssetUpdate(hullBody);
                    #if USE_SHADOWS
                        application.AddShadow(hullBody);
                    #endif
                #endif
            }
        }
    }

}


int main(int argc, char* argv[]) {

    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a Chrono physical system
    ChSystem mphysicalSystem;

    #if USE_IRRLICHT
        // Create the Irrlicht visualization (open the Irrlicht device,
        // bind a simple user interface, etc. etc.)
        ChIrrApp application(&mphysicalSystem, L"Terrain generator", core::dimension2d<u32>(800, 600),
                             false);  // screen dimensions

                                      // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
        application.AddTypicalLogo();
        application.AddTypicalSky();
        application.AddTypicalLights();
        application.AddTypicalCamera(core::vector3df(-0.5, 0.5, 1.5),
                                     core::vector3df(0, 0.5, 0));// to change the position of camera
                                                                 // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

        #if USE_SHADOWS
        application.AddLightWithShadow(vector3df(-1.5, 2.0, 0.5), vector3df(0, 0, 0), 3, 0.5, 3.2, 47, 512,
                                       video::SColorf(1.6, 1.6, 1.6));
        #endif

    #endif USE_IRRLICHT

    mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.01);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);



    // 1-Build the container which is cylindrical h=0.4m and radius=0.75m and the floor.


    auto floorBody = std::make_shared<ChBodyEasyBox>(1.6, .1, 1.6,
                                                     1700,
                                                     true,
                                                     true);
    floorBody->SetPos(ChVector<>(0, -.05, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetMaterialSurface()->SetRestitution(0.55f);
    floorBody->GetMaterialSurface()->SetFriction(1.0f);

    mphysicalSystem.Add(floorBody);

    double container_h = 0.9;
    for (int i = 0; i < 180; i++) {
        auto containerBody = std::make_shared<ChBodyEasyBox>(.03, container_h, .03,
                                                             1700,
                                                             true,
                                                             false);
        containerBody->SetPos(ChVector<>(.775*cos(i*CH_C_DEG_TO_RAD*2), container_h*0.5, .775*sin(i*CH_C_DEG_TO_RAD*2)));
        containerBody->SetBodyFixed(true);
        containerBody->GetMaterialSurface()->SetRestitution(0.55f);
        containerBody->GetMaterialSurface()->SetFriction(1.0f);
        mphysicalSystem.Add(containerBody);
    }




    ChStreamOutAsciiFile file_R("terrain_R.txt");
    ChStreamOutAsciiFile file_X("terrain_X.txt");
    ChStreamOutAsciiFile file_Y("terrain_Y.txt");
    ChStreamOutAsciiFile file_Z("terrain_Z.txt");
    ChStreamOutAsciiFile file_Q0("terrain_Q0.txt");
    ChStreamOutAsciiFile file_Q1("terrain_Q1.txt");
    ChStreamOutAsciiFile file_Q2("terrain_Q2.txt");
    ChStreamOutAsciiFile file_Q3("terrain_Q3.txt");
    ChStreamOutAsciiFile file_Verts("terrain_Verts.txt");


  

    //======================================================================


    // Adjust some settings:
    mphysicalSystem.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
    mphysicalSystem.SetSolverType(ChSystem::SOLVER_BARZILAIBORWEIN);
	mphysicalSystem.SetMaxItersSolverSpeed(60);
    mphysicalSystem.SetMaxItersSolverStab(25);
    mphysicalSystem.Set_G_acc(ChVector<>(0, -9.8, 0));
    mphysicalSystem.SetMinBounceSpeed(0.002);


    double simulation_end_time = fall_duration + settling_duration;

    //
    // THE SOFT-REAL-TIME CYCLE
    //


    #if USE_IRRLICHT

        application.SetTimestep(timestep);
        application.SetTryRealtime(true);
        application.SetStepManage(true);

        // Use this function for adding a ChIrrNodeAsset to all items
        // Otherwise use application.AssetBind(myitem); on a per-item basis.
        application.AssetBindAll();

        // Use this function for 'converting' assets into Irrlicht meshes
        application.AssetUpdateAll();


        #if USE_SHADOWS
        // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
        // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
        application.AddShadowAll();
        #endif USE_SHADOWS

        while (application.GetDevice()->run()) {
            application.BeginScene();

            application.DrawAll();

            // generate particle waterfall  
            generate_particles(mphysicalSystem, file_R, file_Verts, application);

            // This performs the integration timestep!
            application.DoStep();

            if (mphysicalSystem.GetChTime() > simulation_end_time) {

                application.GetDevice()->closeDevice();
            }

            application.EndScene();
             
            GetLog() << "Time = " << mphysicalSystem.GetChTime() << ", created particles = " << created_particles << "\n";
        }
    
    #else

        while(mphysicalSystem.GetChTime() < simulation_end_time) {
            
            // generate particle waterfall  
            generate_particles(mphysicalSystem, file_R, file_Verts);

            // This performs the integration timestep!
            mphysicalSystem.DoStepDynamics(timestep);

            GetLog() << "Time = " << mphysicalSystem.GetChTime() << ", created particles = " << created_particles << "\n";
        }


    #endif USE_IRRLICHT


    // at the end of the simulation output all the positions 
    for (int i = 0; i < particles.size(); ++i) {
        file_X << particles[i]->GetPos().x << "\n";
        file_Y << particles[i]->GetPos().y << "\n";
        file_Z << particles[i]->GetPos().z << "\n";
        file_Q0 << particles[i]->GetRot().e0 << "\n";
        file_Q1 << particles[i]->GetRot().e1 << "\n";
        file_Q2 << particles[i]->GetRot().e2 << "\n";
        file_Q3 << particles[i]->GetRot().e3 << "\n";
    }

    
    return 0;
}


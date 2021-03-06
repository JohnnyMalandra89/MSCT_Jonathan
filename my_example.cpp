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

// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.

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





int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a Chrono physical system
    ChSystem mphysicalSystem;

    #if USE_IRRLICHT
        // Create the Irrlicht visualization (open the Irrlicht device,
        // bind a simple user interface, etc. etc.)
        ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(800, 600),
                             false);  // screen dimensions

                                      // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
        application.AddTypicalLogo();
        application.AddTypicalSky();
        application.AddTypicalLights();
        application.AddTypicalCamera(core::vector3df(-0.5, 0.5, 1.5),
                                     core::vector3df(0, 0.5, 0));// to change the position of camera
                                                                 // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

                                                                 //======================================================================

                                                                 // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.

                                                                 // Here I set up the parameters of the simulation for the collisions
                                                                 //mphysicalSystem.SetMinBounceSpeed(0.005);
        #if USE_SHADOWS
        application.AddLightWithShadow(vector3df(-1.5, 2.0, 0.5), vector3df(0, 0, 0), 3, 0.5, 3.2, 47, 512,
                                       video::SColorf(1.6, 1.6, 1.6));
        #endif

    #endif USE_IRRLICHT

    mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.01);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    double GLOBAL_max_simulation_time = 20.0;
    double GLOBAL_timestep = 0.005;


    // 1-Build the container which is cylindrical h=0.4m and radius=0.75m and the floor.

    //auto material_spheres = std::make_shared<ChMaterialSurface>();
    //material_spheres->SetRestitution(0.55f);
    //material_spheres->SetFriction(1.0f);
    //material_spheres->SetRollingFriction(.0105f);

    auto floorBody = std::make_shared<ChBodyEasyBox>(1.6, .1, 1.6,
                                                     1700,
                                                     true,
                                                     true);
    floorBody->SetPos(ChVector<>(0, -.05, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetMaterialSurface()->SetRestitution(0.55f);
    floorBody->GetMaterialSurface()->SetFriction(1.0f);
    floorBody->GetMaterialSurface()->SetRollingFriction(.0105f);
    floorBody->GetMaterialSurface()->SetSpinningFriction(.0105f);


    mphysicalSystem.Add(floorBody);
    /*auto mtexturecylinder = std::make_shared<ChTexture>();
    mtexturecylinder->SetTextureFilename(GetChronoDataFile("rock.jpg"));
    floorBody->AddAsset(mtexturecylinder);*/
    auto color_floor = std::make_shared<ChColorAsset>();
    color_floor->SetColor(ChColor(2.0f, 0.05f, 0.0f));
    floorBody->AddAsset(color_floor);

    for (int i = 0; i < 180; i++) {
        auto containerBody = std::make_shared<ChBodyEasyBox>(.03, .15, .03,
                                                             1700,
                                                             true,
                                                             false);
        containerBody->SetPos(ChVector<>(.775*cos(i*CH_C_DEG_TO_RAD*2), .075, .775*sin(i*CH_C_DEG_TO_RAD*2)));
        containerBody->SetBodyFixed(true);
        containerBody->GetMaterialSurface()->SetRestitution(0.55f);
        containerBody->GetMaterialSurface()->SetFriction(1.0f);
        containerBody->GetMaterialSurface()->SetRollingFriction(.0105f);
		containerBody->GetMaterialSurface()->SetSpinningFriction(.0105f);
        mphysicalSystem.Add(containerBody);
        //auto mtexturecontainer = std::make_shared<ChTexture>();
        //mtexturecontainer->SetTextureFilename(GetChronoDataFile("rock.jpg"));
        //containerBody->AddAsset(mtexturecontainer);
    }

    // Floor, container and ground share the same physical properties.

    // 2-Import the ground

    // I have a ground create with YADE, so I have uploaded it in chrono engine in order to avoid differences due to differents position of the particles.
    //std::string xx;
    //double radius = .005; // radius of the particle
    int density = 1700; // density of the particle
                        //double volume = 4 / 3 * 3.14 * pow(radius,3); // volume of the particle
                        //double mass = density*volume; // mass of the particle
                        //const int nspheres = 688;
    int nspheres;

	cout << "Enter the number of particles: \n";
	cin >> nspheres;
    //double posx[nspheres];
    //ifstream readin1("X1cm.txt");
    //assert(readin1.is_open());
    ////myfile0.open("Xrest2.txt"); // x position of each particle
    ////myfile0.open("X1cm.txt");
    //for (int i = 0; i < nspheres; i++) {
    //    //getline(myfile0, xx);
    //    readin1 >> posx[i];
    //    //std::string::size_type sz;
    //    //double x = std::stod(xx, &sz);
    //    //posx[i] = x;
    //}
    ////std::string yy;
    //double posy[nspheres];
    //ifstream readin2("Z1cm.txt");
    ////myfile1.open("Yrest2.txt"); // y position of each particle
    ////myfile1.open("Z1cm.txt");
    //for (int i = 0; i < nspheres; i++) {
    //    readin2 >> posy[i];
    //    /*getline(myfile1, yy);
    //    std::string::size_type sz;
    //    double y = std::stod(yy, &sz);
    //    posy[i] = y;*/
    //}
    ////std::string zz;
    //double posz[nspheres];
    //ifstream readin3("Y1cm.txt");
    ////myfile2.open("Zrest2.txt"); // z position of each particle
    ////myfile2.open("Y1cm.txt");
    //for (int i = 0; i < nspheres; i++) {
    //    readin3 >> posz[i];
    //    //getline(myfile2, zz);
    //    //std::string::size_type sz;
    //    //double z = std::stod(zz, &sz);
    //    //posz[i] = z;
    //}
    ////std::string rr;
    //double radius[nspheres];
    //ifstream readin_radius("R1cm.txt");
    ////myfile3.open("R1cm.txt");
    //for (int i = 0; i < nspheres; i++) {
    //    readin_radius >> radius[i];
    //    //radius[i] = radius[i] * 2;
    //    /*getline(myfile3, rr);
    //    std::string::size_type sr;
    //    double r = std::stod(rr, &sr);
    //    radius[i] = r;*/
    //}

    ifstream readin_radius("R1cm.txt");
    ifstream readin3("Y1cm.txt");
    ifstream readin2("Z1cm.txt");
    ifstream readin1("X1cm.txt");


    bool do_spheres = false;
    bool do_hulls = true;
    double radius_temp = 0;
    double pos_x, pos_y, pos_z;
    for (int i = 0; i < nspheres; i++) { // to eliminate particles out of the cylinder
        readin_radius >> radius_temp;
        readin3 >> pos_z;
        readin2 >> pos_y;
        readin1 >> pos_x;

        double rx = pos_x;
        double rz = pos_z;
        double rc = sqrt(rx*rx + rz*rz);
        if (pos_y <= .30 && pos_y>0 && rc<0.75) {
            if (do_spheres) {
                //double volume = 4. / 3. * 3.14 * pow(radius[i], 3); // volume of the particle
                //double mass = density*volume; // mass of the particle
                auto sphereBody = std::make_shared<ChBodyEasySphere>(radius_temp,
                                                                     density,
                                                                     true,
                                                                     true);
                sphereBody->SetBodyFixed(false);
                sphereBody->SetPos(ChVector<double>(pos_x, pos_y, pos_z));
                sphereBody->GetMaterialSurface()->SetFriction(1.0f);
                sphereBody->GetMaterialSurface()->SetRollingFriction(1.05*radius_temp); 
			    sphereBody->GetMaterialSurface()->SetSpinningFriction(1.05*radius_temp); 
                sphereBody->GetMaterialSurface()->SetRestitution(.55f);
                /*sphereBody->SetIdentifier(1);*/
                mphysicalSystem.Add(sphereBody);
            }
            if (do_hulls) {
                // Randomize n points as vertexes of a polytope (or read them from a file?)
                std::vector<ChVector<>> vertexes(16);
                for (int iv = 0; iv<vertexes.size(); ++iv) {
                    vertexes[iv] = ChVector<>(
                        ((ChRandom()*2)-1.0) * radius_temp,
                        ((ChRandom()*2)-1.0) * radius_temp,
                        ((ChRandom()*2)-1.0) * radius_temp
                        ); 
                }
                auto sphereBody = std::make_shared<ChBodyEasyConvexHull>(vertexes,
                                                                     density,
                                                                     true,
                                                                     true);
                sphereBody->SetBodyFixed(false);
                sphereBody->SetPos(ChVector<double>(pos_x, pos_y, pos_z));
                sphereBody->GetMaterialSurface()->SetFriction(1.0f);
                sphereBody->GetMaterialSurface()->SetRestitution(.55f);
                /*sphereBody->SetIdentifier(1);*/
                mphysicalSystem.Add(sphereBody);
            }
        }
    }
    //myfile0.close();
    //myfile1.close();
    //myfile2.close();
    //myfile3.close();

    // 3-Create MASCOT

    auto mascot = std::make_shared<ChBodyAuxRef>();
    mascot->SetBodyFixed(false);

    // Define material for MASCOT
    auto material_mascot = std::make_shared<ChMaterialSurface>();
    material_mascot->SetRestitution(0.55f);
    material_mascot->SetFriction(1.0f);
    material_mascot->SetRollingFriction(.0105f);
    material_mascot->SetSpinningFriction(.0105f);

    // Define a collision shape
    ChVector<double> mascot_dimensions = { 0.2774 ,0.1973, 0.2922 };
    mascot->GetCollisionModel()->ClearModel();
    mascot->GetCollisionModel()->AddBox(mascot_dimensions(0) / 2., mascot_dimensions(1) / 2., mascot_dimensions(3) / 2., ChVector<>(0, 0, 0)); //***TODO*** POS RESPECT TO REF!!
    mascot->GetCollisionModel()->BuildModel();
    mascot->SetCollide(true);

    // ==Asset== attach a 'box' shape.
    auto mascot_box = std::make_shared<ChBoxShape>();
    mascot_box->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
    mascot_box->GetBoxGeometry().Size = ChVector<>(mascot_dimensions(0) / 2.0, mascot_dimensions(1) / 2.0, mascot_dimensions(3) / 2.0);
    mascot->AddAsset(mascot_box);


    //// Optionally, attach a RGB color asset to the floor, for better visualization
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.6f, 0.45f, 0.0f));
    mascot->AddAsset(color);

	ChMatrix33<> inertia;
	inertia(0, 0) = 0.099; inertia(0, 1) = -0.0051; inertia(0, 2) = 0.0019;
	inertia(1, 0) = -0.0051; inertia(1, 1) = 0.0825; inertia(1, 2) = 0.0005;
	inertia(2, 0) = 0.0019; inertia(2, 1) = 0.0005; inertia(2, 2) = 0.121;
	mascot->SetInertia(inertia);
    mascot->SetMass(10);
    mascot->SetMaterialSurface(material_mascot);

    mascot->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, .43, 0))); // ***NOTE*** would be better than SetPos(), that in ChBodyAuxRef would pick the object by its COG

    ChVector<> my_COG_to_REF_offset(0.01257, 0.00296, -0.00757);  // note: use AFTER SetFrame_REF_to_abs  ****NOTE*** UPDATE VALUES AS IN CAD
    mascot->SetFrame_COG_to_REF(ChFrame<>(my_COG_to_REF_offset));

    mascot->SetPos_dt(ChVector<>(0, -.19, 0));
    mascot->SetWvel_loc(ChVector<>(0, 0, 0));

    // Add MASCOT to system
    mphysicalSystem.Add(mascot);


    //======================================================================


    // Adjust some settings:
    mphysicalSystem.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
    mphysicalSystem.SetSolverType(ChSystem::SOLVER_BARZILAIBORWEIN);
	mphysicalSystem.SetMaxItersSolverSpeed(60);
    mphysicalSystem.SetMaxItersSolverStab(25);
    mphysicalSystem.Set_G_acc(ChVector<>(0, -2.5e-4, 0));
    mphysicalSystem.SetMinBounceSpeed(0.002);


    ofstream myfile4;
    myfile4.open("data.txt"); // to save angular and linear speed and the position of MASCOT


    //
    // THE SOFT-REAL-TIME CYCLE
    //


    #if USE_IRRLICHT

        application.SetTimestep(GLOBAL_timestep);
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

            // This performs the integration timestep!
            application.DoStep();

            ChVector<> W = mascot->GetWvel_loc();
            ChVector<> V = mascot->GetPos_dt();
            ChVector<> P = mascot->GetPos();
            double t = mphysicalSystem.GetChTime();
            myfile4 << t << "," << W(0) << "," << W(1) << "," << W(2) << "," << V(0) << "," << V(1) << "," << V(2) << "," << P(0) << "," << P(1) << "," << P(2) << endl;
            cout << mphysicalSystem.GetChTime() << endl;

            if (mphysicalSystem.GetChTime() > GLOBAL_max_simulation_time) {

                application.GetDevice()->closeDevice();
            }

            application.EndScene();
        }
    
    #else

        while(mphysicalSystem.GetChTime() < GLOBAL_max_simulation_time) {
            
            mphysicalSystem.DoStepDynamics(GLOBAL_timestep);

            ChVector<> W = mascot->GetWvel_loc();
            ChVector<> V = mascot->GetPos_dt();
            ChVector<> P = mascot->GetPos();
            double t = mphysicalSystem.GetChTime();
            myfile4 << t << "," << W(0) << "," << W(1) << "," << W(2) << "," << V(0) << "," << V(1) << "," << V(2) << "," << P(0) << "," << P(1) << "," << P(2) << endl;
            cout << mphysicalSystem.GetChTime() << endl;

        }


    #endif USE_IRRLICHT

   

    
    myfile4.close();
    return 0;
}


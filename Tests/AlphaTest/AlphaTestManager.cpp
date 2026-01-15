/*
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  AlphaTestManager.cpp
//  Stonefish - Alpha AUV 0114 Coordinate Test
//
//  Created for testing 0114.obj coordinate system
//

#include "AlphaTestManager.h"

#include <entities/solids/Polyhedron.h>
#include <actuators/Thruster.h>
#include <actuators/Servo.h>
#include <utils/UnitSystem.h>
#include <utils/SystemUtil.hpp>
#include <core/FeatherstoneRobot.h>
#include <entities/FeatherstoneEntity.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/Sample.h>
#include <iostream>

AlphaTestManager::AlphaTestManager(sf::Scalar stepsPerSecond)
   : SimulationManager(stepsPerSecond, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE)
{
}

void AlphaTestManager::BuildScenario()
{
    std::cout << "=== AlphaTest BuildScenario() started ===" << std::endl;

    ///////MATERIALS////////
    std::cout << "Creating materials..." << std::endl;
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.3);
    CreateMaterial("Aluminum", sf::UnitSystem::Density(sf::CGS, sf::MKS, 2.7), 0.3);
    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Aluminum", "Aluminum", 0.3, 0.1);

    ///////LOOKS///////////
    CreateLook("yellow", sf::Color::RGB(1.0f, 0.9f, 0.0f), 0.4f, 0.8f);
    CreateLook("white", sf::Color::Gray(1.0f), 0.9f, 0.0f);
    CreateLook("gray", sf::Color::Gray(0.5f), 0.3f, 0.5f);
    CreateLook("propeller", sf::Color::Gray(0.7f), 0.3f, 0.0f);

    ////////ENVIRONMENT
    // Create underwater environment
    // EnableOcean(0.0) sets ocean surface at z=0
    // Objects with z < 0 are underwater
    EnableOcean(0.0);
    getOcean()->setWaterType(0.2);  // Clear water (Jerlov type)
    getAtmosphere()->SetSunPosition(45.0, 60.0);  // Azimuth 45°, Elevation 60°

    ////////ALPHA AUV 0114
    // Hull with coordinate system test - rpy = (0, 0, 0)
    sf::PhysicsSettings phy;
    phy.mode = sf::PhysicsMode::SUBMERGED;  // Fully submerged
    phy.collisions = true;
    phy.buoyancy = true;

    // Create hull using separate physics and graphics meshes (like FloatingTest)
    // full.obj = graphics mesh (detailed, like boat_gra.obj)
    // base.obj = physics mesh (simplified, like boat.obj)
    std::string graphicsPath = "/home/cloudpark/underwater_sim_ws/full.obj";
    std::string physicsPath = "/home/cloudpark/underwater_sim_ws/base.obj";
    std::cout << "Loading AUV graphics mesh from: " << graphicsPath << std::endl;
    std::cout << "Loading AUV physics mesh from: " << physicsPath << std::endl;

    // Hull (like FloatingTest boat pattern)
    sf::Polyhedron* hull = new sf::Polyhedron(
        "AlphaHull",
        phy,
        graphicsPath,               // Graphics mesh (full.obj, like boat_gra.obj)
        sf::Scalar(1.0),            // Graphics scale
        sf::I4(),                   // Graphics transform (identity)
        physicsPath,                // Physics mesh (base.obj, like boat.obj)
        sf::Scalar(1.0),            // Physics scale
        sf::I4(),                   // Physics transform (identity)
        "Fiberglass",               // Material
        "yellow",                   // Look
        sf::Scalar(0.01)            // Shell thickness (like boat 0.09)
    );
    // Scale mass - slightly heavier for better stability
    hull->ScalePhysicalPropertiesToArbitraryMass(25.0);

    ////////WINGS - Control surfaces for pitch/dive control
    // Wing physics settings - enable buoyancy for balance
    sf::PhysicsSettings wingPhy;
    wingPhy.mode = sf::PhysicsMode::SUBMERGED;
    wingPhy.collisions = false;
    wingPhy.buoyancy = true;  // Enable buoyancy for lateral balance

    // Wing paths
    std::string rightWingPath = "/home/cloudpark/underwater_sim_ws/rightwing.obj";
    std::string leftWingPath = "/home/cloudpark/underwater_sim_ws/leftwing.obj";

    // Right wing (starboard) - root at Y=0.1, centered around X=-0.425
    sf::Polyhedron* rightWing = new sf::Polyhedron(
        "RightWing", wingPhy,
        rightWingPath, sf::Scalar(1.0), sf::I4(),
        "Aluminum", "gray"
    );
    rightWing->ScalePhysicalPropertiesToArbitraryMass(0.5);

    // Left wing (port) - root at Y=-0.1, centered around X=-0.425
    sf::Polyhedron* leftWing = new sf::Polyhedron(
        "LeftWing", wingPhy,
        leftWingPath, sf::Scalar(1.0), sf::I4(),
        "Aluminum", "gray"
    );
    leftWing->ScalePhysicalPropertiesToArbitraryMass(0.5);

    ////////CRP (Contra-Rotating Propellers)
    // Propeller physics - no buoyancy effect
    sf::PhysicsSettings propPhy;
    propPhy.mode = sf::PhysicsMode::SUBMERGED;
    propPhy.collisions = false;
    propPhy.buoyancy = false;

    // Front propeller (right-handed)
    std::shared_ptr<sf::Polyhedron> propellerFront = std::make_shared<sf::Polyhedron>(
        "PropellerFront", propPhy,
        sf::GetDataPath() + "propeller.obj", sf::Scalar(1), sf::I4(),
        "Fiberglass", "propeller"
    );

    // Rear propeller (left-handed, mirrored) - flip on X axis
    sf::Transform mirrorTransform = sf::Transform(sf::IQ(), sf::Vector3(0, 0, 0));
    mirrorTransform.setBasis(sf::Matrix3(
        -1, 0, 0,
        0, 1, 0,
        0, 0, 1
    ));
    std::shared_ptr<sf::Polyhedron> propellerRear = std::make_shared<sf::Polyhedron>(
        "PropellerRear", propPhy,
        sf::GetDataPath() + "propeller.obj", sf::Scalar(1), mirrorTransform,
        "Fiberglass", "propeller"
    );

    // Rotor dynamics: Kp, Ki, iClamp, maxSpeed
    std::shared_ptr<sf::MechanicalPI> rotorDynamicsFront = std::make_shared<sf::MechanicalPI>(1.0, 10.0, 5.0, 5.0);
    std::shared_ptr<sf::MechanicalPI> rotorDynamicsRear = std::make_shared<sf::MechanicalPI>(1.0, 10.0, 5.0, 5.0);

    // Thrust models - front right-handed, rear left-handed (opposite rotation)
    std::shared_ptr<sf::FDThrust> thrustModelFront = std::make_shared<sf::FDThrust>(
        0.18, 0.48, 0.48, 0.05, true, getOcean()->getLiquid().density);  // right-handed
    std::shared_ptr<sf::FDThrust> thrustModelRear = std::make_shared<sf::FDThrust>(
        0.18, 0.48, 0.48, 0.05, false, getOcean()->getLiquid().density); // left-handed

    // Create CRP thrusters
    sf::Thruster* thrusterFront = new sf::Thruster(
        "ThrusterFront", propellerFront, rotorDynamicsFront, thrustModelFront,
        0.18, true, 105.0, false, true
    );
    sf::Thruster* thrusterRear = new sf::Thruster(
        "ThrusterRear", propellerRear, rotorDynamicsRear, thrustModelRear,
        0.18, false, 105.0, false, true  // left-handed rotation
    );

    // Add odometry sensor
    sf::Odometry* odom = new sf::Odometry("Odom");

    // Create robot with wings
    sf::FeatherstoneRobot* alpha = new sf::FeatherstoneRobot("AlphaAUV");

    // Define links: hull (base) + wings
    std::vector<sf::SolidEntity*> wingLinks;
    wingLinks.push_back(rightWing);
    wingLinks.push_back(leftWing);
    alpha->DefineLinks(hull, wingLinks);

    // Define revolute joints for wing pitch control
    // Wing root is at mesh origin (0,0,0)
    // Joint origin places wing on hull at (-0.425, ±0.1, 0)
    alpha->DefineRevoluteJoint(
        "RightWingJoint",
        "AlphaHull",
        "RightWing",
        sf::Transform(sf::IQ(), sf::Vector3(-0.425, 0.1, 0.0)),
        sf::Vector3(0, 1, 0),  // Y-axis rotation for pitch
        std::make_pair(sf::Scalar(-0.5), sf::Scalar(0.5)),
        sf::Scalar(1.0)  // Add damping to stabilize
    );

    alpha->DefineRevoluteJoint(
        "LeftWingJoint",
        "AlphaHull",
        "LeftWing",
        sf::Transform(sf::IQ(), sf::Vector3(-0.425, -0.1, 0.0)),
        sf::Vector3(0, 1, 0),
        std::make_pair(sf::Scalar(-0.5), sf::Scalar(0.5)),
        sf::Scalar(1.0)  // Add damping
    );

    alpha->BuildKinematicStructure();

    // Servo actuators - use lower gains like JointsTest
    sf::Servo* rightWingServo = new sf::Servo("RightWingServo", 1.0, 1.0, 100.0);
    rightWingServo->setControlMode(sf::ServoControlMode::POSITION);
    alpha->AddJointActuator(rightWingServo, "RightWingJoint");

    sf::Servo* leftWingServo = new sf::Servo("LeftWingServo", 1.0, 1.0, 100.0);
    leftWingServo->setControlMode(sf::ServoControlMode::POSITION);
    alpha->AddJointActuator(leftWingServo, "LeftWingJoint");

    // Add CRP thrusters at stern (rear) of AUV
    // Hull stern is at x = -0.91m, propellers attached close to hull
    // Front propeller closer to hull, rear propeller behind it
    alpha->AddLinkActuator(thrusterFront, "AlphaHull", sf::Transform(
        sf::Quaternion(0, 0, 0),
        sf::Vector3(-0.92, 0.0, 0.0)  // Front propeller - close to hull
    ));
    alpha->AddLinkActuator(thrusterRear, "AlphaHull", sf::Transform(
        sf::Quaternion(0, 0, 0),
        sf::Vector3(-0.97, 0.0, 0.0)  // Rear propeller - 5cm behind front
    ));

    alpha->AddLinkSensor(odom, "AlphaHull", sf::I4());

    // Add robot at water surface (z=0.05)
    AddRobot(alpha, sf::Transform(sf::IQ(), sf::Vector3(0, 0, 0.05)));

    // Set CRP thruster setpoints (opposite rotation for contra-rotating)
    thrusterFront->setSetpoint(50);   // Forward rotation
    thrusterRear->setSetpoint(-50);   // Reverse rotation (contra-rotating)

    std::cout << "=== AlphaTest BuildScenario() completed ===" << std::endl;
}

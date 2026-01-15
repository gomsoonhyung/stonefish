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
//  main.cpp
//  AlphaTest
//
//  Alpha AUV 0114 Coordinate System Test
//

#include <core/GraphicalSimulationApp.h>
#include "AlphaTestManager.h"

int main(int argc, const char * argv[])
{
    sf::RenderSettings s;
    s.windowW = 1600;
    s.windowH = 900;
    s.aa = sf::RenderQuality::HIGH;
    s.shadows = sf::RenderQuality::MEDIUM;
    s.ao = sf::RenderQuality::MEDIUM;
    s.atmosphere = sf::RenderQuality::HIGH;
    s.ocean = sf::RenderQuality::HIGH;
    s.verticalSync = true;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = true;        // Show coordinate system!
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = true;          // Show forces for debugging

    AlphaTestManager* simulationManager = new AlphaTestManager(500.0);
    sf::GraphicalSimulationApp app("AlphaTest - 0114 Coordinate Check", std::string(DATA_DIR_PATH), s, h, simulationManager);
    app.Run();

    return 0;
}

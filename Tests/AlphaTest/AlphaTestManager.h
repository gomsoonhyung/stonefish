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
//  AlphaTestManager.h
//  Stonefish - Alpha AUV 0114 Coordinate Test
//
//  Created for testing 0114.obj coordinate system
//

#ifndef __Stonefish__AlphaTestManager__
#define __Stonefish__AlphaTestManager__

#include <core/SimulationManager.h>

class AlphaTestManager : public sf::SimulationManager
{
public:
    AlphaTestManager(sf::Scalar stepsPerSecond);

    void BuildScenario();
};

#endif

/* Copyright (c) 2019, Acme Robotics, Ethan Quist, Corbyn Yhap
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "Coordinate.hpp"

Coordinate::Coordinate(double aX, double aY, double aZ)
    :
    x(aX),
    y(aY),
    z(aZ) {

}

Coordinate::~Coordinate() {

}
double Coordinate::getX() {
  return x;
}
double Coordinate::getY() {
  return y;
}

double Coordinate::getZ() {
  return z;
}

void Coordinate::setX(const double &aX) {
  x = aX;
}

void Coordinate::setY(const double &aY) {
  y = aY;
}

void Coordinate::setZ(const double &aZ) {
  z = aZ;
}

void Coordinate::setXYZ(const double &aX, const double &aY, const double &aZ) {
  x = aX;
  y = aY;
  z = aZ;
}
bool Coordinate::operator==(const Coordinate &rhs) const {
  return (x == rhs.x && y == rhs.y && z == rhs.z);
}

double Coordinate::convertM2F(double aMeters) {
  (void) aMeters;
  return 0;
}

double Coordinate::convertF2M(double aFeet) {
  (void) aFeet;
  return 0;
}

Coordinate::Coordinate()
    :
    x(0),
    y(0),
    z(0) {

}

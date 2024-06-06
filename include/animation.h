#ifndef ANIMATION_H_INCLUDED
#define ANIMATION_H_INCLUDED

#include "geometry.h"


class Animation
{
private:
    double phi, theta; // Azimuthal and polar angles for local coordinate system orientation
    double mass; // Mass of the object
    Vector acc, spd; //  Instantaneous acceleration and speed
    Point pos; // Instantaneous position of the local coordinate system origin

public:
    Animation(double ph = 0.0, double th = 0.0,
              Vector accel = Vector(0.0, 0.0, 0.0),
              Vector speed = Vector(0.0, 0.0, 0.0),
              Point p = Point(0.0, 0.0, 0.0),
                double m = 1.0
              );
    double getPhi() const {return phi;}
    double getTheta() const {return theta;}
    void setPhi(double agl) {phi = agl;}
    void setTheta(double agl) {theta = agl;}
    Vector getAccel() const {return acc;}
    Vector getSpeed() const {return spd;}
    void setAccel(Vector vect) {acc = vect;}
    void setSpeed(Vector vect) {spd = vect;}
    void setmass(double m) {mass = m;}
    double getmass() const {return mass;}
    Point getPos() const {return pos;}
    void setPos(Point pt) {pos = pt;}
};


#endif // ANIMATION_H_INCLUDED

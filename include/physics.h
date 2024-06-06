#ifndef PHYSICS_H_INCLUDED
#define PHYSICS_H_INCLUDED

#include "forms.h"

const double g = -9.81;

void gravity(double delta_t, Animation &anim)
{
    double k = 0.1;
    anim.setAccel(Vector(-k * anim.getSpeed().x, anim.getmass()*g, -k * anim.getSpeed().z));
    anim.setSpeed(Vector(anim.getSpeed().x + delta_t * anim.getAccel().x,
                         anim.getSpeed().y + delta_t * anim.getAccel().y,
                         anim.getSpeed().z + delta_t * anim.getAccel().z));
    anim.setPos(Point(anim.getPos().x + delta_t * anim.getSpeed().x + 0.5 * delta_t * delta_t * anim.getAccel().x,
                      anim.getPos().y + delta_t * anim.getSpeed().y + 0.5 * delta_t * delta_t * anim.getAccel().y,
                      anim.getPos().z + delta_t * anim.getSpeed().z + 0.5 * delta_t * delta_t * anim.getAccel().z));
}

void solid(Animation &anim)
{
    if (anim.getPos().y < 0.0)
    {
        anim.setSpeed(Vector(anim.getSpeed().x*0.95, -anim.getSpeed().y * 0.10, anim.getSpeed().z*0.95));
        anim.setPos(Point(anim.getPos().x, -anim.getPos().y * 0.10, anim.getPos().z));
    }
}

void CollisionResponse(Animation &anim, Cuboid c1, Cuboid c2, Point p)
{
    Vector v2 = c2.getAnim().getSpeed();
    double m1 = c1.getMass();
    double m2 = c2.getMass();
    Vector v1 = c1.getAnim().getSpeed();
    double Ecx = 0.5 * m1 * v1.x*v1.x + 0.5 * m2 * v2.x*v2.x;
    double Ecy = 0.5 * m1 * v1.y*v1.y + 0.5 * m2 * v2.y*v2.y;
    double Ecz = 0.5 * m1 * v1.z*v1.z + 0.5 * m2 * v2.z*v2.z;
    double vx = sqrt(2 * Ecx / (m1 + m2));
    double vy = sqrt(2 * Ecy / (m1 + m2));
    double vz = sqrt(2 * Ecz / (m1 + m2));
    Vector v = Vector(vx, vy, vz);
    c1.getAnim().setSpeed(v);
    c1.getAnim().setPos(p);
}

void CollisionResponse2(Animation &anim, Cuboid c, Point p, double delta_t)
{
    // Make the cube rotate around theta and stop the movement if it hits the ground
    double theta = anim.getTheta(); // rotation angle in radians

    // Rotate the cube around theta
    if(theta <= 90.0){
    glPushMatrix();
    anim.setTheta(theta + 3 - delta_t * g);
    c.getAnim().setTheta(theta + 3 - delta_t * g);
    glRotated(theta, 0.0, 0.0, -1.0);
    glPopMatrix();
    }
    // Stop the movement if it hits the ground
    if (anim.getPos().y < 0.0)
    {
        anim.setSpeed(Vector(0.0, 0.0, 0.0));
        anim.setPos(Point(anim.getPos().x, 0.0, anim.getPos().z));
    }

}
void CollisionResponse3(Animation &anim, Cuboid c, Point p, double delta_t, double w)
{
    
    anim.setPos(Point(anim.getPos().x*cos(w*delta_t) - anim.getPos().y*sin(w*delta_t),
                      anim.getPos().x*sin(w*delta_t) + anim.getPos().y*cos(w*delta_t),
                      anim.getPos().z));
}



#endif // PHYSICS_H_INCLUDED
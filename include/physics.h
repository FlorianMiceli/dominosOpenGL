#ifndef PHYSICS_H_INCLUDED
#define PHYSICS_H_INCLUDED

#include "geometry.h"
#include "animation.h"

void gravity(double delta_t, Animation &anim)
{
    double k = 0.5;
    anim.setAccel(Vector(-k * anim.getSpeed().x, -9.81, -k * anim.getSpeed().z));
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
        anim.setSpeed(Vector(anim.getSpeed().x, -anim.getSpeed().y * 0.3, anim.getSpeed().z));
        anim.setPos(Point(anim.getPos().x, -anim.getPos().y * 0.3, anim.getPos().z));
    }
}

// void checkCollision(Cuboid &c1, Cuboid &c2)
// {
//     return (c1.getPos().x < c2.getPos().x + c2.getSize().x &&
//             c1.getPos().x + c1.getSize().x > c2.getPos().x &&
//             c1.getPos().y < c2.getPos().y + c2.getSize().y &&
//             c1.getPos().y + c1.getSize().y > c2.getPos().y &&
//             c1.getPos().z < c2.getPos().z + c2.getSize().z &&
//             c1.getPos().z + c1.getSize().z > c2.getPos().z);
// }

#endif // PHYSICS_H_INCLUDED
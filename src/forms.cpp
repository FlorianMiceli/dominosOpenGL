#include <cmath>
#include <SDL2/SDL_opengl.h>
#include <GL/GLU.h>
#include "forms.h"
#include <algorithm>
#include "physics.h"

void Form::update(double delta_t)
{
    // Nothing to do here, animation update is done in child class method
}

void Form::render()
{
    // Point of view for rendering
    // Common for all Forms
    Point org = anim.getPos();
    glTranslated(org.x, org.y, org.z);
    glColor3f(col.r, col.g, col.b);
}

Sphere::Sphere(double r, Point org, Color cl)
{
    radius = r;
    anim.setPos(org);
    col = cl;
    anim.setSpeed(Vector(0.0, 0.0, -2.0));
}

void Sphere::update(double delta_t)
{
    gravity(delta_t, anim);
    solid(anim);
}

void Sphere::render()
{
    GLUquadric *quad;
    quad = gluNewQuadric();

    Form::render();

    gluSphere(quad, radius, 20, 20);
    gluDeleteQuadric(quad);
}

Cube_face::Cube_face(Vector v1, Vector v2, Point org, double l, double w, Color cl)
{
    vdir1 = 1.0 / v1.norm() * v1;
    vdir2 = 1.0 / v2.norm() * v2;
    anim.setPos(org);
    length = l;
    width = w;
    col = cl;
}

void Cube_face::update(double delta_t)
{
    // Complete this part
}

void Cube_face::render()
{
    Point p1 = Point();
    Point p2 = p1, p3, p4 = p1;
    p2.translate(length * vdir1);
    p3 = p2;
    p3.translate(width * vdir2);
    p4.translate(width * vdir2);

    Form::render();

    glBegin(GL_QUADS);
    {
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glVertex3d(p4.x, p4.y, p4.z);
    }
    glEnd();
}

Cuboid::Cuboid(Vector v1, Vector v2, Vector v3, Point org, double l, double w, double h, Color cl)
{
    vdir1 = 1.0 / v1.norm() * v1;
    vdir2 = 1.0 / v2.norm() * v2;
    vdir3 = 1.0 / v3.norm() * v3;
    anim.setPos(org);
    length = l;
    width = w;
    height = h;
    col = cl;
}

void Cuboid::update(double delta_t)
{
    gravity(delta_t, anim);
    solid(anim);
}

void Cuboid::render()
{
    Point p1 = Point();
    Point p2 = p1, p3, p4 = p1;
    Point p5 = p1, p6 = p1, p7 = p1, p8 = p1;
    p2.translate(length * vdir1);
    p3 = p2;
    p3.translate(width * vdir2);
    p4.translate(width * vdir2);
    p5.translate(height * vdir3);
    p6 = p5;
    p6.translate(length * vdir1);
    p7 = p6;
    p7.translate(width * vdir2);
    p8 = p5;
    p8.translate(width * vdir2);

    Form::render();

    glBegin(GL_QUADS);
    {
        // Right face
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glVertex3d(p4.x, p4.y, p4.z);

        // Top face
        glVertex3d(p4.x, p4.y, p4.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glVertex3d(p7.x, p7.y, p7.z);
        glVertex3d(p8.x, p8.y, p8.z);

        // Left face
        glVertex3d(p8.x, p8.y, p8.z);
        glVertex3d(p7.x, p7.y, p7.z);
        glVertex3d(p6.x, p6.y, p6.z);
        glVertex3d(p5.x, p5.y, p5.z);

        // Bottom face
        glVertex3d(p5.x, p5.y, p5.z);
        glVertex3d(p6.x, p6.y, p6.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p1.x, p1.y, p1.z);

        // Front face
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p4.x, p4.y, p4.z);
        glVertex3d(p8.x, p8.y, p8.z);
        glVertex3d(p5.x, p5.y, p5.z);

        // Back face
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p6.x, p6.y, p6.z);
        glVertex3d(p7.x, p7.y, p7.z);
        glVertex3d(p3.x, p3.y, p3.z);
    }

    glEnd();
}

Point Cube_face::collision(Vector& v)
{
    Point p1 = anim.getPos();
    Point p2 = p1, p3, p4 = p1;
    p2.translate(length*vdir1);
    p3 = p2;
    p3.translate(width*vdir2);
    p4.translate(width*vdir2);

    float minAx = std::min({p1.x, p2.x, p3.x, p4.x});
    float maxAx = std::max({p1.x, p2.x, p3.x, p4.x});
    float minAy = std::min({p1.y, p2.y, p3.y, p4.y});
    float maxAy = std::max({p1.y, p2.y, p3.y, p4.y});
    float minAz = std::min({p1.z, p2.z, p3.z, p4.z});
    float maxAz = std::max({p1.z, p2.z, p3.z, p4.z});

    float minBx = v.x;
    float maxBx = v.x;
    float minBy = v.y;
    float maxBy = v.y;
    float minBz = v.z;
    float maxBz = v.z;

    if (minAx > maxBx || maxAx < minBx || minAy > maxBy || maxAy < minBy || minAz > maxBz || maxAz < minBz)
        return Point(); // return an invalid point if there is no collision
    else
    {
        // calculate the midpoint of the overlapping region
        float midX = (std::max(minAx, minBx) + std::min(maxAx, maxBx)) / 2;
        float midY = (std::max(minAy, minBy) + std::min(maxAy, maxBy)) / 2;
        float midZ = (std::max(minAz, minBz) + std::min(maxAz, maxBz)) / 2;
        return Point(midX, midY, midZ); // return the midpoint
    }
}

Point Cube_face::collision(Cube_face &cf)
{
    Point p1 = anim.getPos();
    Point p2 = p1, p3, p4 = p1;
    p2.translate(length*vdir1);
    p3 = p2;
    p3.translate(width*vdir2);
    p4.translate(width*vdir2);


    Point p5 = cf.anim.getPos();
    Point p6 = p5, p7, p8 = p5;
    p6.translate(cf.length*cf.vdir1);
    p7 = p6;
    p7.translate(cf.width*cf.vdir2);
    p8.translate(cf.width*cf.vdir2);

    float minAx = std::min({p1.x, p2.x, p3.x, p4.x});
    float maxAx = std::max({p1.x, p2.x, p3.x, p4.x});
    float minAy = std::min({p1.y, p2.y, p3.y, p4.y});
    float maxAy = std::max({p1.y, p2.y, p3.y, p4.y});
    float minAz = std::min({p1.z, p2.z, p3.z, p4.z});
    float maxAz = std::max({p1.z, p2.z, p3.z, p4.z});

    float minBx = std::min({p5.x, p6.x, p7.x, p8.x});
    float maxBx = std::max({p5.x, p6.x, p7.x, p8.x});
    float minBy = std::min({p5.y, p6.y, p7.y, p8.y});
    float maxBy = std::max({p5.y, p6.y, p7.y, p8.y});
    float minBz = std::min({p5.z, p6.z, p7.z, p8.z});
    float maxBz = std::max({p5.z, p6.z, p7.z, p8.z});

    if (minAx > maxBx || maxAx < minBx || minAy > maxBy || maxAy < minBy || minAz > maxBz || maxAz < minBz)
        return Point(); // return an invalid point if there is no collision
    else
    {
        // calculate the midpoint of the overlapping region
        float midX = (std::max(minAx, minBx) + std::min(maxAx, maxBx)) / 2;
        float midY = (std::max(minAy, minBy) + std::min(maxAy, maxBy)) / 2;
        float midZ = (std::max(minAz, minBz) + std::min(maxAz, maxBz)) / 2;
        return Point(midX, midY, midZ); // return the midpoint
    }
}
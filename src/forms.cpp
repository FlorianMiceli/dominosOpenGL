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
    glPushMatrix();
    glRotated(anim.getTheta(), 0.0, 0.0, -1.0);
    glPushMatrix();
    glRotated(anim.getTheta(), 0.0, 0.0, -1.0);
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

    // Mise en route de la texture associee
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    gluQuadricTexture(quad,texture_id);
    gluQuadricNormals(quad,GLU_SMOOTH);

    gluSphere(quad, radius, 20, 20);
    gluDeleteQuadric(quad);

    // Desactivation de la texture
    glDisable(GL_TEXTURE_2D);
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

    // Autorisation de la texture choisie a la creation de la face (cf main())
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    Form::render();

    glBegin(GL_QUADS);
    {
        glTexCoord3f(p1.x, p1.y, p1.z);
        glVertex3d(p1.x, p1.y, p1.z);
        glTexCoord3f(p2.x, p2.y, p2.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glTexCoord3f(p3.x, p3.y, p3.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glTexCoord3f(p4.x, p4.y, p4.z);
        glVertex3d(p4.x, p4.y, p4.z);
    }
    glEnd();

    // Ne plus appliquer la texture pour la suite
    glDisable(GL_TEXTURE_2D);
}

Cuboid::Cuboid(Vector v1, Vector v2, Vector v3, Point org, double l, double w, double h,double mass, Color cl)
Cuboid::Cuboid(Vector v1, Vector v2, Vector v3, Point org, double l, double w, double h,double mass, Color cl)
{
    vdir1 = 1.0 / v1.norm() * v1;
    vdir2 = 1.0 / v2.norm() * v2;
    vdir3 = 1.0 / v3.norm() * v3;
    anim.setPos(org);
    length = l;
    width = w;
    height = h;
    col = cl;
    m = mass;
    anim.setmass(m);
    m = mass;
    anim.setmass(m);
}

void Cuboid::update(double delta_t)
{
    // if (anim.getPos().y <= 0.5)
    // {
    //     // Collision response
    //     // std::cout << anim.getTheta() << std::endl;
    //     CollisionResponse2(anim, *this, anim.getPos(), delta_t);
    // }
    // gravity(delta_t, anim);
    // solid(anim);
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
    if(anim.getTheta() <= 5.0)
    {
        anim.setTheta(10.0);
    }
    if(anim.getTheta() <= 5.0)
    {
        anim.setTheta(10.0);
    }

    Form::render();

    glBegin(GL_QUADS);
    {
        // Right face
        glTexCoord3f(p1.x, p1.y, p1.z);
        glVertex3d(p1.x, p1.y, p1.z);
        glTexCoord3f(p2.x, p2.y, p2.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glTexCoord3f(p3.x, p3.y, p3.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glTexCoord3f(p4.x, p4.y, p4.z);
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
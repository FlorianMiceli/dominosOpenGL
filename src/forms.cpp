#include <cmath>
#include <SDL2/SDL_opengl.h>
#include <GL/GLU.h>
#include "forms.h"


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
        anim.setSpeed(Vector(anim.getSpeed().x, -anim.getSpeed().y*0.3, anim.getSpeed().z));
        anim.setPos(Point(anim.getPos().x, -anim.getPos().y*0.3, anim.getPos().z));

    }
}

void Form::update(double delta_t)
{
    // Nothing to do here, animation update is done in child class method
}


void Form::render()
{
    // Point of view for rendering
    // Common for all Forms
    Point org = anim.getPos();
    glRotated(anim.getPhi(), 0, 1, 0);
    glTranslated(org.x, org.y, org.z);
    glColor3f(col.r, col.g, col.b);
}

Segment::Segment(Point pt1, Point pt2, Color cl)
{
    p1 = pt1;
    p2 = pt2;
    direction = Vector(p1, p2);
    col = cl;
}

// Segment::Segment(Point pt1, Vector v, Color cl)
// {
//     p1 = pt1;
//     direction = v;
//     p2 = Point(p1.x + v.x, p1.y + v.y, p1.z + v.z);
//     col = cl;
// }

void Segment::render()
{
    Form::render();

    glBegin(GL_LINES);
    {
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
    }
    glEnd();
}

void Segment::update(double delta_t)
{
    anim.setPhi(anim.getPhi() + 1);
}

Sphere::Sphere(double r, Color cl)
{
    radius = r;
    col = cl;
}


void Sphere::update(double delta_t)
{
    // Complete this part
}


void Sphere::render()
{
    GLUquadric *quad;

    quad = gluNewQuadric();

    // Complete this part

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
    anim.setPhi(anim.getPhi() + 1);
    gravity(delta_t, anim);
    solid(anim);
}


void Cube_face::render()
{
    Point p1 = Point();
    Point p2 = p1, p3, p4 = p1;
    p2.translate(length*vdir1);
    p3 = p2;
    p3.translate(width*vdir2);
    p4.translate(width*vdir2);

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

Point Cube_face::getCenter()
{
    return Point(anim.getPos().x + 0.5 * length * vdir1.x + 0.5 * width * vdir2.x,
                 anim.getPos().y + 0.5 * length * vdir1.y + 0.5 * width * vdir2.y,
                 anim.getPos().z + 0.5 * length * vdir1.z + 0.5 * width * vdir2.z);
}

Point Cube_face::checkForCollision(Segment s)
{
    // Collision with a segment
    
    // get segment endpoints
    Point p1 = s.getP1();
    Point p2 = s.getP2();

    // get cube face information
    Point org = anim.getPos();
    Point face_center = this->getCenter();
    Vector v1 = vdir1;
    Vector v2 = vdir2;
    double l = length;
    double w = width;
    
    // calculate plane equation of the face
    Vector n = v1.cross(v2);
    //If n=(a,b,c), the plane equation is ax+by+cz+d=0ax+by+cz+d=0.
    double d = -n.x * face_center.x - n.y * face_center.y - n.z * face_center.z;
    //Thus, the plane equation becomes:ax+by+cz+d=0

    // Check if the Segment Intersects the Plane:
    /*Evaluate the plane equation at the segment endpoints:
    F(P1​)=a⋅x1​+b⋅y1​+c⋅z1​+d
    F(P2​)=a⋅x2​+b⋅y2​+c⋅z2​+d*/
    std::cout << "n = " << n << ", d = " << d << std::endl;
    double f_p1 = n.x * p1.x + n.y * p1.y + n.z * p1.z + d;
    double f_p2 = n.x * p2.x + n.y * p2.y + n.z * p2.z + d;

    // If F(P1)F(P1​) and F(P2)F(P2​) have opposite signs OR if one of them = 0, the segment intersects the plane. Otherwise, it does not.
    if (f_p1 * f_p2 >= 0)
    {
        if (f_p1 == 0)
        {
            return p1;
        }
        else if (f_p2 == 0)
        {
            return p2;
        }
        else
        {
            return Point();
        }
    }

    /*Find the Intersection Point:
    If there is an intersection, find the parameter tt where the segment intersects the plane:
    t=−F(P1)F(P2)−F(P1)*/
    
    double t = -f_p1 / (f_p2 - f_p1);

    /*Compute the intersection point P(t)P(t):
    P(t)=P1​+t(P2​−P1​)*/
    Point intersection = Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y), p1.z + t * (p2.z - p1.z));

    /*Check if the Intersection Point Lies Within the Cube Face:

    Express the intersection point in terms of the face’s local coordinate system:
        Local coordinates relative to the center CC:
        Plocal=P(t)−C=(xlocal,ylocal,zlocal)*/
    Point p_local = Point(intersection.x - face_center.x, intersection.y - face_center.y, intersection.z - face_center.z);

    //Project onto the direction vectors:
        //u=Plocal​⋅vdir1
        //v=Plocal⋅vdir2
    double u = p_local.x * v1.x + p_local.y * v1.y + p_local.z * v1.z;
    double v = p_local.x * v2.x + p_local.y * v2.y + p_local.z * v2.z;

    std::cout << "intersection point: " << intersection << std::endl;

    //Check if uu and vv lie within half the length and width:
    //−2L ​≤ u ≤ 2L​ and −2W ​≤ v ≤ 2W​ */
    if (u >= -0.5 * l && u <= 0.5 * l && v >= -0.5 * w && v <= 0.5 * w)
    {
        // intersection point lies within the face
        return intersection;
    }
    else
    {
        // intersection point lies outside the face
        return Point();
    }
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
    anim.setPhi(anim.getPhi() + 0);
    anim.setTheta(anim.getTheta() + 0);
    gravity(delta_t, anim);
    solid(anim);
}

void Cuboid::render()
{
    Point p1 = Point();
    Point p2 = p1, p3, p4 = p1;
    Point p5 = p1, p6 = p1, p7 = p1, p8 = p1;
    p2.translate(length*vdir1);
    p3 = p2;
    p3.translate(width*vdir2);
    p4.translate(width*vdir2);
    p5.translate(height*vdir3);
    p6 = p5;
    p6.translate(length*vdir1);
    p7 = p6;
    p7.translate(width*vdir2);
    p8 = p5;
    p8.translate(width*vdir2);

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

Point Cuboid::checkForCollision(Cuboid c)
{
    // Collision with another cuboid

}


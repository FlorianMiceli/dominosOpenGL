#include <cmath>
#include <SDL2/SDL_opengl.h>
#include <GL/GLU.h>
#include "forms.h"
#include <vector>
#include <algorithm>


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
    glRotated(anim.getTheta(), 1, 0, 0);
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
    anim.setPhi(anim.getPhi() + 0);
    // gravity(delta_t, anim);
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

Vector Cube_face::getNormal()
{
    return Vector();
}

Point Cube_face::getCenter()
{
    return Point(anim.getPos().x + 0.5 * length * vdir1.x + 0.5 * width * vdir2.x,
                 anim.getPos().y + 0.5 * length * vdir1.y + 0.5 * width * vdir2.y,
                 anim.getPos().z + 0.5 * length * vdir1.z + 0.5 * width * vdir2.z);
}

Point Cube_face::checkForCollision(Segment s)
{
    // Get segment endpoints
    Point p1 = s.getP1();
    Point p2 = s.getP2();

    // Get cube face information
    Point face_center = this->getCenter();
    Vector v1 = vdir1;
    Vector v2 = vdir2;
    double l = length;
    double w = width;
    
    // Calculate plane normal
    Vector n = v1.cross(v2);
    double d = -n.x * face_center.x - n.y * face_center.y - n.z * face_center.z;

    // Evaluate plane equation at segment endpoints
    double f_p1 = n.x * p1.x + n.y * p1.y + n.z * p1.z + d;
    double f_p2 = n.x * p2.x + n.y * p2.y + n.z * p2.z + d;


    // Check if the segment intersects the plane
    if (f_p1 * f_p2 > 0)
    {
        // Segment does not intersect the plane
        return Point();  // No collision
    }

    // Calculate intersection point
    double t = -f_p1 / (f_p2 - f_p1);
    Point intersection(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y), p1.z + t * (p2.z - p1.z));

    // Express the intersection point in local coordinates
    Point p_local(intersection.x - face_center.x, intersection.y - face_center.y, intersection.z - face_center.z);

    // Project onto direction vectors
    double u = p_local.x * v1.x + p_local.y * v1.y + p_local.z * v1.z;
    double v = p_local.x * v2.x + p_local.y * v2.y + p_local.z * v2.z;


    // Check if the intersection point lies within the face
    if (u >= -0.5 * l && u <= 0.5 * l && v >= -0.5 * w && v <= 0.5 * w)
    {
        return intersection;  // Intersection point lies within the face
    }
    else
    {
        return Point();  // Intersection point lies outside the face
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
    Point p1 = this->getPosition();
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

Point Cuboid::checkForCollision(Cuboid c, int& ri, int& rj)
{
    for (int i = 0; i < 12; i++)
    {
        Segment s = this->getSegment(i);
        for (int j = 0; j < 6; j++)
        {
            Cube_face f = c.getFace(j);
            Point intersection = f.checkForCollision(s);
            if (intersection.x != 0 || intersection.y != 0 || intersection.z != 0)
            {
                ri = i;
                rj = j;
                return intersection;
            }
        }
    }
    return Point();
}

Cube_face Cuboid::getFace(int i)
{
    // Return the ith face of the cuboid
    // 0: right face, 1: left face, 2: back face, 3: front face, 4: bottom face, 5: top face
    Point org = anim.getPos();
    switch (i)
    {
    case 0:
        return Cube_face(vdir1, vdir2, Point(org.x, org.y, org.z), length, width, col); //right face
    case 1:
        return Cube_face(vdir1, vdir2, Point(org.x, org.y, org.z+height), length, width, col); //left face
    case 2:
        return Cube_face(vdir2, vdir3, Point(org.x, org.y, org.z), width, height, col); //back face
    case 3:
        return Cube_face(vdir2, vdir3, Point(org.x+length, org.y, org.z), width, height, col); //front face
    case 4:
        return Cube_face(vdir1, vdir3, Point(org.x, org.y, org.z), length, height, col); //bottom face
    case 5:
        return Cube_face(vdir1, vdir3, Point(org.x, org.y+width, org.z), length, height, col); //top face
    default:
        break;
    }
}

Segment Cuboid::getSegment(int i)
{
    // Return the ith segment of the cuboid
    //show vdir1, vdir2, vdir3, length, width, height
    switch (i)
    {
    
    case 0:
        return Segment(anim.getPos(), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z), col); // bottom right 
    case 1:
        return Segment(anim.getPos(), Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z), col); // back right
    case 2:
        return Segment(Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z), col); // top right
    case 3:
        return Segment(Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z), col); // front right
    case 4:
        return Segment(Point(anim.getPos()), Point(anim.getPos().x, anim.getPos().y, anim.getPos().z + height), col); // bottom back
    case 5:
        return Segment(Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z), Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), col); // back top
    case 6:
        return Segment(Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), col); // top front
    case 7:
        return Segment(Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), col); // top left
    case 8:
        return Segment(Point(anim.getPos().x, anim.getPos().y, anim.getPos().z + height), Point(anim.getPos().x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), col); // back left
    case 9:
        return Segment(Point(anim.getPos().x + length * vdir1.x, anim.getPos().y + width * vdir2.y, anim.getPos().z + height), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z + height), col); // front left
    case 10:
        return Segment(Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z + height), col); // bottom front
    case 11:
        return Segment(Point(anim.getPos().x, anim.getPos().y, anim.getPos().z + height), Point(anim.getPos().x + length * vdir1.x, anim.getPos().y, anim.getPos().z + height), col); // bottom left
    
    default:
        break;
    }
}

// double Domino::getSpeedAbs()
// {
//     return sqrt(pow(Speed.x, 2) + pow(Speed.y, 2) + pow(Speed.z, 2));
// }

void Domino::update(double delta_t)
{
    // Update Positions and Orientations


    // Get the current state of the domino
    Point r = this->getPosition();
    Vector v = this->getVelocity();
    Vector omega = this->getAngularVelocity();
    double theta = this->getTheta();
    double phi = this->getPhi();
    double l = this->getLength();
    double w = this->getWidth();
    double h = this->getHeight();
    double mu = this->getFrictionCoefficient();

    // Update velocity
    // Vector v1 = Vector(v.x + delta_t * anim.getAccel().x, v.y + delta_t * anim.getAccel().y, v.z + delta_t * anim.getAccel().z);
    //arbirtary velocity for now
    Vector v1 = Vector(1, 0, 0);
    this->setVelocity(v1);


    // Update angular velocity
    Vector omega1 = Vector(omega.x, omega.y, omega.z);
    this->setAngularVelocity(omega1);

    // Update theta
    double theta1 = theta + delta_t * omega1.x;
    this->setTheta(theta1);

    // Update phi
    double phi1 = phi + delta_t * omega1.y;
    this->setPhi(phi1);

    // Update the acceleration
    Vector a1 = Vector(-0.5 * anim.getSpeed().x, -9.81, -0.5 * anim.getSpeed().z);
    anim.setAccel(a1);

    // Update the position based on speed and angular velocity
    Point r1 = Point(r.x + delta_t * v1.x + 0.5 * delta_t * delta_t * anim.getAccel().x,
                     r.y + delta_t * v1.y + 0.5 * delta_t * delta_t * anim.getAccel().y,
                     r.z + delta_t * v1.z + 0.5 * delta_t * delta_t * anim.getAccel().z);

    std::cout << "Position: " << r1.x << " " << r1.y << " " << r1.z << std::endl;

    this->setPosition(r1);
    
    
    // Check for collisions
    int ri = -1;
    int rj = -1;

    // Get all dominoes (empty for now)
    std::vector<Domino> allDominoes;

    for (Domino& otherDomino : allDominoes)
    {
        if (&otherDomino == this) continue; // Skip self

        Point collisionPoint = this->checkForCollision(otherDomino, ri, rj);
        // get the face of the other domino that was hit
        Cube_face face = otherDomino.getFace(rj);
        if (collisionPoint.x != 0 || collisionPoint.y != 0 || collisionPoint.z != 0)
        {
            // Handle the collision
            this->handleCollision(otherDomino, collisionPoint, face.getNormal());
        }
    }
}

void Domino::render()
{
    std::cout << "Rendering domino" << std::endl;
    //use the cuboid render method
    Cuboid::render();
}



Point Domino::checkForCollision(Domino d, int &ri, int &rj)
{
    // Check for collision with another domino
    return Cuboid::checkForCollision(d, ri, rj);
}


void Domino::handleCollision(Domino &d, const Point &collisionPoint, const Vector &collisionNormal)
{
    // get information
    // Domino 1
    double m1 = this->getMass();
    Vector v1 = this->getVelocity();
    Vector omega1 = this->getAngularVelocity();
    Matrix3x3 I1 = this->getMomentOfInertia();
    Point r1 = this->getPosition();
    double theta1 = this->getTheta();
    double phi1 = this->getPhi();
    double l1 = this->getLength();
    double w1 = this->getWidth();
    double h1 = this->getHeight();
    double mu1 = this->getFrictionCoefficient();

    // Domino 2
    double m2 = d.getMass();
    Vector v2 = d.getVelocity();
    Vector omega2 = d.getAngularVelocity();
    Matrix3x3 I2 = d.getMomentOfInertia();
    Point r2 = d.getPosition();
    double theta2 = d.getTheta();
    double phi2 = d.getPhi();
    double l2 = d.getLength();
    double w2 = d.getWidth();
    double h2 = d.getHeight();
    double mu2 = d.getFrictionCoefficient();

    //collision point
    Point rc = collisionPoint;

    // Calculate relative velocity at the point of collision
    Vector rc_minus_r1 = Vector(r1, rc);
    Vector rc_minus_r2 = Vector(r2, rc);
    Vector v_rel = (v1 + omega1.cross(rc_minus_r1)) - (v2 + omega2.cross(rc_minus_r2));

    // Assuming the normal vector 'n' is given
    Vector n = collisionNormal;

    // Calculate the normal component of the relative velocity
    Vector v_rel_n = n * (v_rel.dot(n));

    // Calculate the tangential component of the relative velocity
    Vector v_rel_t = v_rel - v_rel_n;

    // Coefficient of restitution
    double e = 0.95; 

    // Calculate the inverse of the moment of inertia for both dominos
    Matrix3x3 I1_inv = I1.inverse();
    Matrix3x3 I2_inv = I2.inverse();

    // Calculate the cross product of (rc - r1) and n, and (rc - r2) and n
    Vector cross1 = (rc_minus_r1.cross(n));
    Vector cross2 = (rc_minus_r2.cross(n));

    // Calculate the dot product of n with the result of the cross product and inverse moment of inertia multiplication
    double dot1 = n.dot(I1_inv * cross1);
    double dot2 = n.dot(I2_inv * cross2);

    // Calculate the normal impulse Jn
    double Jn = - (1 + e) * v_rel_n.dot(n) * (1/m1 + 1/m2 + dot1 + dot2);

    // Calculate the combined friction coefficient
    double mu = (mu1 + mu2) / 2;

    // Calculate the magnitude of the normal impulse
    double Jn_magnitude = std::abs(Jn);

    // Calculate the magnitude of the tangential component of the relative velocity
    double v_rel_t_magnitude = v_rel_t.norm();

    // Calculate the tangential impulse Jt
    Vector Jt = - std::min(mu * Jn_magnitude, v_rel_t_magnitude) * (v_rel_t / v_rel_t_magnitude);

    // Update the linear velocities
    v1 = v1 + Jn / m1;
    v2 = v2 - Jn / m2;

    // Update the angular velocities
    omega1 = omega1 + I1_inv * (rc_minus_r1.cross(Jn));
    omega2 = omega2 - I2_inv * (rc_minus_r2.cross(Jn)); 

    
    

}


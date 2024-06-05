// Using SDL, SDL OpenGL and standard IO
#include <iostream>
#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <GL/GLU.h>

// Module for space geometry
#include "geometry.h"
// Module for generating and rendering forms
#include "forms.h"


/***************************************************************************/
/* Constants and functions declarations                                    */
/***************************************************************************/
// Screen dimension constants
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

// Max number of forms : static allocation
const int MAX_FORMS_NUMBER = 100;

// Animation actualization delay (in ms) => 100 updates per second
const Uint32 ANIM_DELAY = 10;


// Starts up SDL, creates window, and initializes OpenGL
bool init(SDL_Window** window, SDL_GLContext* context);

// Initializes matrices and clear color
bool initGL();

// Updating forms for animation
void update(Form* formlist[MAX_FORMS_NUMBER], double delta_t);

// Renders scene to the screen
void render(Form* formlist[MAX_FORMS_NUMBER], const Point &cam_pos);

// Frees media and shuts down SDL
void close(SDL_Window** window);


/***************************************************************************/
/* Functions implementations                                               */
/***************************************************************************/
bool init(SDL_Window** window, SDL_GLContext* context)
{
    // Initialization flag
    bool success = true;

    // Initialize SDL
    if(SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        success = false;
    }
    else
    {
        // Use OpenGL 2.1
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

        // Create window
        *window = SDL_CreateWindow( "TP intro OpenGL / SDL 2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN );
        if( *window == NULL )
        {
            std::cerr << "Window could not be created! SDL Error: " << SDL_GetError() << std::endl;
            success = false;
        }
        else
        {
            // Create context
            *context = SDL_GL_CreateContext(*window);
            if( *context == NULL )
            {
                std::cerr << "OpenGL context could not be created! SDL Error: " << SDL_GetError() << std::endl;
                success = false;
            }
            else
            {
                // Use Vsync
                if(SDL_GL_SetSwapInterval(1) < 0)
                {
                    std::cerr << "Warning: Unable to set VSync! SDL Error: " << SDL_GetError() << std::endl;
                }

                // Initialize OpenGL
                if( !initGL() )
                {
                    std::cerr << "Unable to initialize OpenGL!"  << std::endl;
                    success = false;
                }
            }
        }
    }

    return success;
}


bool initGL()
{
    bool success = true;
    GLenum error = GL_NO_ERROR;

    // Initialize Projection Matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set the viewport : use all the window to display the rendered scene
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Fix aspect ratio and depth clipping planes
    gluPerspective(40.0, (GLdouble)SCREEN_WIDTH/SCREEN_HEIGHT, 1.0, 100.0);


    // Initialize Modelview Matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Initialize clear color : black with no transparency
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f );

    // Activate Z-Buffer
    glEnable(GL_DEPTH_TEST);


    // Lighting basic configuration and activation
    const GLfloat light_ambient[]  = { 0.3f, 0.3f, 0.3f, 1.0f };
    const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_position[] = { 2.0f, 5.0f, 5.0f, 0.0f };

    const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0f };
    const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
    const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat high_shininess[] = { 100.0f };

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);


    // Check for error
    error = glGetError();
    if( error != GL_NO_ERROR )
    {
        std::cerr << "Error initializing OpenGL!  " << gluErrorString( error ) << std::endl;
        success = false;
    }

    return success;
}

void update(Form* formlist[MAX_FORMS_NUMBER], double delta_t)
{
    // Update the list of forms
    unsigned short i = 0;
    while(formlist[i] != NULL)
    {
        formlist[i]->update(delta_t);
        i++;
    }
}

void render(Form* formlist[MAX_FORMS_NUMBER], const Point &cam_pos)
{
    // Clear color buffer and Z-Buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Initialize Modelview Matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    // Set the camera position and parameters
    gluLookAt(cam_pos.x,cam_pos.y,cam_pos.z, 0.0,0.0,0.0, 0.0,1.0,0.0);
    // Isometric view
    glRotated(-45, 0, 1, 0);
    glRotated(30, 1, 0, -1);

    // X, Y and Z axis
    glPushMatrix(); // Preserve the camera viewing point for further forms
    // Render the coordinates system
    glBegin(GL_LINES);
    {
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3i(0, 0, 0);
        glVertex3i(1, 0, 0);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3i(0, 0, 0);
        glVertex3i(0, 1, 0);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3i(0, 0, 0);
        glVertex3i(0, 0, 1);
    }
    glEnd();
    glPopMatrix(); // Restore the camera viewing point for next object

    // Render the list of forms
    unsigned short i = 0;
    while(formlist[i] != NULL)
    {
        glPushMatrix(); // Preserve the camera viewing point for further forms
        formlist[i]->render();
        glPopMatrix(); // Restore the camera viewing point for next object
        i++;
    }
}

void close(SDL_Window** window)
{
    //Destroy window
    SDL_DestroyWindow(*window);
    *window = NULL;

    //Quit SDL subsystems
    SDL_Quit();
}


/***************************************************************************/
/* MAIN Function                                                           */
/***************************************************************************/
int main(int argc, char* args[])
{
    // The window we'll be rendering to
    SDL_Window* gWindow = NULL;

    // OpenGL context
    SDL_GLContext gContext;


    // Start up SDL and create window
    if( !init(&gWindow, &gContext))
    {
        std::cerr << "Failed to initialize!" << std::endl;
    }
    else
    {
        // Main loop flag
        bool quit = false;
        Uint32 current_time, previous_time, elapsed_time;

        // Event handler
        SDL_Event event;

        // Camera position
        Point camera_position(0, 0.0, 5.0);

        // The forms to render
        Form* forms_list[MAX_FORMS_NUMBER];
        unsigned short number_of_forms = 0, i;
        for (i=0; i<MAX_FORMS_NUMBER; i++)
        {
            forms_list[i] = NULL;
        }


        Cube_face *pFace = NULL;
        pFace = new Cube_face(Vector(1,0,0), Vector(0,1,0), Point(0, 0, 0), 1, 1, RED);
        forms_list[number_of_forms] = pFace;

        Cuboid *pCuboid = NULL;
        pCuboid = new Cuboid(Vector(1,0,0), Vector(0,1,0), Vector(0,0,1), Point(0.5, 0.5, 0.5), 1, 1, 1, WHITE);
        // forms_list[++number_of_forms] = pCuboid;

        Segment *pSegment = NULL;
        pSegment = new Segment(Point(0, 0, 0), Point(1, 1, 1), BLUE);
        forms_list[++number_of_forms] = pSegment;


        // //test for Cuboid::getFace, afficher pcuboid
        // Cube_face face = pCuboid->getFace(0);
        // forms_list[++number_of_forms] = &face;
        // Cube_face face1 = pCuboid->getFace(1);
        // forms_list[++number_of_forms] = &face1;
        // Cube_face face2 = pCuboid->getFace(2);
        // forms_list[++number_of_forms] = &face2;
        // Cube_face face3 = pCuboid->getFace(3);
        // forms_list[++number_of_forms] = &face3;
        // Cube_face face4 = pCuboid->getFace(4);
        // forms_list[++number_of_forms] = &face4;
        // Cube_face face5 = pCuboid->getFace(5);
        // forms_list[++number_of_forms] = &face5;

        //test for Cuboid::getSegment, afficher pcuboid
        Segment segment = pCuboid->getSegment(0);
        forms_list[++number_of_forms] = &segment;
        Point p1 = segment.getP1();
        Point p2 = segment.getP2();
        std::cout << "p1: " << p1 << std::endl;
        std::cout << "p2: " << p2 << std::endl;
        Segment segment1 = pCuboid->getSegment(1);
        forms_list[++number_of_forms] = &segment1;
        Point p3 = segment1.getP1();
        Point p4 = segment1.getP2();
        std::cout << "p3: " << p3 << std::endl;
        std::cout << "p4: " << p4 << std::endl;
        Segment segment2 = pCuboid->getSegment(2);
        forms_list[++number_of_forms] = &segment2;
        Point p5 = segment2.getP1();
        Point p6 = segment2.getP2();
        std::cout << "p5: " << p5 << std::endl;
        std::cout << "p6: " << p6 << std::endl;
        Segment segment3 = pCuboid->getSegment(3);
        forms_list[++number_of_forms] = &segment3;
        Point p7 = segment3.getP1();
        Point p8 = segment3.getP2();
        std::cout << "p7: " << p7 << std::endl;
        std::cout << "p8: " << p8 << std::endl;
        Segment segment4 = pCuboid->getSegment(4);
        forms_list[++number_of_forms] = &segment4;
        Point p9 = segment4.getP1();
        Point p10 = segment4.getP2();
        std::cout << "p9: " << p9 << std::endl;
        std::cout << "p10: " << p10 << std::endl;
        Segment segment5 = pCuboid->getSegment(5);
        forms_list[++number_of_forms] = &segment5;
        Point p11 = segment5.getP1();
        Point p12 = segment5.getP2();
        std::cout << "p11: " << p11 << std::endl;
        std::cout << "p12: " << p12 << std::endl;
        Segment segment6 = pCuboid->getSegment(6);
        forms_list[++number_of_forms] = &segment6;
        Point p13 = segment6.getP1();
        Point p14 = segment6.getP2();
        std::cout << "p13: " << p13 << std::endl;
        std::cout << "p14: " << p14 << std::endl;
        Segment segment7 = pCuboid->getSegment(7);
        forms_list[++number_of_forms] = &segment7;
        Point p15 = segment7.getP1();
        Point p16 = segment7.getP2();
        std::cout << "p15: " << p15 << std::endl;
        std::cout << "p16: " << p16 << std::endl;
        Segment segment8 = pCuboid->getSegment(8);
        forms_list[++number_of_forms] = &segment8;
        Point p17 = segment8.getP1();
        Point p18 = segment8.getP2();
        std::cout << "p17: " << p17 << std::endl;
        std::cout << "p18: " << p18 << std::endl;
        Segment segment9 = pCuboid->getSegment(9);
        forms_list[++number_of_forms] = &segment9;
        Point p19 = segment9.getP1();
        Point p20 = segment9.getP2();
        std::cout << "p19: " << p19 << std::endl;
        std::cout << "p20: " << p20 << std::endl;
        Segment segment10 = pCuboid->getSegment(10);
        forms_list[++number_of_forms] = &segment10;
        Point p21 = segment10.getP1();
        Point p22 = segment10.getP2();
        std::cout << "p21: " << p21 << std::endl;
        std::cout << "p22: " << p22 << std::endl;
        Segment segment11 = pCuboid->getSegment(11);
        forms_list[++number_of_forms] = &segment11;
        Point p23 = segment11.getP1();
        Point p24 = segment11.getP2();
        std::cout << "p23: " << p23 << std::endl;
        std::cout << "p24: " << p24 << std::endl;




        for (i=0; i<MAX_FORMS_NUMBER; i++)
        {
            if (forms_list[i] != NULL)
            {
                number_of_forms++;
            }
        }

        // Get first "current time"
        previous_time = SDL_GetTicks();
        // While application is running
        while(!quit)
        {
            // Handle events on queue
            while(SDL_PollEvent(&event) != 0)
            {
                int x = 0, y = 0;
                SDL_Keycode key_pressed = event.key.keysym.sym;

                switch(event.type)
                {
                // User requests quit
                case SDL_QUIT:
                    quit = true;
                    break;
                case SDL_KEYDOWN:
                    // Handle key pressed with current mouse position
                    SDL_GetMouseState( &x, &y );

                    switch(key_pressed)
                    {
                    // Quit the program when 'q' or Escape keys are pressed
                    case SDLK_q:
                    case SDLK_ESCAPE:
                        quit = true;
                        break;

                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
            }

            // Update the scene
            current_time = SDL_GetTicks(); // get the elapsed time from SDL initialization (ms)
            elapsed_time = current_time - previous_time;
            if (elapsed_time > ANIM_DELAY)
            {
                previous_time = current_time;
                update(forms_list, 1e-3 * elapsed_time); // International system units : seconds
            }

            // Render the scene
            render(forms_list, camera_position);

            // Update window screen
            SDL_GL_SwapWindow(gWindow);
        }
    }

    // Free resources and close SDL
    close(&gWindow);

    return 0;
}

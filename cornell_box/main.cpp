
#include "monteCarloPathTracing.h"

extern void PathTracing(double *eye, GLfloat *image);

GLint window_width = 512, window_height = 512;


void display()
{
    double eye[3] = { 256.0, 256.0, 1228.0 };
    
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static GLfloat *image, *buffer;
    static int k = 0;

    if(k == 0){
        // initialize rendering target
        image = new GLfloat [window_width*window_height*3];
        for(int i = 0; i < window_width*window_height*3; i++) image[i] = 0.0;

        buffer = new GLfloat [window_width*window_height*3];
    }

    srand(k++);

    PathTracing(eye, image);

    cerr << "Samples: " << k << endl;

    double inv_k = 1.0 / (double)(k);
    for(int i = 0; i < window_width*window_height*3; i++) buffer[i] = image[i] * inv_k;

    glDrawPixels(window_width, window_height, GL_RGB, GL_FLOAT, buffer);

    glutSwapBuffers();
 
    if(k < N_SAMPLES) glutPostRedisplay();
}



int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);

    glutInitWindowPosition(300, 20);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow(argv[0]);

    glutDisplayFunc(display);
    glutMainLoop();
    return 1;
}


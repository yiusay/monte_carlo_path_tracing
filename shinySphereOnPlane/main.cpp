
#include "monteCarloPathTracing.h"

#include <windows.h>
#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")


extern void PathTracing(double *eye, GLfloat *image);

GLint window_width = 512, window_height = 512;

double *png_data;
int     png_width, png_height;

double *table_texture;
int     table_texture_width, table_texture_height;

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

double* LoadPngData(WCHAR *filename, int &img_width, int &img_height)
{
    Bitmap bitmap(filename);

    img_width  = bitmap.GetWidth();
    img_height = bitmap.GetHeight();

    cerr << img_width << " " << img_height << endl;

    double *pixel_data = new double [img_width*img_height*3];

    for(int i = 0; i < img_height; i++){
        for(int j = 0; j < img_width; j++){
            Color pixelColor;
            bitmap.GetPixel(j, i, &pixelColor);

            static double scale = 1.0/255.0;
            pixel_data[((img_height-1-i)*img_width+j)*3+0] = pixelColor.GetR()*scale;
            pixel_data[((img_height-1-i)*img_width+j)*3+1] = pixelColor.GetG()*scale;
            pixel_data[((img_height-1-i)*img_width+j)*3+2] = pixelColor.GetB()*scale;
        }
    }

    return pixel_data;
}

int main(int argc, char *argv[])
{
    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR           gdiplusToken;

    // Initialize GDI+.
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    png_data      = LoadPngData(L"panorama.png", png_width, png_height);
    table_texture = LoadPngData(L"chiyo.png", table_texture_width, table_texture_height);

    GdiplusShutdown(gdiplusToken);


    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowPosition(300, 20);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutMainLoop();

    return 1;
}


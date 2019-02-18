/*
Image Plot API for QCL Chemical Holographic Imaging System
last edit: 02.08.2018
copyright: Shihao Ran
STIM LAB
*/


#include <GL/freeglut.h>
#include <GL/glut.h>		//GLUT hearder file for window creating and image rendering
#include <iostream>			//windows standard input/output header
#include <vector>			//vector reading and processing header
#include <cstdlib>			//std hearder
#include <fstream>			//file operation header
#include <stdint.h>	


// specify input image location
const char* input_path = "C://Users//shihao//Desktop//ir-images//ir-holography//render-test-1//0//sbf161_img_00000_1600.txt";

//declear GLUT functions
void render(void);
void keyboard(char c, int x, int y);
void mouse(int button, int state, int x, int y);

//for each pixel on the image, creat a Point structure
struct Point{
	float x, y;				//x and y coordinates of the pixel
	float r, g, b, a;		//RGBA values of the pixel, A value is for shadering, no real impact for 2D rendering
};

//a vector of Point structures for the whole image
std::vector < Point > points;

//reshape function for reshaping the image window
void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
}

//render function for ploting the image
void render(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				//clear the current GL buffers for ploting new image
	glMatrixMode(GL_PROJECTION);									//applies subsequent matrix operations to the prejection matrix stack so we can set the parameters of the camera to view objects
	glLoadIdentity();												//replace the current matrix with identical matrix, basically put us at (0, 0, 0)
	gluOrtho2D(0, 128, 0, 128);										//define a 2D orthographic projection matrix, here basiccaly set the coordinate system, tell GLUT that the bottom left corner of the image should be at (0, 0) and top right corner should be at (128, 128)
	glMatrixMode(GL_MODELVIEW);										//after setting the coordinate system, set the matrix operation mode to defualt, i.e. how your objects are transformed
	glLoadIdentity();												//send us back to origin

	glEnableClientState(GL_VERTEX_ARRAY);							//enbale vertex array for writing during rendering
	glEnableClientState(GL_COLOR_ARRAY);							//enbale color array for writing during rendering, basically tell GLUT that we are using color and vertex arraies for fixed-function attribute
	glPointSize(5);													//specify the diameter of rasterized points, must be called before glDrawArrays
	glVertexPointer(2, GL_FLOAT, sizeof(Point), &points[0].x);		//define the array of vertex data, (number of coordinates per vertex, date type of the coordinates, byte offset between consecutive vertices, the pointer of the first coordinate of the first vertex in the array)
	glColorPointer(4, GL_FLOAT, sizeof(Point), &points[0].r);		//similar to glVertexPointer, corresponding to color information of each vertex 
	glDrawArrays(GL_POINTS, 0, points.size());						//render primitives from array data, (type of promitives to render, the starting index in the enabled arrays, the number of indices to be rendered)
	
	glDisableClientState(GL_VERTEX_ARRAY);							//disable vertex array for writing during rendering
	glDisableClientState(GL_COLOR_ARRAY);							//disable color array for writing during rendering
	glutSwapBuffers();												//swap the front and back buffer to show new rendered image
}

//keyboard function
//if ESC is hit, close the window 
void keyboard(unsigned char c, int x, int y){
	if (c == 27){
		exit(0);
	}
}

//mouse function
//if right-click is clicked, close the window
void mouse(int button, int state, int x, int y){
	if (button == GLUT_RIGHT_BUTTON){
		exit(0);
	}
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);											//initialize the GLUT library
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);		//set the initial display mode: have a depth buffer, a double buffered window and an RGBA mode window
	glutInitWindowPosition(300, 300);								//the initail position of the window on the screen
	glutInitWindowSize(128, 128);									//window size
	glutCreateWindow("Hologram");									//create a window with the specified title

	std::ifstream inImage;					//initialize a input file stream
	uint16_t num;							//set byte offset of the input file stream
	std::vector<uint16_t> Himage;			//input image will be saved to vector Himage
    inImage.open(input_path);				//open the input image for reading

	if (inImage.is_open()){
		while(inImage >> num){
			Himage.push_back(num);			//keep reading in elements in input image
		}
		inImage.close();					//finish reading and close the input file
	}
    
	else std::cout << "Unable to open file" << std::endl;
	for(int i = 0; i<16384; i++){
		
		float Ipixel = Himage[i];			//allocate each element in the input image file to the vertex array
		float Icolor = Ipixel / 16384;		//convert element value into color information
		int Pcolor = Icolor;
		Point pt;
		pt.x = i % 128;
		pt.y = int(i / 128);				//reshape the input vector to 128 * 128 image
		pt.r = Icolor;
		pt.g = Icolor;
		pt.b = Icolor;
		pt.a = 1;
		points.push_back(pt);				//keep allocating points until all 16384 pixels are specified
		
	}

	glutDisplayFunc(render);				//tell GLUT to render all points
	
    glutReshapeFunc(reshape);				//tell GLUT how to reshape window
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMainLoop();							//tell GLUT to keep rendering
}

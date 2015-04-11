// Main.cpp : Defines the entry point for the console application.

/**********************************************************
*		CollisionDetectionColdet Demo
*		author: Tomasz Augustyn
* 
**********************************************************/

#include "../Defs/defs.h"
#include <thread>
#include <vector>
#include <iostream>
#include <ostream>
#include <GL/glut.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "../include/CollisionDetection/CollisionDetectionColdet.h"
#include <chrono>
#include <ctime>

/**********************************************************
 * VARIABLES DECLARATION
 *********************************************************/

// The width and height of your window, change them as you like
int screen_width=1024;
int screen_height=640;

double rotation_x=0;
double rotation_y=0;
double rotation_z=0;
double translation_x=0;
double translation_y=0;
double translation_z=0;
// Flag for rendering as lines or filled polygons
int filling=1; //0=OFF 1=ON


CollisionDetection* robot_structure;
bool czy_jest_kolizja;		///bedzie przyjmowala wartosc zwracana przez metoda CheckCollision - 1 -> sa kolije,  0->brak kolizji
std::vector<coldet::float_type> config(18, -0.90);  ///pozycje serwonapedow nog robota
std::vector<bool> collision_table(19);  ///tablica kolizji, jej elementy przyjmuja wartosc '1' jezeli odpowiadajaca jej czesc robota koliduje. [0] -> corpus, [1-6] -> coxa1-6,  [7-12] -> femur1-6,   [13-18] -> vitulus1-6
short int wybor_nogi=0;  //wybor nogi do sterowania pozycjami jej serownapedow w openGL'u

coldet::Mat34 pose;
std::vector<coldet::float_type> set_pose(6); // x="0" y="0" z="0.0" alfa="0.0" beta="0.0" gamma="0.0"


GLfloat light_position[4] =
{
    0.0, 5.0, 2.0, 0.0
};

GLfloat light_rotatex = -10.0;  //-10
GLfloat light_rotatey = 90.0;    //0, 90

// katy obrotu obiektu przy obsludze myszki
GLfloat rotatex = 0.0;
GLfloat rotatey = 0.0;

// rozmiary bryly obcinania
const GLdouble left1 = -2.0;
const GLdouble right1 = 2.0;
const GLdouble bottom = -2.0;
const GLdouble top = 2.0;
const GLdouble near = 3.0;
const GLdouble far = 7.0;

// wskaznik nacisniecia lewego przycisku myszki
int button_state = GLUT_UP;

// polozenie kursora myszki
int button_x, button_y;

// wspolczynnik skalowania
GLfloat scale = 1.0;

/**********************************************************
 * SUBROUTINE init()
 *
 * Used to initialize OpenGL and to setup our world
 *
 *********************************************************/

void init(void)
{
    glClearColor(0.0, 0.5, 0.2, 0.0); // This clear the background color to black  0.0, 0.5, 0.2, 0.0    1.056, 0.768, 0.312, 0.0,   0.951, 0.12, 0.22, 0.7
    glShadeModel(GL_SMOOTH); // Type of shading for the polygons
   	
    // Viewport transformation
    glViewport(0,0,screen_width,screen_height);  

    // Projection transformation
    glMatrixMode(GL_PROJECTION); // Specifies which matrix stack is the target for matrix operations 
    glLoadIdentity(); // We initialize the projection matrix as identity
    gluPerspective(45.0f,(GLfloat)screen_width/(GLfloat)screen_height,10.0f,10000.0f); // We define the "viewing volume"
   
    glEnable(GL_DEPTH_TEST); // We enable the depth test (also called z buffer)
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); // GL_FILL-> Polygon rasterization mode (polygon filled),  GL_LINE-> (not filled) filling=0;
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DITHER);

	/// Dodanie oswietlenia
	glEnable(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);


	/// Konfiguracja dla serwonapedow ud
	config[1]=0.8;
	config[4]=0.8;
	config[7]=0.8;
	config[10]=0.8;
	config[13]=0.8;
	config[16]=0.8;

	/// Konfiguracja dla serwonapedow lydek
	config[2]=-2.3;
	config[5]=-2.3;
	config[8]=-2.3;
	config[11]=-2.3;
	config[14]=-2.3;
	config[17]=-2.3;

	config[15]=0.7;


	/// Konfiguracja pozycji robota
	set_pose[0]=0.0;				//x
	set_pose[1]=0.0;				//y
	set_pose[2]=0.0;				//z
	set_pose[3]=45;					//alfa 90
	set_pose[4]=0;					//beta 45
	set_pose[5]=0;					//gamma 
	
}

/**********************************************************
 *
 * SUBROUTINE resize(int,int)
 *
 * This routine must be called everytime we resize our window.
 * 
 *********************************************************/

void resize (int width, int height)
{
    screen_width=width;		// We obtain the new screen width values and store it
    screen_height=height;	// Height value

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// We clear both the color and the depth buffer so to draw the next frame
    glViewport(0,0,screen_width,screen_height);				// Viewport transformation

    glMatrixMode(GL_PROJECTION);	// Projection transformation
    glLoadIdentity();				// We initialize the projection matrix as identity

    gluPerspective(45.0f,(GLfloat)screen_width/(GLfloat)screen_height,10.0f,10000.0f);

    glutPostRedisplay (); // This command redraw the scene (it calls the same routine of glutDisplayFunc)
}

/**********************************************************
 *
 * SUBROUTINE keyboard(unsigned char,int,int)
 *
 * Used to handle the keyboard input (ASCII Characters)
 * 
 *********************************************************/

void keyboard (unsigned char key, int x, int y)
{    
    switch (key)
    {
        case 'r': case 'R':
            if (filling==0)
            {
                glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); // Polygon rasterization mode (polygon filled)
                filling=1;
            }   
            else 
            {
                glPolygonMode (GL_FRONT_AND_BACK, GL_LINE); // Polygon rasterization mode (polygon outlined)
                filling=0;
            }
        break;
		case '.':
			translation_z = translation_z + 0.5;
		break;
		case ',':
			translation_z = translation_z - 0.5;
		break;
		case ';':
			translation_x = translation_x + 0.5;
		break;
		case 'k': case 'K':
			translation_x = translation_x - 0.5;
		break;
		case 'o': case 'O':
			translation_y = translation_y + 0.5;
		break;
		case 'l': case 'L':
			translation_y = translation_y - 0.5;
		break;
		case '1':
			wybor_nogi=1;
		break;
		case '2':
			wybor_nogi=2;
		break;
		case '3':
			wybor_nogi=3;
		break;
		case '4':
			wybor_nogi=4;
		break;
		case '5':
			wybor_nogi=5;
		break;
		case '6':
			wybor_nogi=6;
		break;
		case '0':
			wybor_nogi=0;
		break;
		case 'q': case 'Q':
			if(wybor_nogi==1)
				config[2]=config[2]+0.02;
			else if(wybor_nogi==2)
				config[5]=config[5]+0.02;
			else if(wybor_nogi==3)
				config[8]=config[8]+0.02;
			else if(wybor_nogi==4)
				config[11]=config[11]+0.02;
			else if(wybor_nogi==5)
				config[14]=config[14]+0.02;
			else if(wybor_nogi==6)
				config[17]=config[17]+0.02;
		break;

		case 'a': case 'A':
			if(wybor_nogi==1)
				config[2]=config[2]-0.02;
			else if(wybor_nogi==2)
				config[5]=config[5]-0.02;
			else if(wybor_nogi==3)
				config[8]=config[8]-0.02;
			else if(wybor_nogi==4)
				config[11]=config[11]-0.02;
			else if(wybor_nogi==5)
				config[14]=config[14]-0.02;
			else if(wybor_nogi==6)
				config[17]=config[17]-0.02;
		break;

		case 'w': case 'W':
			if(wybor_nogi==1)
				config[1]=config[1]+0.02;
			else if(wybor_nogi==2)
				config[4]=config[4]+0.02;
			else if(wybor_nogi==3)
				config[7]=config[7]+0.02;
			else if(wybor_nogi==4)
				config[10]=config[10]+0.02;
			else if(wybor_nogi==5)
				config[13]=config[13]+0.02;
			else if(wybor_nogi==6)
				config[16]=config[16]+0.02;
		break;

		case 's': case 'S':
			if(wybor_nogi==1)
				config[1]=config[1]-0.02;
			else if(wybor_nogi==2)
				config[4]=config[4]-0.02;
			else if(wybor_nogi==3)
				config[7]=config[7]-0.02;
			else if(wybor_nogi==4)
				config[10]=config[10]-0.02;
			else if(wybor_nogi==5)
				config[13]=config[13]-0.02;
			else if(wybor_nogi==6)
				config[16]=config[16]-0.02;
		break;

		case 'e': case 'E':
			if(wybor_nogi==1)
				config[0]=config[0]+0.02;
			else if(wybor_nogi==2)
				config[3]=config[3]+0.02;
			else if(wybor_nogi==3)
				config[6]=config[6]+0.02;
			else if(wybor_nogi==4)
				config[9]=config[9]+0.02;
			else if(wybor_nogi==5)
				config[12]=config[12]+0.02;
			else if(wybor_nogi==6)
				config[15]=config[15]+0.02;
		break;

		case 'd': case 'D':
			if(wybor_nogi==1)
				config[0]=config[0]-0.02;
			else if(wybor_nogi==2)
				config[3]=config[3]-0.02;
			else if(wybor_nogi==3)
				config[6]=config[6]-0.02;
			else if(wybor_nogi==4)
				config[9]=config[9]-0.02;
			else if(wybor_nogi==5)
				config[12]=config[12]-0.02;
			else if(wybor_nogi==6)
				config[15]=config[15]-0.02;
		break;

		/// wyswietlenie tabeli kolizji
		case 'z': case 'Z':
			std::cout<<czy_jest_kolizja<<"\n";
			for (int i=0; i<19; i++)
				std::cout <<collision_table[i];
			std::cout<<"\n";

		break;

		/// konfiguracja robota tak aby nie wystepowaly kolizje
		case 'c': case 'C':
			for(int i=0; i<18; i++)
				config[i]=-0.75;

				//uda
				config[1]=0.8;
				config[4]=0.8;
				config[7]=0.8;
				config[10]=0.8;
				config[13]=0.8;
				config[16]=0.8;

				/// Konfiguracja dla serwonapedow lydek
				config[2]=-2.3;
				config[5]=-2.3;
				config[8]=-2.3;
				config[11]=-2.3;
				config[14]=-2.3;
				config[17]=-2.3;

		break;

				/// konfiguracja robota tak aby wystapily 3 kolizje
				case 'v': case 'V':
			for(int i=0; i<18; i++)
				config[i]=-0.85;
				
				//uda
				config[1]=0.8;
				config[4]=0.8;
				config[7]=0.8;
				config[10]=0.8;
				config[16]=0.8;

				/// Konfiguracja dla serwonapedow lydek
				config[2]=-2.3;
				config[5]=-2.3;
				config[8]=-2.3;
				config[11]=-2.3;
				config[14]=-2.3;
				config[17]=-2.3;

				config[9]=-0.76;
				config[12]=0.15;
				config[6]=-0.75;
				config[13]=0.65;

		break;

				/// konfiguracja robota tak aby wystapilo 7 kolizji
				case 'b': case 'B':
			for(int i=0; i<18; i++)
				config[i]=-0.85;
				
				//uda
				config[1]=0.8;
				config[4]=0.8;
				config[7]=0.8;
				config[10]=0.8;
				config[16]=0.8;

				/// Konfiguracja dla serwonapedow lydek
				config[2]=-2.3;
				config[5]=-2.3;
				config[8]=-2.3;
				config[11]=-2.3;
				config[14]=-2.3;
				config[17]=-2.3;

				config[9]=-0.76;
				config[12]=0.15;
				config[6]=-0.93;
				config[13]=0.65;

		break;


			/// konfiguracja robota tak aby wystapilo 9 kolizji
			case 'n': case 'N':
			for(int i=0; i<18; i++)
				config[i]=-0.90;
				
			/// Konfiguracja dla serwonapedow ud
			config[1]=0.8;
			config[4]=0.8;
			config[7]=0.8;
			config[10]=0.8;
			config[13]=0.8;
			config[16]=0.8;

			/// Konfiguracja dla serwonapedow lydek
			config[2]=-2.3;
			config[5]=-2.3;
			config[8]=-2.3;
			config[11]=-2.3;
			config[14]=-2.3;
			config[17]=-2.3;

			config[15]=0.72;

		break;

			/// konfiguracja robota tak aby wystapilo 13 kolizji
			case 'm': case 'M':
			for(int i=0; i<18; i++)
				config[i]=-0.90;
				
			/// Konfiguracja dla serwonapedow ud
			config[1]=0.8;
			config[4]=0.8;
			config[7]=0.8;
			config[10]=0.8;
			config[13]=0.8;
			config[16]=0.8;

			/// Konfiguracja dla serwonapedow lydek
			config[2]=-2.3;
			config[5]=-2.3;
			config[8]=-2.3;
			config[11]=-2.3;
			config[14]=-2.3;
			config[17]=-2.3;

			config[15]=0.72;
			config[0]=0.72;

		break;

			/// konfiguracja robota tak aby wystapilo 16 kolizji
			case '7':
			for(int i=0; i<18; i++)
				config[i]=-0.90;
				
			/// Konfiguracja dla serwonapedow ud
			config[1]=0.8;
			config[4]=0.8;
			config[7]=0.8;
			config[10]=0.8;
			config[13]=0.8;
			config[16]=0.8;

			/// Konfiguracja dla serwonapedow lydek
			config[2]=-2.3;
			config[11]=-2.3;
			config[17]=-2.3;

			config[15]=0.72;
			config[0]=0.72;

			config[5]=-2.72;
			config[8]=-2.92;
			config[14]=-2.90;

		break;

			/// konfiguracja robota tak aby wystapilo 19 kolizji
			case '8':
			for(int i=0; i<18; i++)
				config[i]=-0.90;
				
			/// Konfiguracja dla serwonapedow ud
			config[1]=0.8;
			config[4]=0.8;
			config[7]=0.8;
			config[10]=0.8;
			config[13]=0.8;
			config[16]=0.8;

			/// Konfiguracja dla serwonapedow lydek
			config[2]=-2.3;
			config[11]=-2.3;
			config[17]=-2.3;

			config[5]=-2.88;
			config[8]=-2.92;
			config[14]=-2.90;

			config[0]=1.32;
			config[3]=-1.32;
			config[6]=-1.57;
			config[9]=-1.57;
			config[12]=1.71;
			config[15]=1.32;

		break;


		/// Zmiana kata padanie swiatla w OpenGL'u w osiach x i y
		case 't': case 'T':
			light_rotatex = light_rotatex + 5.0;
		break;

		case 'f': case 'F':
			light_rotatex = light_rotatex - 5.0;
		break;

		case 'y': case 'Y':
			light_rotatey = light_rotatey + 5.0;
		break;

		case 'g': case 'G':
			light_rotatey = light_rotatey - 5.0;
		break;
    }
}

/**********************************************************
 *
 * SUBROUTINE keyboard(int,int,int)
 *
 * Used to handle the keyboard input (not ASCII Characters)
 * 
 *********************************************************/

void keyboard_s (int key, int x, int y)
{  
     switch (key)
    {
       case GLUT_KEY_UP:
            rotation_x = rotation_x  +1;
        break;
        case GLUT_KEY_DOWN:
            rotation_x = rotation_x -1;
        break;
        case GLUT_KEY_LEFT:
            rotation_y  = rotation_y +1;
        break;
        case GLUT_KEY_RIGHT:
            rotation_y = rotation_y -1;
        break; 
    }
}
void MouseButton( int button, int state, int x, int y )
{
    if( button == GLUT_LEFT_BUTTON )
    {
        // zapamietanie stanu lewego przycisku myszki
        button_state = state;
        
        // zapamietanie polozenia kursora myszki
        if( state == GLUT_DOWN )
        {
            button_x = x;
            button_y = y;
        }
    }
}

// obsluga ruchu kursora myszki
void MouseMotion( int x, int y )
{
    if( button_state == GLUT_DOWN )
    {
        rotatey += 30 *(right1 - left1) / glutGet( GLUT_WINDOW_WIDTH ) *( x - button_x );
        button_x = x;
        rotatex -= 30 *( top - bottom ) / glutGet( GLUT_WINDOW_HEIGHT ) *( button_y - y );
        button_y = y;
        glutPostRedisplay();
    }
}

/**********************************************************
 *
 * SUBROUTINE display()
 *
 * This is our main rendering subroutine, called each frame
 * 
 *********************************************************/

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// This clear the background color to dark blue
    glMatrixMode(GL_MODELVIEW);		// Modeling transformation
    glLoadIdentity();	// Initialize the model matrix as identity
    
	glTranslatef(0,0, -18.0);	// We move the object forward (the model matrix is multiplied by the translation matrix)

	if (rotation_x > 359) rotation_x = 0;
	if (rotation_y > 359) rotation_y = 0;
	if (rotation_z > 359) rotation_z = 0;

    glRotatef(rotation_x,1.0,0.0,0.0);	// Rotations of the object (the model matrix is multiplied by the rotation matrices)
    glRotatef(rotation_y,0.0,1.0,0.0);
    glRotatef(rotation_z,0.0,0.0,1.0);
	glTranslatef(translation_x, 0.0, 0.0);
    glTranslatef(0.0, translation_y, 0.0);
	glTranslatef(0.0, 0.0, translation_z);

	glEnable(GL_NORMALIZE);

	// obroty obiektu za pomoca myszki
    glRotatef( rotatex, 1.0, 0, 0 );
    glRotatef( rotatey, 0, 1.0, 0 );

	glPushMatrix();
    glLoadIdentity();     /// macierz modelowania = macierz jednostkowa
    
    // obroty kierunku zrodla swiatla
    glRotatef(light_rotatex, 1.0, 0, 0 );
    glRotatef(light_rotatey, 0, 1.0, 0 );
    
    // ustalenie kierunku zrodla swiatla
    glLightfv( GL_LIGHT0, GL_POSITION, light_position );

    glPopMatrix();     // przywrocenie pierwotnej macierzy modelowania

	//pose = coldet::Quaternion(set_pose[0], set_pose[1], set_pose[2], set_pose[3])* coldet::Vec3(set_pose[4], set_pose[5], set_pose[6]);	
	pose = coldet::Vec3(set_pose[0], set_pose[1], set_pose[2])* Eigen::AngleAxisd (set_pose[3]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (set_pose[4]*M_PI/180, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd (set_pose[5]*M_PI/180, Eigen::Vector3d::UnitZ());
	czy_jest_kolizja=robot_structure->checkCollision (pose, config, collision_table);
	robot_structure->GLDrawRobot (pose, config, collision_table);


    glFlush(); // This force the execution of OpenGL commands
    glutSwapBuffers(); // In double buffered mode we invert the positions of the visible buffer and the writing buffer
}

/**********************************************************
 *
 * The main routine
 * 
 *********************************************************/

int main(int argc, char **argv)
{
    // We use the GLUT utility to initialize the window, to handle the input and to interact with the windows system
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(screen_width,screen_height);
    glutInitWindowPosition(400,200);
    glutCreateWindow("Model robota Messor II");    
	robot_structure = createCollisionDetectionColdet("Messor_II_Model.xml");  // Wczytywanie pliku .xml Messora II, wywolanie konstruktora.
	init();
    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutReshapeFunc (resize);
    glutKeyboardFunc (keyboard);
    glutSpecialFunc (keyboard_s);
    glutMouseFunc( MouseButton );
    glutMotionFunc( MouseMotion );

    glutMainLoop();
    return(0);    
}
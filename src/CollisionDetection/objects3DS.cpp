#include "../include/CollisionDetection/functions.h"
#include "../include/CollisionDetection/objects3DS.h"

CObjects3DS::CObjects3DS(){
	obj_qty =0;
}

CObjects3DS::~CObjects3DS(){
}

void CObjects3DS::Object3DS(int obj_qty, float scale)
{
    int i,j;
	float normal[3];
	float ** vert = new float*[3];
	for(int i = 0; i < 3; i++)
		vert[i] = new float[3];
//	glColor3f(0.5,0.5,0.5);	
	glBegin(GL_TRIANGLES); // glBegin and glEnd delimit the vertices that define a primitive (in our case triangles)
	for (j=0;j<object[obj_qty].polygons_qty;j++) {
		//----------------- FIRST VERTEX -----------------
		// Coordinates of the first vertex
		vert[0][0]=object[obj_qty].vertex[ object[obj_qty].polygon[j].a ].x*0.254*scale;
		vert[0][1]=object[obj_qty].vertex[ object[obj_qty].polygon[j].a ].y*0.254*scale;
		vert[0][2]=object[obj_qty].vertex[ object[obj_qty].polygon[j].a ].z*0.254*scale;
		//----------------- SECOND VERTEX -----------------
		// Coordinates of the second vertex
		vert[1][0]=object[obj_qty].vertex[ object[obj_qty].polygon[j].b ].x*0.254*scale;
		vert[1][1]=object[obj_qty].vertex[ object[obj_qty].polygon[j].b ].y*0.254*scale;
		vert[1][2]=object[obj_qty].vertex[ object[obj_qty].polygon[j].b ].z*0.254*scale;
		//----------------- THIRD VERTEX -----------------
		// Coordinates of the Third vertex
		vert[2][0]=object[obj_qty].vertex[ object[obj_qty].polygon[j].c ].x*0.254*scale;
		vert[2][1]=object[obj_qty].vertex[ object[obj_qty].polygon[j].c ].y*0.254*scale;
		vert[2][2]=object[obj_qty].vertex[ object[obj_qty].polygon[j].c ].z*0.254*scale;
		
		calcNormal(vert, normal);
		glNormal3d (normal[0],normal[1],normal[2]);

		glVertex3f(vert[0][0], vert[0][1], vert[0][2]);
		glVertex3f(vert[1][0], vert[1][1], vert[1][2]);
		glVertex3f(vert[2][0], vert[2][1], vert[2][2]);
	}
	glEnd();
	for(int i = 0; i < 3; i++)
		delete [] vert[i];
	delete [] vert;	
}

char CObjects3DS::ObjLoad(char *p_object_name)
{
    if (Load3DS (&object[obj_qty],p_object_name)==0) return(0);		//Object loading
    obj_qty++;		// Let's increase the object number and get ready to load another object!
	return (1);		// If all is ok then return 1
}



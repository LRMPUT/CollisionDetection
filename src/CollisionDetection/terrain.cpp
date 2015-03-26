#include "../include/CollisionDetection/terrain.h"
#include <stdlib.h>
#include <GL/glut.h> // glut.h includes gl.h and glu.h

CTerrain::CTerrain(void){
	MAP_X = 85;
    MAP_Z = 85;
}

CTerrain::~CTerrain(void){
}

// desc: initializes the heightfield terrain data
void CTerrain::InitializeTerrain()
{

	for (int z = 0; z < MAP_Z; z++)
	{
		for (int x = 0; x < MAP_X; x++)
		{
						
			terrain[x][z][0] = -(MAP_X/2.0+0.5)+float(x);				
			terrain[x][z][2] = (MAP_Z/2.0+0.5)-float(z);	
			if (((terrain[x][z][0]<1)||(terrain[x][z][0]>1)))
				terrain[x][z][1] = -25.4;//-1+(float)(x+z)/10;//(float)( (	imageData[counter] + imageData[counter+1] + imageData[counter+2]) /3.0f * MAP_SCALE);
			else
				terrain[x][z][1] = -25.4;
		}
	}
}

// desc: initializes the heightfield terrain data
void CTerrain::InitializeTerrain(double *** map, int offset_x, int offset_y)
{

	for (int z = 0; z < MAP_Z; z++)
	{
		for (int x = 0; x < MAP_X; x++)
		{
						
			terrain[x][z][0] = map[z+offset_x-int(MAP_Z/2)][x+offset_y-int(MAP_X/2)][0]*10;//-(MAP_X/2.0+0.5)+float(x);				
			terrain[x][z][2] = map[z+offset_x-int(MAP_Z/2)][x+offset_y-int(MAP_X/2)][1]*10;//(MAP_Z/2.0+0.5)-float(z);	
			terrain[x][z][1] = map[z+offset_x-int(MAP_Z/2)][x+offset_y-int(MAP_X/2)][2]*10;
		}
	}
}

void CTerrain::RenderTerrain_1()
{
	//glPolygonMode (GL_NONE, GL_POINT);
	glBegin(GL_TRIANGLES);
	for (int z = 1; z < MAP_Z; z++)
	{
		
		for (int x = 1; x < MAP_X; x++)
		{
			glVertex3f(terrain[x-1][z-1][0], terrain[x-1][z-1][1], terrain[x-1][z-1][2]);

			glVertex3f(terrain[x][z-1][0], terrain[x][z-1][1], terrain[x][z-1][2]);

			glVertex3f(terrain[x][z][0], terrain[x][z][1], terrain[x][z][2]);
		}		
	}
	glEnd();
}


void CTerrain::RenderTerrain_2()
{
	glBegin(GL_TRIANGLES);
	for (int z = 1; z < MAP_Z; z++)
	{
		for (int x = 1; x < MAP_X; x++)
		{
			glVertex3f(terrain[x-1][z-1][0], terrain[x-1][z-1][1], terrain[x-1][z-1][2]);

			glVertex3f(terrain[x][z][0], terrain[x][z][1], terrain[x][z][2]);
			
			glVertex3f(terrain[x-1][z][0], terrain[x-1][z][1], terrain[x-1][z][2]);
		}
	}
	glEnd();
}

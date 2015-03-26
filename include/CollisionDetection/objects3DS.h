#ifndef H_OBJECTS3DS
#define H_OBJECTS3DS

#include <stdlib.h>
#include <GL/glut.h>
#include "types.h"
#include "3dsloader.h"
#include "terrain.h"
#include <cstdint>
#include <iostream>
#include <vector>

class CObjects3DS {
public:
	CObjects3DS();
	~CObjects3DS();
	
	void Object3DS(int obj_qty, float scale = 1.0);
	char ObjLoad(char *p_object_name);

	obj_type object[MAX_OBJECTS];
	int obj_qty;

};
#endif

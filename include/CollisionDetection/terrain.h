#ifndef TERRAIN_H
#define TERRAIN_H

class CTerrain{
	public:

	CTerrain(void);
	~CTerrain(void);

	float MAP_X;				// size of map along x-axis
	float MAP_Z;				// size of map along z-axis

	float ***terrain;	

	void InitializeTerrain(void);
	// desc: initializes the heightfield terrain data
	void InitializeTerrain(double *** map, int offset_x, int offset_y);
	void RenderTerrain_1(void);
	void RenderTerrain_2(void);

};

#endif

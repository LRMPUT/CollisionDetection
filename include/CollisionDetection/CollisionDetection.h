/**********************************************************
*	
*		author: Tomasz Augustyn
* 
**********************************************************/

#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "../Defs/defs.h"
#include <iostream>
#include <string>
#include <vector>
#include "../include/CollisionDetection/3dsloader.h"
#include "../include/CollisionDetection/types.h"
#include "../include/CollisionDetection/coldet.h"
#include "../include/CollisionDetection/objects3DS.h"
#include <GL/glut.h>


namespace coldet{
	//CollisionDetection interface
	class CollisionDetection {
        public:
            /// CollisinDetection type
            enum Type {
                    /// RGB camera
                    TYPE_COLDET,
                    /// 2D Depth sensor
                    TYPE_VOXEL,
                    /// 3D Depth sensor
            };

			/// overloaded constructor
            CollisionDetection(const std::string _name, Type _type) : name(_name), type(_type) {};

			/// Name of the structure
			virtual const std::string& getName() const {
			return name;
			}

			/// Joints number and legs number
			int jointsNo;
			int legsNo;

			/// Initialize robot structure
			virtual void initStructures(void) = 0;

			/// Draw robot using openGL
			virtual void GLDrawRobot (const coldet::Mat34& pose, const std::vector<coldet::float_type>& config,  std::vector<bool>& collision_table) const = 0;

			/// Check collisions
			virtual bool checkCollision (const coldet::Mat34& pose, const std::vector<coldet::float_type>& config,  std::vector<bool>& collision_table) const = 0;

            /// Virtual descrutor
            virtual ~CollisionDetection() {}

        protected:
            /// CollisonDetection type
            Type type;

            /// CollisonDetection name
            const std::string name;
	};
};

#endif // _COLLISIONDETECTION_H_

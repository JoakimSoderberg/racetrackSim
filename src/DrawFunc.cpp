#include "DrawFunc.h"

void drawCircle(double radius, double x, double y, double z, int step, GLfloat color[])
{
	glDisable( GL_TEXTURE_2D );
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_LINES);

		//Inital point
        double xp =  radius + x;
        double zp =  z;
        double yp =  y;

		for(int j = 0; j < 360; j+=step)
		{
            glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
            xp = radius * cos(j * M_PI/180.0) + x;
            zp = radius * sin(j * M_PI/180.0) + z;

            glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
		}

        glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
        glVertex3f((GLfloat)(radius + x), (GLfloat)y, -(GLfloat)z);

	glEnd();
	glColor3f(1.0f,1.0f,1.0f);
	glEnable( GL_TEXTURE_2D );
	
}

void drawSolidCircle(double radius, double x, double y, double z, int step, GLfloat color[])
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	glColor4f(color[0], color[1], color[2], 0.5f);
	glBegin(GL_TRIANGLES);
		
     
		//Inital point
        double xp = radius + x;
        double zp =  z;
        double yp =  y;

		for(int j = 0; j < 360; j+=step)
		{
            glVertex3f((GLfloat)x, (GLfloat)y, -(GLfloat)z);
            glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
            xp = (double)radius * cos(j * M_PI/180.0f) + x;
            zp = (double)radius * sin(j * M_PI/180.0f) + z;
            glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
		}

        glVertex3f((GLfloat)x, (GLfloat)y, -(GLfloat)z);
        glVertex3f((GLfloat)xp, (GLfloat)yp, -(GLfloat)zp);
        glVertex3f((GLfloat)(radius + x), (GLfloat)y, -(GLfloat)z);
		
	glEnd();
	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
	
}

void drawTrianlge(double heading, double x, double y, double base, GLfloat color[])
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glDisable( GL_TEXTURE_2D );
    glColor4f(color[0], color[1], color[2], 1.0f);
    glBegin(GL_TRIANGLES);

    Point2D p1 = Point2D(base, 0);
    Point2D p2 = Point2D(-base, 0);
    Point2D p3 = Point2D(0, base);

    p1 = p1.rotate(heading);
    p2 = p2.rotate(heading);
    p3 = p3.rotate(heading);

    glVertex3f((GLfloat)p1.x + x, (GLfloat)0.25, -(GLfloat)(p1.y + y));
    glVertex3f((GLfloat)p2.x + x, (GLfloat)0.25, -(GLfloat)(p2.y + y));
    glVertex3f((GLfloat)p3.x + x, (GLfloat)0.25, -(GLfloat)(p3.y + y));

    glEnd();
    glColor4f(1.0f,1.0f,1.0f, 1.0f);
    glEnable( GL_TEXTURE_2D );
    glDisable(GL_BLEND);

}

void drawLine(double x, double y, double z, double xe, double ye, double ze, GLfloat color[])
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	glColor4f(color[0], color[1], color[2], 0.5f);
	glBegin(GL_LINES);
        glVertex3f((GLfloat)x, (GLfloat)y, -(GLfloat)z);
        glVertex3f((GLfloat)xe, (GLfloat)ye, -(GLfloat)ze);
	glEnd();
	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
}

void drawWPTrajectory(list<WayPoint> referenceTrajectory, double scale, double drawOffset_x, double drawOffset_y, GLfloat color[], GLfloat color2[])
{  

	list<WayPoint>::const_iterator wp;
	for(wp=referenceTrajectory.begin(); wp!=referenceTrajectory.end(); ++wp)
	{
		list<WayPoint>::const_iterator wp_next = wp;
		wp_next++;
		if(wp_next != referenceTrajectory.end())
            drawLine((wp->wp.x - drawOffset_x)*scale, 0.2, (wp->wp.y - drawOffset_y)*scale, (wp_next->wp.x - drawOffset_x)*scale, 0.15, (wp_next->wp.y - drawOffset_y)*scale, color2);
		
        drawSolidCircle(0.5f, (wp->wp.x - drawOffset_x)*scale, 0.15, (wp->wp.y - drawOffset_y)*scale, 30, color);
        GLfloat colorTriangle[] = {1.0f, 0.0f, 0.0f};
        if(wp->type == headingConstrained)
            drawTrianlge(wp->heading- M_PI/2, (wp->wp.x - drawOffset_x)*scale, (wp->wp.y - drawOffset_y)*scale, 0.5f, colorTriangle);
	}
}

void drawTreeForward(list<Node*> *nodeList, double SCENE_SCALE, double drawOffset_x, double drawOffset_y, bool DRAW_CONTROL, bool DRAW_TRAJ)
{

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	

	// Draw the graph
    if (nodeList) 
	{
        
        // Draw the vertices
        glPointSize (6.0);
        float color_node[] = {0.1, 0.1, 0.8, 1.0};
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (GLfloat*) color_node);        
        glBegin (GL_POINTS);
		glColor4f(0.2f, 0.2f, 0.8f, 1.0f);
		list<Node*>::const_iterator node;

		if(DRAW_CONTROL)
		{
			
			for(node=nodeList->begin(); node!=nodeList->end(); node++)
			{
			
                Point2D s = (*node)->getStateKey();

			
				if((*node)->isSampleNode)
				{
				
					 glColor4f(0.2f, 1.0f, 0.8f, 1.0f);
					 glPointSize (20.0);

				
				}
				else
				{
				// glPointSize (6.0);
				 glColor4f(0.2f, 0.2f, 0.8f, 1.0f);
				}

                  glVertex3f ((GLfloat)(s.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
						   0.2f,
                           -(GLfloat)(s.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
				glPointSize (6.0);

			}
		}
        glEnd();
       
       
        // Draw the edges
	    glLineWidth (2.0); 
		
	   for(node=nodeList->begin(); node!=nodeList->end(); node++)
	   {      
		   if(!(*node)->isRoot())
		   {
                Point2D me = (*node)->getStateKey();
                Point2D parent = (*node)->getParent().getStateKey();
			   if(DRAW_CONTROL)
			   {
					glBegin (GL_LINE_STRIP);
					glColor4f(0.2f, 0.2f, 0.8f, 1.0f);
                    glVertex3f ((GLfloat)(me.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.2f,
                                -(GLfloat)(me.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
                    glVertex3f ((GLfloat)(parent.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.2f,
                                -(GLfloat)(parent.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
					glEnd();
			   }

				//draw trajectories
						
				GLfloat color[] = {1.0, 0.0, 0.0, 1.0};;
				if((*node)->isSafe())
				{
					color[0] = 1.0f;
					color[1] = 1.0f;
					color[2] = 0.0f;
					color[3] = 1.0f;
				}

				if(DRAW_TRAJ)
				{
					Trajectory &traj = (*node)->getTrajectory();
					if(&traj){
                    if(traj.stateList.size() >= 2)
					{
			    
						list<State>::const_iterator trajectoryState;
                        for(trajectoryState = traj.stateList.begin(); trajectoryState != traj.stateList.end(); trajectoryState++)
						{
					
							glPointSize (4.0);
							glBegin (GL_POINTS);
							glColor4f(color[0], color[1], color[2], color[3]);
                            double hej = trajectoryState->x;
                            glVertex3f ((GLfloat)(trajectoryState->x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.15f,
                                        -(GLfloat)(trajectoryState->y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
						
							glEnd();

						
						
						}
					 }
				 }
			 }
		  }	   
       }
    }

	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
}

void drawOptimalTrajectorylist(std::vector<State> state_for_opt_traj_vector, double scale, double drawOffset_x, double drawOffset_y, GLfloat color[], GLfloat color2[]){
	
	
	for(vector<State>::iterator state = state_for_opt_traj_vector.begin(); state !=state_for_opt_traj_vector.end(); state++)
	{
		drawSolidCircle(0.2f, (state->x - drawOffset_x)*scale, 0.15, (state->y - drawOffset_y)*scale, 30, color);
	}
	
}

void drawTreeReverse(list<Node*> *nodeList, double SCENE_SCALE, double drawOffset_x, double drawOffset_y, bool DRAW_CONTROL, bool DRAW_TRAJ)
{

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	

	// Draw the graph
    if (nodeList) 
	{
        
        // Draw the vertices
        glPointSize (6.0);
        float color_node[] = {0.8, 0.1, 0.1, 1.0};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_node);        
        glBegin (GL_POINTS);
		glColor4f(0.8f, 0.2f, 0.2f, 1.0f);
		list<Node*>::const_iterator node;
		if(DRAW_CONTROL)
		{
			for(node=nodeList->begin(); node!=nodeList->end(); node++)
			{
			
                Point2D s = (*node)->getStateKey();

				if((*node)->isSampleNode)
				{
				
					 glColor4f(0.2f, 1.0f, 0.8f, 1.0f);
					  glPointSize (20.0);
				}
				else
				{
				// glPointSize (6.0);
				 glColor4f(0.2f, 0.2f, 0.8f, 1.0f);
				}

                glVertex3f ((GLfloat)(s.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
						   0.2f,
                           -(GLfloat)(s.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
				glPointSize (6.0);
			}
		}
        glEnd();
       
       
        // Draw the edges
	    glLineWidth (2.0); 
		
	   for(node=nodeList->begin(); node!=nodeList->end(); node++)
	   {      
		   if(!(*node)->isRoot())
		   {
                Point2D me = (*node)->getStateKey();
                Point2D parent = (*node)->getParent().getStateKey();
			   
				if(DRAW_CONTROL)
				{
					glBegin (GL_LINE_STRIP);
					glColor4f(0.8f, 0.2f, 0.2f, 1.0f);
                    glVertex3f ((GLfloat)(me.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.2f,
                                -(GLfloat)(me.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
                    glVertex3f ((GLfloat)(parent.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.2f,
                                -(GLfloat)(parent.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
					glEnd();
				}

				//draw trajectories
					
				GLfloat color[] = {1.0, 0.0, 0.0, 1.0};;
				if((*node)->isSafe())
				{
					color[0] = 0.0f;
					color[1] = 1.0f;
					color[2] = 1.0f;
					color[3] = 1.0f;
				}

				if(DRAW_TRAJ)
				{
					Trajectory &traj = (*node)->getTrajectory();
					if(&traj){
                    if(traj.stateList.size() >= 2)
					{
			    
						list<State>::const_iterator trajectoryState;
                        for(trajectoryState = traj.stateList.begin(); trajectoryState != traj.stateList.end(); trajectoryState++)
						{
					
							glPointSize (4.0);
							glBegin (GL_POINTS);
							glColor4f(color[0], color[1], color[2], color[3]);
                            glVertex3f ((GLfloat)(trajectoryState->x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.15f,
                                        -(GLfloat)(trajectoryState->y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
						
							glEnd();

						
						
						}
					}
				}
			}
		   }	   
        }
    }

	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
}

void drawRoadCenter(vector<Point2D> &centerOfRoad, double SCENE_SCALE, double drawOffset_x, double drawOffset_y)
{
    //glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glDisable( GL_TEXTURE_2D );
    glColor4f(0.5f, 1.0f, 0.0f, 1.0f);

    int r = centerOfRoad.size();

    for(int i = 0; i < r - 1; i++)
    {
        /****************************************************************/
        /*** Draw edge lines ********************************************/
        /****************************************************************/
        glLineWidth (4.0);

        glBegin (GL_LINE_STRIP);

            glVertex3f ((GLfloat)(centerOfRoad[i].x - drawOffset_x)*SCENE_SCALE,
                        0.3f,
                        -(GLfloat)(centerOfRoad[i].y - drawOffset_y)*SCENE_SCALE);

            glVertex3f ((GLfloat)(centerOfRoad[i+1].x - drawOffset_x)*SCENE_SCALE,
                        0.3f,
                        -(GLfloat)(centerOfRoad[i+1].y - drawOffset_y)*SCENE_SCALE);
        glEnd();

         glLineWidth (2.0);
    }



    glColor4f(1.0f,1.0f,1.0f, 1.0f);
    glEnable( GL_TEXTURE_2D );
    glDisable(GL_BLEND);
}

void drawSafeNodes(list<Node*> *nodeList, double SCENE_SCALE, double drawOffset_x, double drawOffset_y)
{

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	

	glPointSize (10.0);
	glColor4f(0.0f,1.0f,0.0f, 1.0f);

	list<Node*>::const_iterator node;
	for(node=nodeList->begin(); node!=nodeList->end(); node++)
	{
		
					
		glBegin (GL_POINTS);
        glVertex3f ((GLfloat)((*node)->getState().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
					0.15f,
                    -(GLfloat)((*node)->getState().y*SCENE_SCALE  -drawOffset_y*SCENE_SCALE));
					
		glEnd();

					
	}

	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
}

void drawLowCostPath(list<Node*> *nodeList, Node& lowCostNode, Point2D commitedControlPoint, double SCENE_SCALE, double drawOffset_x, double drawOffset_y)
{

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	

	// Draw the graph
    if (nodeList) 
	{
        
        // Draw the vertices
        glPointSize (6.0);
        float color_node[] = {1.0, 1.0, 0.0, 1.0};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_node);        
        
			
		Node* currentNode = &lowCostNode;
		


		while(!currentNode->isRoot())
		{
			
			/****************************************************************/
			/*** Draw points at control points   ****************************/
			/****************************************************************/
			glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
			glBegin(GL_POINTS);
            glVertex3f ((GLfloat)(currentNode->getStateKey().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
						0.3f,
                        -(GLfloat)(currentNode->getStateKey().y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
			glEnd();



			/****************************************************************/
			/*** Draw edge lines ********************************************/
			/****************************************************************/
			glLineWidth (2.0); 
		
			glBegin (GL_LINE_STRIP);
            
                glVertex3f ((GLfloat)(currentNode->getStateKey().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
							0.3f,
                            -(GLfloat)(currentNode->getStateKey().y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
                glVertex3f ((GLfloat)(currentNode->getParent().getStateKey().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
							0.3f,
                            -(GLfloat)(currentNode->getParent().getStateKey().y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
			glEnd();

			/****************************************************************/
			/*** Draw trajectories ******************************************/
			/****************************************************************/
			Trajectory &traj = currentNode->getTrajectory();
            if(traj.stateList.size() >= 2)
			{
			    
				list<State>::const_iterator trajectoryState;
                for(trajectoryState = traj.stateList.begin(); trajectoryState != traj.stateList.end(); trajectoryState++)
				{
					
			   
					glBegin (GL_POINTS);
					if(currentNode->getDirection() == Direction::forwardDirection)
						glColor3f(1.0f,0.0f,0.0f);
					else
						glColor3f(0.0f,1.0f,1.0f);

                    glVertex3f ((GLfloat)(trajectoryState->x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.3f,
                                -(GLfloat)(trajectoryState->y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
					
					glEnd();

					
				}
			}


			currentNode = &currentNode->getParent();
		  }

		//here current node will be the root
		/****************************************************************/
		/*** Draw comitted part *****************************************/
		/****************************************************************/
		/****************************************************************/
		/****************************************************************/
		/*** Draw points at control points   ****************************/
		/****************************************************************/
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		
		glPointSize (20.0);
		glBegin(GL_POINTS);
            glVertex3f ((GLfloat)(currentNode->getStateKey().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
						0.3f,
                        -(GLfloat)(currentNode->getStateKey().y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
			glEnd();

			/****************************************************************/
			/*** Draw edge lines ********************************************/
			/****************************************************************/
			glLineWidth (4.0); 
		
			glBegin (GL_LINE_STRIP);
            
                glVertex3f ((GLfloat)(currentNode->getStateKey().x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
							0.3f,
                            -(GLfloat)(currentNode->getStateKey().y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
                glVertex3f ((GLfloat)(commitedControlPoint.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
							0.3f,
                            -(GLfloat)(commitedControlPoint.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
			glEnd();


		/****************************************************************/
		/*** Draw trajectory ********************************************/
		/****************************************************************/
		Trajectory &traj = currentNode->getTrajectory();
		if(&traj)
		{
			glPointSize (8.0);
			glColor3f(0.0f,0.0f,1.0f);
            if(traj.stateList.size() >= 2)
			{
				list<State>::const_iterator trajectoryState;
                for(trajectoryState = traj.stateList.begin(); trajectoryState != traj.stateList.end(); trajectoryState++)
				{
					
					
					glBegin (GL_POINTS);
                    glVertex3f ((GLfloat)(trajectoryState->x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
								0.3f,
                                -(GLfloat)(trajectoryState->y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));
					
					glEnd();

					
				}
			}
		}
          
    }

	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);
}

void putText3D(string s, int x, int y, int z)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glDisable( GL_TEXTURE_2D );
    glColor4f(1.0, 1.0, 0.0, 1.0f);

//    glMatrixMode(GL_PROJECTION);
//    glPushMatrix();
//    glLoadIdentity();
//    //gluOrtho2D(0.0, width, 0.0, height);
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glLoadIdentity();
    glRasterPos3f(x, 2 ,y);

    void * font = GLUT_BITMAP_9_BY_15;
    for (string::iterator i = s.begin(); i != s.end(); ++i)
    {
      char c = *i;
      glutBitmapCharacter(font, c);
    }
//    glMatrixMode(GL_PROJECTION);
//    glPopMatrix();
//    glMatrixMode(GL_MODELVIEW);
//    glPopMatrix();

    glColor4f(1.0f,1.0f,1.0f, 1.0f);
    glEnable( GL_TEXTURE_2D );
    glDisable(GL_BLEND);

}


void putText(string s, int x, int y, int width, int height)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glDisable( GL_TEXTURE_2D );
	glColor4f(1.0, 1.0, 0.0, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0.0, width, 0.0, height);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2i(x, y);
	
	void * font = GLUT_BITMAP_9_BY_15;
	for (string::iterator i = s.begin(); i != s.end(); ++i)
	{
	  char c = *i;
	  glutBitmapCharacter(font, c);
	}
	glMatrixMode(GL_PROJECTION); 
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glColor4f(1.0f,1.0f,1.0f, 1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_BLEND);

}

void drawRectObstacle(list<RectObstacle> *staticObstacles, double SCENE_SCALE, double drawOffset_x, double drawOffset_y)
{
	list<RectObstacle>::iterator obstacle;

	for(obstacle = staticObstacles->begin(); obstacle != staticObstacles->end(); obstacle++)
	{
        double x1 = obstacle->perimiter[0].a.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE;
        double x2 = obstacle->perimiter[1].a.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE;
        double x3 = obstacle->perimiter[2].a.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE;
        double x4 = obstacle->perimiter[3].a.x*SCENE_SCALE - drawOffset_x*SCENE_SCALE;

        double y1 = obstacle->perimiter[0].a.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE;
        double y2 = obstacle->perimiter[1].a.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE;
        double y3 = obstacle->perimiter[2].a.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE;
        double y4 = obstacle->perimiter[3].a.y*SCENE_SCALE - drawOffset_y*SCENE_SCALE;

		double height = 3*SCENE_SCALE;
	// TOP
		glBegin(GL_POLYGON);
			glColor3f(   0.0,  0.0,  0.0 );
                glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
                glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
                glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
                glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
		glEnd();

		glBegin(GL_POLYGON);
		glColor3f(   1.0,  0.0,  0.0 );
                glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
                glVertex3f( (GLfloat)x1, (GLfloat)0.0, -(GLfloat)y1 );
                glVertex3f( (GLfloat)x2, (GLfloat)0.0, -(GLfloat)y2 );
                glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
		glEnd();

		glBegin(GL_POLYGON);
			
                glVertex3f( (GLfloat)x2, (GLfloat)height, -(GLfloat)y2 );
                glVertex3f( (GLfloat)x2, (GLfloat)0.0, -(GLfloat)y2 );
                glVertex3f( (GLfloat)x3, (GLfloat)0.0, -(GLfloat)y3 );
                glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
		glEnd();

		glBegin(GL_POLYGON);
			
                glVertex3f( (GLfloat)x3, (GLfloat)height, -(GLfloat)y3 );
                glVertex3f( (GLfloat)x3, (GLfloat)0.0, -(GLfloat)y3 );
                glVertex3f( (GLfloat)x4, (GLfloat)0.0, -(GLfloat)y4 );
                glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
		glEnd();

		glBegin(GL_POLYGON);
			
                glVertex3f( (GLfloat)x4, (GLfloat)height, -(GLfloat)y4 );
                glVertex3f( (GLfloat)x4, (GLfloat)0.0, -(GLfloat)y4 );
                glVertex3f( (GLfloat)x1, (GLfloat)0.0, -(GLfloat)y1 );
                glVertex3f( (GLfloat)x1, (GLfloat)height, -(GLfloat)y1 );
		glEnd();
	}
}

/*void drawCostMap(mock_lcm::grid_map  *lcmMap, State egoState, double SCENE_SCALE, double drawOffset_x, double drawOffset_y)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable( GL_TEXTURE_2D );


    glPointSize (3.0);


    for(int i = 0; i < lcmMap->cols; i ++)
    {
        for(int j = 0; j < lcmMap->rows; j++)
        {
            if(lcmMap->cost[i][j] > -128)
                glColor4f(1.0f,1.0f,1.0f, 0.9f);
            else if (i == 0)
                glColor4f(0.0f,0.0f,1.0f, 1.0f);
            else if (j == 0)
                glColor4f(0.0f,1.0f,1.0f, 1.0f);
            else
                glColor4f(1.0f,0.0f,0.0f, 0.1f);

           double x = ((lcmMap->rows-i)-lcmMap->rows/2)*0.25;
           double y = ((lcmMap->rows-j)-lcmMap->cols/2)*0.25;

           double x_rot = x;//cos(egoState.heading)*x - sin(egoState.heading)*y;
           double y_rot = y; //sin(egoState.heading)*x + cos(egoState.heading)*y;

           x = x_rot + egoState.x;
           y = y_rot + egoState.y;



            glBegin (GL_POINTS);
            glVertex3f ((GLfloat)(x*SCENE_SCALE - drawOffset_x*SCENE_SCALE),
                        0.15f,
                        -(GLfloat)(y*SCENE_SCALE - drawOffset_y*SCENE_SCALE));

            glEnd();
        }
    }

//    for(int i = 0; i < 400; i ++)
//    {
//        for(int j = 0; j < 400; j++)
//        {
//            if(j > 200)
//                glColor4f(1.0f,0.0f,0.0f, 0.1f);
//            else
//                glColor4f(0.0f,0.0f,0.0f, 0.5f);

//            glBegin (GL_POINTS);
//            glVertex3f ((i-200)*0.25*SCENE_SCALE,
//                        0.15f,
//                        (j-200)*0.25*SCENE_SCALE);

//            glEnd();
//        }
//    }

    glColor4f(1.0f,1.0f,1.0f, 1.0f);
    glEnable( GL_TEXTURE_2D );
    glDisable(GL_BLEND);



}*/

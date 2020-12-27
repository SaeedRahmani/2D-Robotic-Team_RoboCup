/*
Copyright (c) 2007-2008, Saeed Rahmani, Robotic Institute
All rights reserved.A

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the Robotic Institute nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*! \file PlayerTeams.cpp
<pre>
<b>File:</b>          PlayerTest.cpp
<b>Project:</b>       Robocup Soccer Simulation: iRobot2D-Boys
<b>Authors:</b>       Saeed Rahmani,Hussein Shirvani,Milad Mohammadi
<b>Created:</b>       05/02/2008
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
                      Player that are used to test the teams' high level
                      strategy.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
05/02/2008        Milad Mohammadi,Hussein Shirvani,Saeed Rahmani       Initial version created
</pre>
*/

#include "Player.h"

/*!This method is the first complete simple team and defines the actions taken
   by all the players on the field (excluding the goalie). It is based on the
   high-level actions taken by the simple team FC Portugal that it released in
   2000. The players do the following:
   - if ball is kickable
       kick ball to goal (random corner of goal)
   - else if i am fastest player to ball 
       intercept the ball
   - else
       move to strategic position based on your home position and pos ball */

double abs(double a)
{
if(a>=0) 
	return a;
else
	 return -1*a;
}







double absP(double n)
{
if(n<0) return -1*n;
else if(n==0) return 0;
else return n;
}



int Player::Free (int a)
{
VecPosition   posAgent = WM->getAgentGlobalPosition();
int angle = a;
int opop=WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,posAgent+VecPosition(7,angle,POLAR));
return opop;
}

//
//====================GET BEST DRIBBLE=======================\\
//
SoccerCommand Player::getBestDribble()
{

SoccerCommand soc;
VecPosition   posAgent = WM->getAgentGlobalPosition();

double x=posAgent.getX(),y=posAgent.getY(),d=10;
DribbleT drT;
double dribbleAngle;

if(x<33)
{
for(int i=0;i<=180;i+=10)
	{
	if(i==0)
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+15,y))==0)
			{
			dribbleAngle=0;
			drT=DRIBBLE_FAST;
			break;
			}
		else continue;
		}
	else
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,
					VecPosition(7.0,i,POLAR)+posAgent)==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=i;
			break;
			}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,
					VecPosition(7.0,-i,POLAR)+posAgent)==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=-1*i;
			break;
			}
		else continue;
		}
	}
}
else
{
 	ObjectT o       = WM->getClosestRelativeInSet (   OBJECT_SET_OPPONENTS   );
 	VecPosition VP  = WM->getGlobalPosition       ( WM->getAgentObjectType() );
 	VecPosition VP1 = WM->getGlobalPosition       (             o            );
 	double ourY     = VP .getY();
 	double hisY     = VP1.getY();
	double dist=6;
	drT 	        = DRIBBLE_FAST;
	VecPosition VP2(x,y);                                              //end of cone
	dribbleAngle = WM->getRelAngleOpponentGoal();
	for(int i=0;i<=180;i+=6)
		{
		x  = cosDeg( dribbleAngle+i )*dist+VP.getX();
		y  = sinDeg( dribbleAngle+i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=dribbleAngle+i;
			break;
			}
		x            = cosDeg( dribbleAngle-i )*dist+VP.getX();
		y            = sinDeg( dribbleAngle-i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=dribbleAngle-i;
			break;
			}
		}	
	}
soc=dribble(dribbleAngle,drT);

return soc;
}	
//============================GETBESTDRIBBLE1()============================//


SoccerCommand Player::getBestDribble1()
{

SoccerCommand soc;
VecPosition   posAgent = WM->getAgentGlobalPosition();

double x=posAgent.getX(),y=posAgent.getY(),d=10;
DribbleT drT;
double dribbleAngle;

if(x<33)
{
for(int i=90;i<=270;i+=10)
	{
	if(i==90)
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+d,y))==0)
			{
			dribbleAngle = 90;
			drT=DRIBBLE_FAST;
			break;
			}
		else continue;
		}
	else
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(i)*d,y+sinDeg(i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=i;
			break;
			}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(-i)*d,y+sinDeg(-i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle = -1*i;
			break;
			}
		else continue;
		}
	}
}
else
{
 	ObjectT o       = WM->getClosestRelativeInSet (   OBJECT_SET_OPPONENTS   );
 	VecPosition VP  = WM->getGlobalPosition       ( WM->getAgentObjectType() );
 	VecPosition VP1 = WM->getGlobalPosition       (             o            );
 	double ourY     = VP .getY();
 	double hisY     = VP1.getY();
	double dist=6;
	drT 	        = DRIBBLE_FAST;
	VecPosition VP2(x,y);                                              //end of cone
	dribbleAngle = WM->getRelAngleOpponentGoal();
	for(int i=90;i<=270;i+=6)
		{
		x  = cosDeg( dribbleAngle+i )*dist+VP.getX();
		y  = sinDeg( dribbleAngle+i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_FAST;
			dribbleAngle=dribbleAngle+i;
			break;
			}
		x            = cosDeg( dribbleAngle-i )*dist+VP.getX();
		y            = sinDeg( dribbleAngle-i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=dribbleAngle-i;
			break;
			}
		}	
	}
soc=dribble(dribbleAngle,drT);

return soc;
}	


//=================================GETBESTDRIBBLE2()==============================//


SoccerCommand Player::getBestDribble2()
{

SoccerCommand soc=CMD_ILLEGAL;

VecPosition posAgent=WM->getAgentGlobalPosition();
double x=posAgent.getX(),y=posAgent.getY(),d=10;
DribbleT drT;
double dribbleAngle;

if(x<30)
{
for(int i=-90;i<=90;i+=10)
	{
	if(i==-90)
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+d,y))==0)
			{
			dribbleAngle=-90;
			drT=DRIBBLE_FAST;
			break;
			}
		else continue;
		}
	else
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(i)*d,y+sinDeg(i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=i;
			break;
			}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(-i)*d,y+sinDeg(-i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=-1*i;
			break;
			}
		else continue;
		}
	}
}
else
{
 	ObjectT o       = WM->getClosestRelativeInSet (   OBJECT_SET_OPPONENTS   );
 	VecPosition VP  = WM->getGlobalPosition       ( WM->getAgentObjectType() );
 	VecPosition VP1 = WM->getGlobalPosition       (             o            );
 	double ourY     = VP .getY();
 	double hisY     = VP1.getY();
	double dist=6;
	drT 	        = DRIBBLE_FAST;
	VecPosition VP2(x,y);                                              //end of cone
	dribbleAngle = WM->getRelAngleOpponentGoal();
	for(int i=-90;i<=90;i+=6)
		{
		x  = cosDeg( dribbleAngle+i )*dist+VP.getX();
		y  = sinDeg( dribbleAngle+i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_FAST;
			dribbleAngle=dribbleAngle+i;
			break;
			}
		x            = cosDeg( dribbleAngle-i )*dist+VP.getX();
		y            = sinDeg( dribbleAngle-i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=dribbleAngle-i;
			break;
			}
		}	
	}
soc=dribble(dribbleAngle,drT);
return soc;
}	


//=================================dribble==================

SoccerCommand Player::Dribble(int angle)
{

SoccerCommand soc=CMD_ILLEGAL;
int Ang = angle;
VecPosition posAgent=WM->getAgentGlobalPosition();
double x=posAgent.getX(),y=posAgent.getY(),d=10;
DribbleT drT;
double dribbleAngle;

if(x<30)
{
for(int i=Ang;i<=Ang+180;i+=10)
	{
	if(i==Ang)
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+d,y))==0)
			{
			dribbleAngle=0;
			drT=DRIBBLE_FAST;
			break;
			}
		else continue;
		}
	else
		{
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(i)*d,y+sinDeg(i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=i;
			break;
			}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,posAgent,VecPosition(x+cosDeg(-i)*d,y+sinDeg(-i)*d))==0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=-1*i;
			break;
			}
		else continue;
		}
	}
}
else
{
 	ObjectT o       = WM->getClosestRelativeInSet (   OBJECT_SET_OPPONENTS   );
 	VecPosition VP  = WM->getGlobalPosition       ( WM->getAgentObjectType() );
 	VecPosition VP1 = WM->getGlobalPosition       (             o            );
 	double ourY     = VP .getY();
 	double hisY     = VP1.getY();
	double dist=6;
	drT 	        = DRIBBLE_FAST;
	VecPosition VP2(x,y);                                              //end of cone
	dribbleAngle = WM->getRelAngleOpponentGoal();
	for(int i=Ang;i<=Ang+180;i+=6)
		{
		x  = cosDeg( dribbleAngle+i )*dist+VP.getX();
		y  = sinDeg( dribbleAngle+i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_FAST;
			dribbleAngle=dribbleAngle+i;
			break;
			}
		x            = cosDeg( dribbleAngle-i )*dist+VP.getX();
		y            = sinDeg( dribbleAngle-i )*dist+VP.getY();
		VP2.setX(x);
		VP2.setY(y);
		if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,0.5,VP,VP2) == 0)
			{
			drT=DRIBBLE_SLOW;
			dribbleAngle=dribbleAngle-i;
			break;
			}
		}	
	}
soc=dribble(dribbleAngle,drT);
return soc;
}	





//====================GET BEST KICK=======================\\
//

 SoccerCommand Player::getBestKick()
{
SoccerCommand soc(CMD_ILLEGAL);
VecPosition posAgent = WM->getAgentGlobalPosition();

 VecPosition pos1=WM->getGlobalPosition(OBJECT_TEAMMATE_1);
 VecPosition pos2=WM->getGlobalPosition(OBJECT_TEAMMATE_2);
 VecPosition pos3=WM->getGlobalPosition(OBJECT_TEAMMATE_3);
 VecPosition pos4=WM->getGlobalPosition(OBJECT_TEAMMATE_4);
 VecPosition pos5=WM->getGlobalPosition(OBJECT_TEAMMATE_5);
 VecPosition pos6=WM->getGlobalPosition(OBJECT_TEAMMATE_6);
 VecPosition pos7=WM->getGlobalPosition(OBJECT_TEAMMATE_7);	
 VecPosition pos8=WM->getGlobalPosition(OBJECT_TEAMMATE_8);
 VecPosition pos9=WM->getGlobalPosition(OBJECT_TEAMMATE_9);
 VecPosition pos10=WM->getGlobalPosition(OBJECT_TEAMMATE_10);
 VecPosition pos11=WM->getGlobalPosition(OBJECT_TEAMMATE_11);
 
 double x=posAgent.getX();
 double y=posAgent.getY();
 int free = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(5.0),posAgent,VecPosition(x+7,y));
 int free1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(5.0),posAgent,VecPosition(x,y+4));
 int free2 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(5.0),posAgent,VecPosition(x,y-4)); 	    
 
 int nron = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
					,pos9);	
 int nrod = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
					,pos10);	
int nroy = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
					,pos11);	
 int nry = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,-45.0,POLAR)+pos11);
 int nry1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent
			,VecPosition(5.0,45.0,POLAR)+pos11);
 int nros = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6);	

 int nrd = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,45.0,POLAR)+pos10);
 int nrd1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,-45.0,POLAR)+pos10); 

	VecPosition posGKopp(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE));
	double disown=posAgent.getDistanceTo(VecPosition(52.5,0));
	double disoppGK=posAgent.getDistanceTo(posGKopp);

	double lenup,lendown;

	Line L1=Line::makeLineFromTwoPoints(posAgent,VecPosition(52.5,-1*7));
	Line L2=Line::makeLineFromTwoPoints(posAgent,VecPosition(52.5,7));

	double aL1=L1.getBCoefficient(),bL1=L1.getACoefficient(),cL1=L1.getCCoefficient(),
	aL2=L2.getBCoefficient(),bL2=L2.getACoefficient(),cL2=L2.getCCoefficient();

	lenup=absP(aL1*posGKopp.getX()+bL1*posGKopp.getY()+cL1)/(sqrt(aL1*aL1+bL1*bL1));
	lendown=absP(aL2*posGKopp.getX()+bL2*posGKopp.getY()+cL2)/(sqrt(aL2*aL2+bL2*bL2));

	/*if(WM->isVisible(OBJECT_OPPONENT_GOALIE))
		{
		if(disoppGK<8)
		if(lenup<lendown)
			{
			soc=kickTo(VecPosition(52.5,5.75),SS->getBallSpeedMax());
			}
		else 
			{
			soc=kickTo(VecPosition(52.5,-5.75),SS->getBallSpeedMax());
			}
		}



	       */
		 if((posAgent.getX()>=31.0)&&((lenup>0.9)||(lendown>0.9))&&(disown<18))
		{
		if(lenup<lendown)
			{
			soc=kickTo(VecPosition(52.5,6.0),SS->getBallSpeedMax());
			}
		else 
			{
			soc=kickTo(VecPosition(52.5,-6.0),SS->getBallSpeedMax());
			}
		}


		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(x+10,y))==0)&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,3.5))==0))
		
			soc = getBestDribble();



		else if((posAgent.getX()>=31.0)&&((lenup>1.0)||(lendown>1.0)))
		{
		if(lenup<lendown)
			{
			soc=kickTo(VecPosition(52.5,6.0),SS->getBallSpeedMax());
			}
		else 
			{
			soc=kickTo(VecPosition(52.5,-6.0),SS->getBallSpeedMax());
			}
		}

		else 
		{
			if (posAgent.getY()<0.0)
			{
			   if (free==0)
				soc = getBestDribble();
			   else if (Free(-45)==0)
				soc = Dribble(-45);
			   else if (Free(45)==0)
				soc = Dribble(45);
			   else if (Free(90)==0)
				soc = Dribble(90);
			   else if (nrod==0)
				soc = directPass(pos10,PASS_NORMAL);
			   else if (nrd1==0)
				soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			   else if (Free(30)==0)
				soc = Dribble(30);
			   else if (Free(-30)==0)
				soc = Dribble(-30);
			   else 
			   {
				if(lenup<lendown)
				{
					soc=kickTo(VecPosition(52.5,6.0),SS->getBallSpeedMax());
				}
				else 
				{
					soc=kickTo(VecPosition(52.5,-6.0),SS->getBallSpeedMax());
				}
			   }
			   
			}
			else 
			{
			   if (free==0)
				soc = getBestDribble();
			   else if (Free(45)==0)
				soc = Dribble(45);
			   else if (Free(-45)==0)
				soc = Dribble(-45);
			   else if (Free(-90)==0)
				soc = Dribble(-90);
			   else if (nroy==0)
				soc = directPass(pos11,PASS_NORMAL);
			   else if (nry1==0)
				soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(4.0,25.0,POLAR)+pos11 ,NULL );
			   else if (Free(-30)==0)
				soc = Dribble(-30);
			   else if (Free(30)==0)
				soc = Dribble(30);
			   else 
			   {
				if(lenup<lendown)
				{
					soc=kickTo(VecPosition(52.5,6.0),SS->getBallSpeedMax());
				}
				else 
				{
					soc=kickTo(VecPosition(52.5,-6.0),SS->getBallSpeedMax());
				}
			   }
			   
			}
		}
		
		return soc;
}
//================================GETBESTMARK()===============================//

SoccerCommand Player::getBestMark()
{
SoccerCommand soc(CMD_ILLEGAL);

VecPosition   posAgent = WM->getAgentGlobalPosition();
VecPosition   posBall  = WM->getBallPos();

ObjectT opmark = WM->getClosestInSetTo( OBJECT_SET_OPPONENTS , posBall , NULL , -1.0  );	
VecPosition posmark = WM->getGlobalPosition(opmark);

ObjectT op=WM->getClosestRelativeInSet( OBJECT_SET_OPPONENTS , NULL  ); 
VecPosition posop = WM->getGlobalPosition(op);

ObjectT op1=WM->getClosestInSetTo   ( OBJECT_SET_OPPONENTS,VecPosition(-35.0,-20.0),NULL, -1.0 );
VecPosition posop1=WM->getGlobalPosition(op1);

ObjectT op2=WM->getClosestInSetTo   ( OBJECT_SET_OPPONENTS,VecPosition(-35.0,20.0),NULL, -1.0 );
VecPosition posop2=WM->getGlobalPosition(op2);

ObjectT op3=WM->getClosestInSetTo   ( OBJECT_SET_OPPONENTS,VecPosition(-50.0,0.0),NULL, -1.0 );
VecPosition posop3=WM->getGlobalPosition(op3);

ObjectT op4=WM->getClosestInSetTo   ( OBJECT_SET_OPPONENTS,VecPosition(0.0,0.0),NULL, -1.0 );
VecPosition posop4=WM->getGlobalPosition(op4);

ObjectT op5=WM->getClosestInSetTo   ( OBJECT_SET_OPPONENTS,posmark,NULL, -1.0 );
VecPosition posop5=WM->getGlobalPosition(op5);
 
	if( posop.getX()<=-33.0 ) 
	{
		soc = mark( op5 , 1.0 , MARK_GOAL);
	}

	else if( (posop.getX()>-33.0) && (posop.getX()<-10.0) )
	{
		if (posop.getY()< -18.0)
               		soc = mark( op5 , 1.5, MARK_BALL );
                
		else if (posop.getX()>18.0)
			soc = mark(op5,1.5,MARK_BALL);
		
		else 
			soc = mark(op5 ,1.5 ,MARK_BALL );	
	}

	else
	{
		soc = mark( op4 ,1.5 ,MARK_BALL );
	}

return soc;
}

//================================GETBESTMARK1()================================//
SoccerCommand Player::getBestMark1()
{
SoccerCommand soc(CMD_ILLEGAL);
VecPosition   posAgent = WM->getAgentGlobalPosition();
VecPosition   posBall  = WM->getBallPos();

ObjectT op=WM->getClosestRelativeInSet( OBJECT_SET_OPPONENTS , NULL  ); 
VecPosition posop = WM->getGlobalPosition(op);

soc = mark( op , 1.5 , MARK_GOAL);
return soc;
}

SoccerCommand Player::getBestPass()

{
  VecPosition   posBall = WM->getBallPos();
  SoccerCommand soc(CMD_ILLEGAL);
  VecPosition   posAgent = WM->getAgentGlobalPosition();
  VecPosition posGoal(PITCH_LENGTH/2.0,
              (-1 + 2*(WM->getCurrentCycle()%2)) * 0.4 * SS->getGoalWidth());

 ObjectT teammate=WM->getClosestInSetTo(OBJECT_SET_TEAMMATES,posAgent,NULL,-1.0);
  VecPosition posteammate=WM->getGlobalPosition   (   teammate  ) ; 
  int           iTmp;
  ObjectT C=WM->getClosestInSetTo(OBJECT_SET_TEAMMATES_NO_GOALIE , posAgent,NULL,-1.0);
  VecPosition VecC=WM->getGlobalPosition(C);
  ObjectT CC= WM->getSecondClosestRelativeInSet   ( OBJECT_SET_TEAMMATES_NO_GOALIE, NULL  );
  VecPosition VecCC = WM->getGlobalPosition(CC);

 

 double x=posAgent.getX();
 double y=posAgent.getY();
 
 VecPosition pos1=WM->getGlobalPosition(OBJECT_TEAMMATE_1);
 VecPosition pos2=WM->getGlobalPosition(OBJECT_TEAMMATE_2);
 VecPosition pos3=WM->getGlobalPosition(OBJECT_TEAMMATE_3);
 VecPosition pos4=WM->getGlobalPosition(OBJECT_TEAMMATE_4);
 VecPosition pos5=WM->getGlobalPosition(OBJECT_TEAMMATE_5);
 VecPosition pos6=WM->getGlobalPosition(OBJECT_TEAMMATE_6);
 VecPosition pos7=WM->getGlobalPosition(OBJECT_TEAMMATE_7);
 VecPosition pos8=WM->getGlobalPosition(OBJECT_TEAMMATE_8);
 VecPosition pos9=WM->getGlobalPosition(OBJECT_TEAMMATE_9);
 VecPosition pos10=WM->getGlobalPosition(OBJECT_TEAMMATE_10);
 VecPosition pos11=WM->getGlobalPosition(OBJECT_TEAMMATE_11);
 

 int free = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,VecPosition(x+8,y));
 int free1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,VecPosition(x,y+6));
 int free2 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,VecPosition(x,y-6)); 	    
 
 int nron = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
					,pos9);	
 int nrod = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
					,pos10);	
int nroy = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
					,pos11);	
 int nry = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,-45.0,POLAR)+pos11);
 int nry1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent
			,VecPosition(5.0,45.0,POLAR)+pos11);
 int nros = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6);	

 int nrd = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,45.0,POLAR)+pos10);
 int nrd1 = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
			,VecPosition(5.0,-45.0,POLAR)+pos10); 
       if (WM->isFreeKickThem( PM_ILLEGAL ) )
	{
		soc = getBestMark();
	}
	

	else
	{
	    if(posBall.getX()<=-35.0)
	    {
		if(WM->getRelativeDistance(WM->getClosestRelativeInSet(OBJECT_SET_OPPONENTS,NULL))>=7.0)
		{
		   if(posBall.getY()<0.0)
		   {
		      if((WM->isEmptySpace(WM->getAgentObjectType(),-75.0, 7.0)==true) and 
				(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,6.0))==0))
			{
			soc = Dribble(-75);//getBestDribble() ;
			}
		 
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
		    ,WM->getGlobalPosition(WM->getClosestRelativeInSet(OBJECT_SET_TEAMMATES_NO_GOALIE,NULL) ))==0) 
		      and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,
		        Circle(WM->getGlobalPosition(WM->getClosestRelativeInSet 
			  (OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)),3.0))==0) and 
			    (WM->getGlobalPosition(WM->getClosestRelativeInSet
			      (OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)).getX()>posAgent.getX()))
		   {
			soc = directPass(WM->getGlobalPosition(WM->getClosestRelativeInSet
						(OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)),PASS_NORMAL);
		   } 
		   }
		
		   else if (posBall.getY()>0.0)
		   {
			if((WM->isEmptySpace(WM->getAgentObjectType(),75.0,7.0)==true) and 
				(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,6.0))==0))
			{
				soc = Dribble(75);//getBestDribble() ;
			}
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
	             ,WM->getGlobalPosition(WM->getClosestRelativeInSet
			(OBJECT_SET_TEAMMATES_NO_GOALIE,NULL) ))==0)
			   and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  
			     Circle(WM->getGlobalPosition(WM->getClosestRelativeInSet( 
			       OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)),3.0))==0)and 
			         (WM->getGlobalPosition(WM->getClosestRelativeInSet
			            (OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)).getX()>posAgent.getX()))
		   {
			soc = directPass(WM->getGlobalPosition(WM->getClosestRelativeInSet
					(OBJECT_SET_TEAMMATES_NO_GOALIE,NULL)),PASS_NORMAL);
		   }
		   }

		}
		
		else 
			soc = clearBall(CLEAR_BALL_DEFENSIVE,SIDE_ILLEGAL,NULL);
		
	    }
	 
	    else if ((posBall.getX()>-35.0) and (posBall.getX()<0.0))	
	    {
		/*if((WM->getPlayerNumber()==10)||(WM->getPlayerNumber()==11))
		soc =getBestDribble();*/
		
		 if (posBall.getY()<0.0)
    	         {
		    if (WM->getPlayerNumber()!=9)
		    {
           	       if(nron==0)//if nine is free
				soc = directPass(pos9,PASS_NORMAL); //pass to nine
		    else if (free==0)
		    {
			      soc = getBestDribble();
		            if(nron==0)
			 	     soc = directPass(pos9,PASS_NORMAL);
			    else
			    	     soc = getBestDribble();
		    }


	        else if ((WM->isEmptySpace(WM->getAgentObjectType(),-75.0,6.0)==true) and
				(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS , Circle(posAgent,3.0))==0))
		{
			      soc=Dribble(-75);
		        if(nron==0)
			         soc = directPass(pos9,PASS_NORMAL);
			else
		              soc=Dribble(-75);
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),75.0,5.0)==true) and
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
			
		{
			     	    soc=Dribble(75);
			   if(nron==0)
			            soc = directPass(pos9,PASS_NORMAL);
			   else
			            soc=Dribble(75);
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),-90.0,4.0)==true) and 
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
			    	 soc=getBestDribble2();
  			if(nron==0)
			         soc = directPass(pos9,PASS_NORMAL);
			else
			         soc=getBestDribble2();
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),-45.0,5.0)==true) and
				(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS , Circle(posAgent,3.0))==0))
			
		{
			      soc=Dribble(-45);
		        if(nron==0)
			         soc = directPass(pos9,PASS_NORMAL);
			else
			         soc=Dribble(-45);
		}
 		else if ((WM->isEmptySpace(WM->getAgentObjectType(),90.0,5.0) ==true) and
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,4.0))==0))
		
  		{
			    //soc=dribble(90.0,DRIBBLE_SLOW);
  			if(nron==0)
			         soc = directPass(pos9,PASS_NORMAL);
			else
			         soc=getBestDribble1();
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent 
			,WM->getGlobalPosition(WM->getClosestInSetTo(OBJECT_SET_TEAMMATES,posAgent,NULL,-1.0)))==0)
			
			soc = directPass(WM->getGlobalPosition(WM->getClosestInSetTo
					(OBJECT_SET_TEAMMATES,posAgent,NULL,-1.0)),PASS_NORMAL);
		else
		{
			   soc = getBestDribble();
		}
		
		   }

		else if (WM->getPlayerNumber()==9)
		{
                	if (free<=1)
		                  soc = getBestDribble();
		else if((WM->isEmptySpace(WM->getAgentObjectType(),45.0,3.0))&&
			 (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
		       if((nry==0)&&(pos11.getX()<WM->getOffsideX(true)))
		       {
		         soc = throughPass(OBJECT_TEAMMATE_11,
				VecPosition(5.0,-45.0,POLAR)+pos11,NULL);
		       }
		       else		
		         soc = Dribble(45);
		}
		else if(WM->getNrInSetInCone(
			OBJECT_SET_OPPONENTS,tanDeg(10),posAgent,posAgent+VecPosition
				(8,WM->getRelAngleOpponentGoal(),POLAR))<=1)
		{
		       if((nry==0)&&(pos11.getX()<WM->getOffsideX(true)))
		       {
			  soc = throughPass(OBJECT_TEAMMATE_11,
				VecPosition(5.0,-45.0,POLAR)+pos11,NULL);
		       }
		       else
		          soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_SLOW);	
		}
		else if((nry<=1)&&(pos11.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_11, 
			VecPosition(5.0,-45.0,POLAR)+pos11,NULL);
		}
		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,
			pos8)==0))
		{
		   soc = directPass(pos8,PASS_NORMAL);
		}
		else if((WM->isEmptySpace(WM->getAgentObjectType(),-30.0,5.0)==true) && 
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
			soc = Dribble(-30);		
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
			,pos11)==0)
		{
		  soc = directPass(pos11,PASS_FAST);
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
			,WM->getGlobalPosition(OBJECT_TEAMMATE_5))==0)
		{
		       soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_5),PASS_NORMAL);
		}
		else 
		{
		       soc = getBestDribble();
		}
		
		}
	      }
	    else if(posAgent.getY()>0.0)
     	      {
		if (WM->getPlayerNumber()!=6)
		{
		 if(nros==0)//if nine is free
		       soc = directPass(pos6,PASS_NORMAL); //pass to nine
		 else if (free==0)
		 {
		   if(nros==0)
			    soc = directPass(pos6,PASS_NORMAL);
		   else
		            soc=getBestDribble();
		 }
		 else if((WM->isEmptySpace(WM->getAgentObjectType(),75.0,6.0)==true) and
			 (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		 {
		        if(nros==0)
			         soc = directPass(pos6,PASS_NORMAL);
			else
			         soc=Dribble(75);
		 }
		else if((WM->isEmptySpace(WM->getAgentObjectType(),-75.0, 5.0)==true) and
		       (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS , 		
				Circle(posAgent,3.0))==0)and(posAgent.getY()<27.0))
			
		{
		       if(nros==0)
			        soc = directPass(pos6,PASS_NORMAL);
		       else
			        soc=Dribble(-75);
		}
		else if((WM->isEmptySpace(WM->getAgentObjectType(),90.0,4.0)==true) and 
		        (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
  		       if(nros==0)
			        soc = directPass(pos6,PASS_NORMAL);
		       else
			        soc = getBestDribble1();
		}
		else if((WM->isEmptySpace(WM->getAgentObjectType(),-90.0, 5.0)==true) and
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,4.0))==0))
		{
  			if(nros==0)
			         soc = directPass(pos6,PASS_NORMAL);
			else
			         soc = getBestDribble2();
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
			,WM->getGlobalPosition(WM->getClosestInSetTo
				(OBJECT_SET_TEAMMATES,posAgent,NULL, -1.0  ))  )==0)
			
			soc = directPass(WM->getGlobalPosition(WM->getClosestInSetTo
					(OBJECT_SET_TEAMMATES,posAgent,NULL,-1.0)),PASS_NORMAL);
		
		else
		{
			soc = getBestDribble();
		}
		 
		 
		}
	        else if(WM->getPlayerNumber()==6)
		{
		       if (free<=1)
		                 soc = getBestDribble();

		else if((WM->isEmptySpace(WM->getAgentObjectType(),-45.0,3.0)) &&
			 (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
			if((nrd==0) && (pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10,
				 VecPosition(5.0,45.0,POLAR)+pos10,NULL);
			}
			else		
		 	  soc = Dribble(-45);	 
		}
		else if(WM->getNrInSetInCone(
			OBJECT_SET_OPPONENTS,tanDeg(10),posAgent,posAgent+VecPosition
				(8,WM->getRelAngleOpponentGoal(),POLAR))==0)
		{
		       if (( nry==0)&&( pos10.getX()<WM->getOffsideX(true)))
		       {
			  soc = throughPass(OBJECT_TEAMMATE_10,
				 VecPosition(5.0,45.0,POLAR)+pos10,NULL);
		       }
		       else
			  soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_SLOW);
		}
		else if((nrd==0) && (pos10.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_10,
				VecPosition(5.0,45.0,POLAR)+pos10,NULL);
		}
		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent
			,pos7)==0))
		{
		   soc = directPass(pos7,PASS_NORMAL);
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),30.0, 5.0)==true) && 
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(posAgent,3.0))==0))
		{
			soc = Dribble(30);
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
		       ,WM->getGlobalPosition( OBJECT_TEAMMATE_10 )  )==0)
		{
		      soc = directPass(WM->getGlobalPosition( OBJECT_TEAMMATE_10 ),PASS_FAST);
		}
		else if(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent
			,WM->getGlobalPosition( OBJECT_TEAMMATE_2 )  )==0) 
		{
		      soc = directPass(WM->getGlobalPosition( OBJECT_TEAMMATE_2 ),PASS_NORMAL);
		}
		else
		{
		      soc = getBestDribble();
		}
		}
	      }
	}
		
 
  else if ((posBall.getX()>0.0) && (posBall.getX()<33.0))
    {   
		
	
	
	 if (posBall.getY()<=0.0)
	{
	
	if (WM->getPlayerNumber()==9)
	{
        
	 if ((WM->getAgentStamina().getStamina()>1650) && (free==0)&&(Free(20)==0))
	 {
	       

		if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos11)==0)&&(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(55),pos11,
		VecPosition(30,0.0,POLAR)+pos11)==0)&&(pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,5.0))==0))
		soc = directPass(pos11,PASS_NORMAL);
	
		else 
		soc = getBestDribble();
	  }

	 else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,
		VecPosition(5.0,-45.0,POLAR)+pos11)==0) and 
			(pos11.getX()<WM->getOffsideX(true)))
	 {
	        soc = throughPass(OBJECT_TEAMMATE_11 , 
			VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
	
	 }
	   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent
		,pos11)==0) and 
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,
				Circle(pos11,3.0))==0))
	  {
 	     soc = directPass(pos11,PASS_NORMAL);
	  }
	
	else if ((WM->getAgentStamina().getStamina()>1500) && (free<=1))
	 {
	        if (( nry<=1)&&( pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,4.0))==0))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
			}	
		else
		soc = getBestDribble();
	  }


	  else if          ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.5))==0)&&(WM->getAgentStamina().getStamina()>1500))
	 {
		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	        if (( nry<=1)&&((WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
			}	
		else
		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	  }
	 else if          ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(8.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,3.0))==0)&&(WM->getAgentStamina().getStamina()>1500))
	 {
	
		if (( nry<=1)&&( pos11.getX()<WM->getOffsideX(true))&&((WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
			}	 
		else
		  soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_SLOW);
	}
	
 	 else if ((WM->getAgentStamina().getStamina()>1500) && (Free(45)==0))
	 {
	        if (( nry<=1)&&( pos11.getX()<WM->getOffsideX(true)) && 
			(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  
				Circle(pos11,4.0))==0))
		{
		  soc = throughPass(OBJECT_TEAMMATE_11 , 
			VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
		}	
		else
		soc =Dribble(45);
	  }
	  
	else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(4.0,-45.0,POLAR)+pos11)==0)&&( pos11.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
		}
	
	  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(4.0,90.0,POLAR)+pos11)<=1)&&( pos11.getX()<WM->getOffsideX(true)))
	{
	  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(pos11.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos11 ,NULL );
	
	}  
	else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(3.0,-90.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true)))
	{
	  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-65.0,POLAR)+pos11 ,NULL );
	
	}  
	else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
	  {
 	     soc = directPass(pos11,PASS_NORMAL);
	  } 
	  else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.5))==0))
	  {
	     soc = directPass(pos10,PASS_NORMAL);
	  }
	  else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos8)==0)&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,3.0))==0)) 
	  {
	     soc = directPass(pos8,PASS_NORMAL);
	  }
	  else 
		soc = getBestDribble();
	}
	   else if ( (WM->getPlayerNumber()==8)||(WM->getPlayerNumber()==7) )
	   {	
		
// 		  if ((WM->getAgentStamina().getStamina()>1600) && (free==0))
// 	        {
// 	      	  	if (( nry==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
// 			{
// 				  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}	
// 			else if (nron == 0)
// 				soc = directPass(pos9,PASS_NORMAL);
// 			else
// 				soc = getBestDribble();
// 	        }	
// 
// 		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
// 		{
// 	  
// 	
// 		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}
// 	
// 		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
// 			} 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		else
// 		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
// 	        }
// 	
// 	    	 else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(7.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
// 	        {
// 	        
// 		
// 		if (( nry<=1)&&( pos11.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}
// 	
// 		else if (( nrd<=1)&&( pos10.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
// 			} 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		else
// 		soc = getBestDribble();
// 	        }
// 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		
// 		else if ( (Free(-75)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
// 		{
// 	  
// 	
// 		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}
// 	
// 		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
// 			} 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		else
// 		soc = Dribble(-75);
// 	        }
// 
// 		else if ( (Free(-45)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
// 		{
// 	  
// 	
// 		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}
// 	
// 		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
// 			} 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		else
// 		soc = Dribble(-45);
// 	        }
// 
// 		else if ( (Free(75)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
// 		{
// 	  
// 	
// 		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 			}
// 	
// 		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
// 			{
// 			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
// 			} 
// 		else if (nron == 0)
// 			soc = directPass(pos9,PASS_NORMAL);
// 		else
// 		soc = Dribble(75);
// 	        }
// 
// 
// 
// 		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos11)<=1)and( pos11.getX()<WM->getOffsideX(true)))
// 		{
// 	 	 soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(pos11.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos11 ,NULL );
// 	
// 		}  
// 		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos11)<=1)and( pos11.getX()<WM->getOffsideX(true)))
// 		{
// 		  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
// 		}
// 		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,1.5))==0))
// 	        {
//  	               soc = directPass(pos11,PASS_NORMAL);
// 	        } 
// 
// 		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(27.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,1.5))==0))
// 	        {
//  	               soc = directPass(pos10,PASS_NORMAL);
// 	        } 
// 		else if
//                ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,-78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
// 	        {
// 	            soc = Dribble(-78);// soc = getBestDribble();
// 	        }
// 		else if
//                ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
// 	        {
// 	            soc = Dribble(78);// soc = get
// 
// 	        }
// 		else
// 			soc = getBestDribble();

	 if ((WM->getAgentStamina().getStamina()>1600) && (free==0))
	        {
	      	 
			soc = getBestDribble();
		 	if (( nry==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{
// 				soc1 = turnBodyToPoint(pos11,1);
// 		 		soc2 = alignNeckWithBody();
				  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
			
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{/*
				soc1 = turnBodyToPoint(pos11,1);
		 		soc2 = alignNeckWithBody();*/
				  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,25.0,POLAR)+pos11 ,NULL );
			}
	
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{
// 				soc1 = turnBodyToPoint(pos10,0);
// 		 		soc2 = alignNeckWithBody();
				  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-25.0,POLAR)+pos10 ,NULL );
			}	

			else if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{
// 				soc1 = turnBodyToPoint(pos10,0);
// 		 		soc2 = alignNeckWithBody();
				  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,25.0,POLAR)+pos10 ,NULL );
			}		

			else if (nron == 0)
			{
// 				soc1 = turnBodyToPoint(pos9,0);
// 		  		soc2 = alignNeckWithBody();
				soc = directPass(pos9,PASS_NORMAL);
			}
			
			else
				soc = getBestDribble();
	        }	

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 		  	soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
	
		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			soc1 = turnBodyToPoint(pos10,0);
// 		  	soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			}
		
		else if (( nry1==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 		  	soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,25.0,POLAR)+pos11 ,NULL );
			} 
		else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{
// 				soc1 = turnBodyToPoint(pos10,0);
// 		 		soc2 = alignNeckWithBody();
				  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-25.0,POLAR)+pos10 ,NULL );
			}
		else if (nron == 0)
		{
// 			soc1 = turnBodyToPoint(pos11,1);
// 		  	soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		
		else
		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	        }
	
	    	 else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(7.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
	        {
	        
		
		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
// 			soc1 = turnBodyToPoint(pos11,1);
// 		 	soc2 = alignNeckWithBody(); 
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
	


		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos10,0);
// 		 	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			} 
		
		
		else if (( nry1==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos10,0);
// 		 	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(4.0,-25.0,POLAR)+pos11 ,NULL );
			} 
		else if (( nrd1==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos10,0);
// 		 	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			} 
		
		else if (nron == 0)
		{
// 			soc1 = turnBodyToPoint(pos9,0);
// 			soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		else
		soc = getBestDribble();
	        }

		else if (nron == 0)
		{	
// 			soc1 = turnBodyToPoint(pos9,0);
// 		  	soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		else if ( (Free(-75)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos11,1);
// 		  	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
	
		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos10,0);
// 		  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			} 
		else if (nron == 0)
		{
// 			soc1 = turnBodyToPoint(pos9,0);
// 		  	soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		else
		soc = Dribble(-75);
	        }

		else if ( (Free(-45)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos11,1);
// 		  	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
	
		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			 soc1 = turnBodyToPoint(pos10,0);
// 		  	 soc2 = alignNeckWithBody();
			 soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			} 
		else if (nron == 0)
		{
// 			soc1 = turnBodyToPoint(pos9,0);
// 		 	soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		else
		soc = Dribble(-45);
	        }

		else if ( (Free(75)==0) && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos11,1);
// 		 	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
	
		else if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
// 			  soc1 = turnBodyToPoint(pos10,0);
// 		 	  soc2 = alignNeckWithBody();
			  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(4.0,-25.0,POLAR)+pos10 ,NULL );
			} 
		else if (nron == 0)
		{
// 			soc1 = turnBodyToPoint(pos9,0);
// 		 	soc2 = alignNeckWithBody();
			soc = directPass(pos9,PASS_NORMAL);
		}
		else
		soc = Dribble(75);
	        }



		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos11)<=1)and( pos11.getX()<WM->getOffsideX(true)))
		{
// 		soc1 = turnBodyToPoint(pos11,1);
// 		  soc2 = alignNeckWithBody();
	 	 soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(pos11.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos11 ,NULL );
	
		}  
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos11)<=1)and( pos11.getX()<WM->getOffsideX(true)))
		{
// 		  soc1 = turnBodyToPoint(pos11,1);
// 		  soc2 = alignNeckWithBody();
		  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,1.5))==0))
	        {
// 			soc1 = turnBodyToPoint(pos11,1);
// 		        soc2 = alignNeckWithBody();
 	                soc = directPass(pos11,PASS_NORMAL);
	        } 

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(27.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,1.5))==0))
	        {
// 			soc1 = turnBodyToPoint(pos10,0);
// 		  	soc2 = alignNeckWithBody();
 	                soc = directPass(pos10,PASS_NORMAL);
	        } 
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,-78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(-78);// soc = getBestDribble();
	        }
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(78);// soc = get

	        }
		else
			soc = getBestDribble();



	   }
	    else if (WM->getPlayerNumber()==11)
	    {
		 if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,pos10)==0)and( pos10.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)))
		{
			 soc = directPass(pos10,PASS_NORMAL);	
		}
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(2.5,-15.0,POLAR)+pos10 ,NULL );
		}

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,VecPosition(8.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)&&((WM->getAgentStamina().getStamina()>1650)))
			soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_SLOW);

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(50.0),posAgent,VecPosition(8.0,0.0,POLAR)+posAgent)==0)&&((WM->getAgentStamina().getStamina()>1600)))
		soc = getBestDribble();


		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.0))==0)and(pos10.getX()>pos11.getX()-5))
	           {
 	               soc = directPass(pos10,PASS_NORMAL);
	           } 

		 else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(2.5,-10.0,POLAR)+pos10 ,NULL );
		}		

		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(6.0,0.0,POLAR)+posAgent)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.5))<=1)and ((WM->getAgentStamina().getStamina()>1600)))		
	           {
	              
			 soc = getBestDribble();
	           }

		  else  if ((free2==0)&&((WM->getAgentStamina().getStamina()>1500)))
			{
			soc = getBestDribble2() ;
			if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,25.0,POLAR)+pos10 ,NULL );
			}
			else if (free==0)
				soc = getBestDribble();
			else
			soc = getBestDribble2() ;
			}
		
		 else  if ((free1==0)&&((WM->getAgentStamina().getStamina()>1500)))
			{
			soc = getBestDribble1() ;
			if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,25.0,POLAR)+pos10 ,NULL );
			}
			else if (free==0)
				soc = getBestDribble();
			else
			soc = getBestDribble1() ;
			}

		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_10, VecPosition(pos10.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/4.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos10 ,NULL );
		}

		 else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(2.5,-25.0,POLAR)+pos10 ,NULL );
		}		

		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.0))==0))
	           {
 	               soc = directPass(pos10,PASS_NORMAL);
	           } 

		     else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,2.0,POLAR)+pos9)==0)and( pos9.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_9 , VecPosition(5.0,10.0,POLAR)+pos9 ,NULL );
		}


		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos9)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos9,2.0))==0))
		   {
 			soc = directPass(pos9,PASS_NORMAL);
		   }
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos8)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,2.0))==0))
		   {
 			soc = directPass(pos8,PASS_NORMAL);
		   }
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos7)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,2.0))==0))
		   {
 			soc = directPass(pos7,PASS_NORMAL);
		   }
		   else 
		   {
 			soc = getBestDribble();
		   }

    }
	        else if (WM->getPlayerNumber()==10)
		{
		    if
                 ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,0.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	         {
	            soc = getBestDribble();
	         }
		   
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(32.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,2.0))==0))
	           {
 	               soc = directPass(pos11,PASS_NORMAL);
	           } 

		   else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,2.0,POLAR)+pos9)==0)and( pos9.getX()<WM->getOffsideX(true)))
		    {
		  	soc = throughPass(OBJECT_TEAMMATE_9 , VecPosition(5.0,10.0,POLAR)+pos9 ,NULL );
		    }			

		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos9)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos9,2.0))==0))
		   {
 			soc = directPass(pos9,PASS_NORMAL);
		   }
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos8)==0) and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(pos8,2.0))==0))
		   {
 			soc = directPass(pos8,PASS_NORMAL);
		   }
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos7)==0) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,2.0))==0))
		   {
 			soc = directPass(pos7,PASS_NORMAL);
		   }
		   else 
		   {
 			soc = getBestDribble();
		   }

		}
		else {

		if (free==0)
			soc =getBestDribble();
		
		   else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,2.0,POLAR)+pos9)==0)and( pos9.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_9 , VecPosition(2.0,10.0,POLAR)+pos9 ,NULL );
		}


		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos9)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos9,2.0))==0))
		   {
 			soc = directPass(pos9,PASS_NORMAL);
		   }

		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos7)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,2.0))==0))
		{
			soc = directPass(pos7,PASS_NORMAL);
		}
		
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,-45.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(3.0,-45.0,POLAR)+pos11 ,NULL );
		}

		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos8)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,2.0))==0))
		{
			soc = directPass(pos8,PASS_NORMAL);
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos9)==0) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos9,2.0))==0))
		{
			soc = directPass(pos9,PASS_NORMAL);
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos6,2.0))==0))
		{
			soc = directPass(pos6,PASS_NORMAL);
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),0.0, 6.0)==true )and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=getBestDribble();		
		  }
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),-80.0, 6.0) ==true) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=Dribble(-80);		
		  }
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),80.0, 6.0) == true )and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=Dribble(80);		
		  }
		else
			soc =getBestDribble();
		}
	}
	//start from here copy paste
	if (posBall.getY()>=0.0)
	{

	 if (WM->getPlayerNumber()==6)
	{
	 if ((WM->getAgentStamina().getStamina()>1650) && (free==0)&&(Free(-15)==0))
	 {
		if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos10)==0)&&(WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(55),pos10,
		VecPosition(30,0.0,POLAR)+pos10)==0)&&(pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,5.0))==0))
		soc = directPass(pos10,PASS_NORMAL);

		else
	  		soc = getBestDribble();
	  }

	else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(5.0,45.0,POLAR)+pos10)==0)and( pos11.getX()<WM->getOffsideX(true))&&(pos10.getX()>10.0))
	{
	  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
	
	} 

	   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
	  {
 	     soc = directPass(pos10,PASS_NORMAL);
	  } 
	
	 else if ((WM->getAgentStamina().getStamina()>1500) && (free==0))
	 {
	        if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,4.0))==0))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}	
		else
		soc = getBestDribble();
	  }


	  else if          ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.5))==0)&&(WM->getAgentStamina().getStamina()>1500))
	 {
	        if (( nrd==0)&&((WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}	
		else
		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	  }
	 else if          ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(8.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,3.0))==0)&&(WM->getAgentStamina().getStamina()>1500))
	 {
		
		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true))&&((WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}	 
		else
		  soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_SLOW);
	}
	
	 else if ((WM->getAgentStamina().getStamina()>1500) && (Free(-45)==0))
	 {

		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,4.0))==0))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}	
		else
		soc = Dribble(-45);
	  }
	  
	else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)==0)&&( pos10.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
		}
	
	  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)==0)&&( pos10.getX()<WM->getOffsideX(true)))
	{
	  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(pos10.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos10 ,NULL );
	
	}  
	else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true)))
	{
	  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
	
	}  
	else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
	  {
 	     soc = directPass(pos10,PASS_NORMAL);
	  } 
	  else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,2.5))==0))
	  {
	     soc = directPass(pos11,PASS_NORMAL);
	  }
	  else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos7)==0)&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,3.0))==0)) 
	  {
	     soc = directPass(pos7,PASS_NORMAL);
	  }
	  else 
		soc = getBestDribble();
	}
	else if ( (WM->getPlayerNumber()==7)||(WM->getPlayerNumber()==8) )
	   {
	   
		/*
		  if ((WM->getAgentStamina().getStamina()>1600) && (free==0))
	        {
	      	  	if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{
				  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}	
			else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
			else
				soc = getBestDribble();
	        }	

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
	
		else if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11, VecPosition(4.0,45.0,POLAR)+pos11 ,NULL );
			} 
		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
		else
		soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	        }
	
	    	 else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(7.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
	        {
	        
		
		if (( nrd<=1)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
	
		else if (( nry<=1)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(4.0,45.0,POLAR)+pos11 ,NULL );
			} 
		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
		else
		soc = getBestDribble();
	        }

		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);

		else if ((Free(75)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
	
		else if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11, VecPosition(4.0,45.0,POLAR)+pos11 ,NULL );
			} 
		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
		else
		soc = Dribble(75);
	        }
	
		else if ((Free(45)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
	
		else if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11, VecPosition(4.0,45.0,POLAR)+pos11 ,NULL );
			} 
		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
		else
		soc = Dribble(45);
	        }

		else if ((Free(-75)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( pos10.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
	
		else if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11, VecPosition(4.0,45.0,POLAR)+pos11 ,NULL );
			} 
		else if (nros == 0)
			soc = directPass(pos6,PASS_NORMAL);
		else
		soc = Dribble(-75);
	        }

		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)<=1)and( pos10.getX()<WM->getOffsideX(true)))
		{
	 	 soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(pos10.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos10 ,NULL );
	
		}  
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)<=1)and( pos10.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos10)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,1.5))==0))
	        {
 	               soc = directPass(pos10,PASS_NORMAL);
	        } 

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(27.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,1.5))==0))
	        {
 	               soc = directPass(pos11,PASS_NORMAL);
	        } 
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(78);// soc = getBestDribble();
	        }
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,-78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(-78);// soc = get

	        }
		else
			soc = getBestDribble();*/

		  if ((WM->getAgentStamina().getStamina()>1600) && (free==0))
	        {
	      	  	if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
				
		
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}
			else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}
			else
				soc = getBestDribble();
	        }	

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(13.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
				
			
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}
			else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}
		
			else
				soc = dribble(WM->getRelAngleOpponentGoal(),DRIBBLE_FAST);
	        }
	
	    	 else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,VecPosition(7.0,WM->getRelAngleOpponentGoal(),POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
	        {
	        
		
		if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
				
			
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}
		else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}

		else
		soc = getBestDribble();
	        }

		else if (nros == 0)
		{
// 			soc1 = turnBodyToPoint(pos6,0);
// 			soc2 = alignNeckWithBody();
			soc = directPass(pos6,PASS_NORMAL);
		}
		else if ((Free(75)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
			
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}

				
			else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}

		else
			soc = Dribble(75);
	        }
	
		else if ((Free(45)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
		
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{
/*
			soc1 = turnBodyToPoint(pos11,1);
			soc2 = alignNeckWithBody();*/
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}
					
			else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}

		else
			soc = Dribble(45);
	        }

		else if ((Free(-75)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.0))==0))		
		{
	  
	
		if (( nrd==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
			}
			else if (( nrd1==0)&&( 	pos10.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,-45.0,POLAR)+pos10 ,NULL );
			}
				
			
			else if (( nry1==0)&&( 	pos11.getX()<WM->getOffsideX(true))&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
			{

// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
			soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,45.0,POLAR)+pos11 ,NULL );
			}

		else if (nros == 0)
			{	
// 				soc1 = turnBodyToPoint(pos6,0);
// 				soc2 = alignNeckWithBody();
				soc = directPass(pos6,PASS_NORMAL);
			}

		else
		soc = Dribble(-75);
	        }

		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)<=1)and( pos10.getX()<WM->getOffsideX(true)))
		{
// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
	 	 soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(pos10.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos10 ,NULL );
	
		}  
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)<=1)and( pos10.getX()<WM->getOffsideX(true)))
		{
// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
		  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(5.0,45.0,POLAR)+pos10 ,NULL );
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(25.0),posAgent,pos10)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,1.5))==0))
	        {
// 			soc1 = turnBodyToPoint(pos10,0);
// 			soc2 = alignNeckWithBody();
 	               soc = directPass(pos10,PASS_NORMAL);
	        } 

		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(27.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,1.5))==0))
	        {
// 			soc1 = turnBodyToPoint(pos11,1);
// 			soc2 = alignNeckWithBody();
 	               soc = directPass(pos11,PASS_NORMAL);
	        } 
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(78);// soc = getBestDribble();
	        }
		else if
               ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,-78.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	        {
	            soc = Dribble(-78);// soc = get

	        }
		else
			soc = getBestDribble();


	   }	
		else if (WM->getPlayerNumber()==10)
	    {

		if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)))
		{
		 soc = directPass(pos11,PASS_NORMAL);
		}		

		if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(5.0,65.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(2.5,15.0,POLAR)+pos11 ,NULL );
		}	


		  else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(10.0),posAgent,VecPosition(6.0,0.0,POLAR)+posAgent)==0)&&((WM->getAgentStamina().getStamina()>1600)))
		soc = getBestDribble();


		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos11)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,2.0))==0)and(pos11.getX()>pos10.getX()-5))
	           {
 	               soc = directPass(pos11,PASS_NORMAL);
	           } 

		
		 else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true))and (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos11,VecPosition(8.0,0.0,POLAR)+pos11)<WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(89.0),pos10,VecPosition(8.0,0.0,POLAR)+pos10)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(2.5,15.0,POLAR)+pos11 ,NULL );
		}		

		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(6.0,0.0,POLAR)+posAgent)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,2.5))<=1))		
	           {
	              
			 soc = getBestDribble();
	           }

		 else  if ((free1==0)&&((WM->getAgentStamina().getStamina()>1500)))
			{
			soc = getBestDribble1() ;
			if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
			else if (free==0)
				soc = getBestDribble();
			else
			soc = getBestDribble1() ;
			}
		
		 else  if ((free2==0)&&((WM->getAgentStamina().getStamina()>1500)))
			{
			soc = getBestDribble2() ;
			if (( nry==0)&&( pos11.getX()<WM->getOffsideX(true)))
			{
			  soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(5.0,-25.0,POLAR)+pos11 ,NULL );
			}
			else if (free==0)
				soc = getBestDribble();
			else
			soc = getBestDribble2() ;
			}

		  else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_11, VecPosition(pos11.getDistanceTo(WM->getGlobalPosition(OBJECT_OPPONENT_GOALIE))/3.0,WM->getRelAngleOpponentGoal( ),POLAR)+pos11 ,NULL );
		}

		 else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos11)==0)and( pos11.getX()<WM->getOffsideX(true)))
		{
		 soc = throughPass(OBJECT_TEAMMATE_11 , VecPosition(2.5,45.0,POLAR)+pos11 ,NULL );
		}
		     else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,-2.0,POLAR)+pos6)==0)and( pos6.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_6 , VecPosition(5.0,-10.0,POLAR)+pos6 ,NULL );
		}


		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos6)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos6,2.0))==0))
		   {
 			soc = directPass(pos6,PASS_NORMAL);
		   }
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos7)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,2.0))==0))
		   {
 			soc = directPass(pos7,PASS_NORMAL);
		   }
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos8)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,2.0))==0))
		   {
 			soc = directPass(pos8,PASS_NORMAL);
		   }
		   else 
		   {
 			soc = getBestDribble();
		   }

    }
	        else if (WM->getPlayerNumber()==11)
		{
		      if
                 ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(22.0),posAgent,VecPosition(8.0,0.0,POLAR)+posAgent)==0)  && (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
	         {
	            soc = getBestDribble();
	         }
		   
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(32.0),posAgent,pos10)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.0))==0))
	           {
 	               soc = directPass(pos10,PASS_NORMAL);
	           } 

		   else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,-2.0,POLAR)+pos6)==0)and( pos6.getX()<WM->getOffsideX(true)))
		    {
		  	soc = throughPass(OBJECT_TEAMMATE_6 , VecPosition(5.0,-10.0,POLAR)+pos6 ,NULL );
		    }			

		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos6,2.0))==0))
		   {
 			soc = directPass(pos6,PASS_NORMAL);
		   }
		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos7)==0) and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS,Circle(pos7,2.0))==0))
		   {
 			soc = directPass(pos7,PASS_NORMAL);
		   }
		   else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos8)==0) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,2.0))==0))
		   {
 			soc = directPass(pos8,PASS_NORMAL);
		   }
		   else 
		   {
 			soc = getBestDribble();
		   }

		}
		else

		 {
		    if (free==0)
			soc =getBestDribble();
		
		   else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(6.0,-2.0,POLAR)+pos6)==0)and( pos6.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_6 , VecPosition(2.0,-10.0,POLAR)+pos6 ,NULL );
		}


		   else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6)==0)  and(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos6,2.0))==0))
		   {
 			soc = directPass(pos6,PASS_NORMAL);
		   }

		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos8)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos8,2.0))==0))
		{
			soc = directPass(pos8,PASS_NORMAL);
		}
		
		else if ( (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0),posAgent,VecPosition(3.0,45.0,POLAR)+pos10)==0)and( pos10.getX()<WM->getOffsideX(true)))
		{
		  soc = throughPass(OBJECT_TEAMMATE_10 , VecPosition(3.0,45.0,POLAR)+pos10 ,NULL );
		}

		else if((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos7)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos7,2.0))==0))
		{
			soc = directPass(pos7,PASS_NORMAL);
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos6)==0) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos6,2.0))==0))
		{
			soc = directPass(pos6,PASS_NORMAL);
		}
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0),posAgent,pos9)==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos9,2.0))==0))
		{
			soc = directPass(pos9,PASS_NORMAL);
		}
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),0.0, 6.0)==true )and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=getBestDribble();	
		  }
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),80.0, 6.0) ==true) and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=Dribble(80);		
		  }
		else if ((WM->isEmptySpace(WM->getAgentObjectType(),-80.0, 6.0) == true )and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,1.5))==0))
		  {
			soc=Dribble(-80);		
		  }
		else
			soc =getBestDribble();

		}
	 	
	}
    
	}

  else if (posBall.getX()>=33.0)
  {
		
     if (posBall.getY()<-20.0)
     {	
	if (WM->getPlayerNumber()!=11)
	{

	if ((free1==0)&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,4.0))==0))
			{
			soc = getBestDribble1() ;
			} 

	else if ((nroy==0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,3.0))==0))
	soc = directPass(pos11,PASS_NORMAL);
	
	else  if (free1==0)
			{
			soc = getBestDribble1() ;
			} 
	else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)<=0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.5))==0))
			soc = directPass(pos10,PASS_NORMAL);
	
       }
	else 
	{
		if (free1==0)
			soc = getBestDribble1();
		
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)<=0)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,2.5))==0))
			soc = directPass(pos10,PASS_NORMAL);
		else if (nron=0)
			soc = directPass(pos9,PASS_NORMAL);
		else 
			soc = getBestKick();	
	}
	}
	else if (posBall.getY()>20.0)
     {

	if (WM->getPlayerNumber()!=10)
	{
	
	  if ((free2==0)&&(WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(posAgent,4.0))==0))
			{
			soc = getBestDribble2() ;
			} 

         else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos10)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos10,3.0))==0))
	soc = directPass(pos10,PASS_NORMAL);
	
	else  if (free2==0)
			{
			soc = getBestDribble2() ;
			} 
	else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos11)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,2.5))==0))
			soc = throughPass(OBJECT_TEAMMATE_11,
			VecPosition(3.0,-45.0,POLAR)+pos11,NULL);

	
	}
	else 
	{
		if (free2==0)
			soc = getBestDribble2();
		
		else if ((WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0),posAgent,pos11)<=1)  and (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS ,  Circle(pos11,2.5))==0))
			soc = throughPass(OBJECT_TEAMMATE_11,
			VecPosition(3.0,-45.0,POLAR)+pos11,NULL);
		else if (nros=0)
			soc = directPass(pos6,PASS_NORMAL);
		else 
			soc = getBestKick(); 

	}
     }
	else if((posBall.getY()<=20.0)&&(posBall.getY()>=-20.0)) 
	{
	    soc =getBestKick();  

	}
}
}
return soc;
}



SoccerCommand Player::deMeer5(  )
{
  Stamina sta;
  SoccerCommand soc(CMD_ILLEGAL);
  VecPosition   posAgent = WM->getAgentGlobalPosition();
  VecPosition   posBall  = WM->getBallPos();
  ObjectT teammate=WM->getClosestInSetTo(OBJECT_SET_TEAMMATES,posAgent,NULL,-1.0);
  VecPosition posteammate=WM->getGlobalPosition   (   teammate  ) ; 
  int           iTmp;
  ObjectT C=WM->getClosestInSetTo(OBJECT_SET_TEAMMATES_NO_GOALIE , posAgent,NULL,-1.0);
  VecPosition VecC=WM->getGlobalPosition(C);
  ObjectT CC= WM->getSecondClosestRelativeInSet   ( OBJECT_SET_TEAMMATES_NO_GOALIE, NULL  );
  VecPosition VecCC = WM->getGlobalPosition(CC);

  if( WM->isBeforeKickOff( ) )
  {
    if( WM->isKickOffUs( ) && WM->getPlayerNumber() == 11 ) // 11 takes kick
    {
      if( WM->isBallKickable() )
      {
        VecPosition posGoal( PITCH_LENGTH/2.0,
                             (-1 + 2*(WM->getCurrentCycle()%2)) * 
                             0.4 * SS->getGoalWidth() );
        soc=directPass(posteammate,PASS_NORMAL); // kick maximal
	cout<<endl<<endl<<endl<<";;;;;;;;;;;;;;;;;;;;;;;;;;;;;:"<<endl;
	
        Log.log( 100, "take kick off" );
      }
      else
      {
        soc = intercept( false );  
        Log.log( 100, "move to ball to take kick-off" );
      }  
      ACT->putCommandInQueue( soc );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      return soc;
    }  
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
    {
      formations->setFormation( FT_DEFENSIVE );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos( WM->getStrategicPosition() ));
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc=turnBodyToPoint( VecPosition( 0, 0 ), 0 ) );
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
  }
  else
  {
// 	if (isThem()==true )  
//     {
      formations->setFormation(FT_DEFENSIVE );
      soc.commandType = CMD_ILLEGAL;
//     }
// 	else 
//     {
//       formations->setFormation(FT_343_ATTACKING);
//       soc.commandType = CMD_ILLEGAL;
//     }

    if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
    {
      ACT->putCommandInQueue( soc = searchBall() );   // if ball pos unknown
      ACT->putCommandInQueue( alignNeckWithBody( ) ); // search for it
    }
    else if( WM->isBallKickable())                    // if kickable
    {
      VecPosition posGoal( PITCH_LENGTH/2.0,
              (-1 + 2*(WM->getCurrentCycle()%2)) * 0.4 * SS->getGoalWidth() );

	if(WM->getPlayMode()==PM_PLAY_ON)
	{
		soc = getBestPass();
	} 

 	else if (WM->isFreeKickUs()==true)
 	{
 	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
 		soc=directPass(VecC,PASS_FAST);
 	else
 		soc = directPass(VecCC,PASS_FAST);
 	}

	else if (WM->isKickInUs()==true)
	{
	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
		soc=directPass(VecC,PASS_FAST);
	else
		soc = directPass(VecCC,PASS_FAST);
	}

	else if (WM->isCornerKickUs()==true)
	{
	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
		soc=directPass(VecC,PASS_FAST);
	else
		soc = directPass(VecCC,PASS_FAST);
	} 
// 
// 	else if (WM->isFreeKickFaultUs()==true)
// 	{
// 	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
// 		soc=directPass(VecC,PASS_FAST);
// 	else
// 		soc = directPass(VecCC,PASS_FAST);
// 	}
// 
// 	else if (WM->isOffsideUs()==true)
// 	{
// 	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
// 		soc=directPass(VecC,PASS_FAST);
// 	else
// 		soc = directPass(VecCC,PASS_FAST);
// 	}
// 
// 	else if (WM->isDeadBallUs()==true)
// 	{
// 	if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,VecC)==0)
// 		soc=directPass(VecC,PASS_FAST);
// 	else
// 		soc = directPass(VecCC,PASS_FAST);
// 	}
	else if (WM->isGoalKickUs()==true)
	{
		if (posAgent.getY()<0.0)
	{
		
	
		if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_5)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_5),PASS_FAST);
	
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_9)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_9),PASS_FAST);


		

		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_4)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_4),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_3)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_3),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_2)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_2),PASS_FAST);
		else 
			soc=clearBall(CLEAR_BALL_DEFENSIVE,SIDE_LEFT,NULL );
	}
	else if (posAgent.getY()>=0.0)
	
	{

		
		if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_2)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_2),PASS_FAST);
		
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_6)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_6),PASS_FAST);

		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_3)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_3),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_4)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_4),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_5)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_5),PASS_FAST);
		else 
			soc=clearBall(CLEAR_BALL_DEFENSIVE,SIDE_RIGHT,NULL );
	}
	}
	else if (WM->isPenaltyUs()==true)
	{
	 	soc = getBestKick();
	}

	else
	{
	 if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(20.0) , posAgent,posteammate)==0)
		{
 		soc=directPass(posteammate,PASS_FAST);
		cout<<endl<<endl<<endl<<";;;;;;;;;;;;;;;;;;;;;;;;;;;;;:"<<endl;
		}
 	else
		{
		cout<<endl<<endl<<endl<<"sssssssssssssssssssssssssssss:"<<endl;
 		soc = directPass(VecCC,PASS_FAST);
		}
	}
	//soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal
   
      ACT->putCommandInQueue( soc );
     ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      Log.log( 100, "kick ball" );
    }
    else if( WM->getFastestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp )
              == WM->getAgentObjectType()  && !WM->isDeadBallThem() )
    {                                                // if fastest to ball
      Log.log( 100, "I am fastest to ball; can get there in %d cycles", iTmp );
      soc = intercept( false );                      // intercept the ball

      if( soc.commandType == CMD_DASH &&             // if stamina low
          WM->getAgentStamina().getStamina() <
             SS->getRecoverDecThr()*SS->getStaminaMax()+200 )
      {
        soc.dPower = 30.0 * WM->getAgentStamina().getRecovery(); // dash slow
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else                                           // if stamina high
      {
        ACT->putCommandInQueue( soc );               // dash as intended
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
     }
     else if( posAgent.getDistanceTo(WM->getStrategicPosition()) >
                  1.5 + fabs(posAgent.getX()-posBall.getX())/10.0)
                                                  // if not near strategic pos
     {
       if( WM->getAgentStamina().getStamina() >     // if stamina high
                            SS->getRecoverDecThr()*SS->getStaminaMax()+800 )
       {
         soc = moveToPos(WM->getStrategicPosition(),
                         PS->getPlayerWhenToTurnAngle());
         ACT->putCommandInQueue( soc );            // move to strategic pos
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
       else                                        // else watch ball
       {
         ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
     }
     else if( fabs( WM->getRelativeAngle( OBJECT_BALL ) ) > 1.0 ) // watch ball
     {
       ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
     }
     else                                         // nothing to do
       ACT->putCommandInQueue( SoccerCommand(CMD_TURNNECK,0.0) );
   }
  return soc;
}

/*!This method is a simple goalie based on the goalie of the simple Team of
   FC Portugal. It defines a rectangle in its penalty area and moves to the
   position on this rectangle where the ball intersects if you make a line
   between the ball position and the center of the goal. If the ball can
   be intercepted in the own penalty area the ball is intercepted and catched.
*/
SoccerCommand Player::deMeer5_goalie(  )

{
 VecPosition   posBall  = WM->getBallPos();
 SoccerCommand soc;
if ((posBall.getX()<-40.0)and((posBall.getY()<-18.0)||(posBall.getY()>18.0)))
{

int i;
  SoccerCommand soc;
  VecPosition   posAgent = WM->getGlobalPosition(WM->getOwnGoalieType());
  AngDeg        angBody  = WM->getAgentGlobalBodyAngle();

  // define the top and bottom position of a rectangle in which keeper moves
  static const VecPosition posLeftTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH/4.0 );
  static const VecPosition posRightTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, +PENALTY_AREA_WIDTH/4.0 );

  // define the borders of this rectangle using the two points.
  static Line  lineFront = Line::makeLineFromTwoPoints(posLeftTop,posRightTop);
  static Line  lineLeft  = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posLeftTop.getY()), posLeftTop );
  static Line  lineRight = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posRightTop.getY()),posRightTop );
//Line LINE_RIGHT,LI_BET,LINE_LEFT;
AngDeg ANG_TWO_LINE;
double m1,m2;
//if(WM->getSide()==SIDE_LEFT)
//{
Line LINE_RIGHT=Line::makeLineFromTwoPoints(WM->getGlobalPosition (OBJECT_FLAG_G_R_T ),WM->getGlobalPosition(OBJECT_BALL));
Line LINE_LEFT=Line::makeLineFromTwoPoints(WM->getGlobalPosition(OBJECT_FLAG_G_R_B ),WM->getGlobalPosition(OBJECT_BALL));
//}
//else
//{
//Line LINE_RIGHT=Line::makeLineFromTwoPoints(WM->getGlobalPosition(OBJECT_FLAG_G_L_T ),WM->getGlobalPosition(OBJECT_BALL));
//Line LINE_LEFT=Line::makeLineFromTwoPoints(WM->getGlobalPosition(OBJECT_FLAG_G_L_B ),WM->getGlobalPosition(OBJECT_BALL));
//}
m1=(-1)*(LINE_RIGHT.getACoefficient())/LINE_RIGHT.getBCoefficient();
m2=(-1)*(LINE_LEFT.getACoefficient())/LINE_LEFT.getACoefficient();
ANG_TWO_LINE=atan(absP((m1-m2)/(1+(m1*m2))));
Line LI_BET=Line::makeLineFromPositionAndAngle((WM->getGlobalPosition(OBJECT_BALL)),((180.0-ANG_TWO_LINE)/2));
  if( WM->isBeforeKickOff( ) )
  {
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos(WM->getStrategicPosition()) );
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc = turnBodyToPoint( VecPosition( 0, 0 ), 0 ));
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
    return soc;
  }

  if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
  {                                                // confidence ball too  low
    ACT->putCommandInQueue( searchBall() );        // search ball
    ACT->putCommandInQueue( alignNeckWithBody( ) );
  }
  else if( WM->getPlayMode() == PM_PLAY_ON || WM->isFreeKickThem() ||
           WM->isCornerKickThem() )               
  {
    if( WM->isBallCatchable() )
    {
      ACT->putCommandInQueue( soc = catchBall() );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
     else if( WM->isBallKickable() )
    {
       soc = clearBall(CLEAR_BALL_OFFENSIVE ,WM->getSide(),NULL);
       ACT->putCommandInQueue( soc );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else if( WM->isInOwnPenaltyArea( getInterceptionPointBall( &i, true ) ) &&
             WM->getFastestInSetTo( OBJECT_SET_PLAYERS, OBJECT_BALL, &i ) == 
                                               WM->getAgentObjectType() )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
    {
      // make line between own goal and the ball
      VecPosition posMyGoal = ( WM->getSide() == SIDE_LEFT )
             ? SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_L, SIDE_LEFT )
             : SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_R, SIDE_RIGHT);
      Line lineBall = Line::makeLineFromTwoPoints( WM->getBallPos(),posMyGoal);

      // determine where your front line intersects with the line from ball
      VecPosition posIntersect = lineFront.getIntersection( lineBall );//lineFront.getIntersection( lineBall );

      // outside rectangle, use line at side to get intersection
      if (posIntersect.isRightOf( posRightTop ) )
        posIntersect = lineRight.getIntersection( lineBall );
      else if (posIntersect.isLeftOf( posLeftTop )  )
        posIntersect = lineLeft.getIntersection( lineBall );

      if( absP(posIntersect.getX()) < 49.0 )   {posIntersect.setX( -52.0 );}
if(posAgent.getY()>5.0||posAgent.getY()<-5.0)      
{
if((WM->getGlobalPosition(OBJECT_BALL)).getY()>1.0)       {posIntersect.setY(posIntersect.getY()-3.67);}
else if((WM->getGlobalPosition(OBJECT_BALL)).getY()<1.0)    {posIntersect.setY(posIntersect.getY()+3.67);}
else if((WM->getGlobalPosition(OBJECT_BALL)).getY()==1.0)    {posIntersect.setY(0.0);}
//else if((WM->getGlobalPosition(OBJECT_BALL)).getY()<0.0) {posIntersect.setY(-2.0);}
//else if((WM->getGlobalPosition(OBJECT_BALL)).getY()==0.0)       {posIntersect.setY(0.0);}
}
if(absP(posIntersect.getX())>-52.0)  {posIntersect.setX(-52.0);}
      // and move to this position
cout<<endl<<endl<<endl<<endl<<endl<<posIntersect;
      if( posIntersect.getDistanceTo( WM->getAgentGlobalPosition() ) > 0.5 )
      {
        soc = moveToPos( posIntersect, PS->getPlayerWhenToTurnAngle() );
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else
      {
        ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
  }
  else if( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true )
  {
    if( WM->isBallKickable() )
    {
      if( WM->getTimeSinceLastCatch() == 25 && WM->isFreeKickUs() )
      {
        // move to position with lesser opponents.
        if( WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                          Circle(posRightTop, 15.0 )) <
            WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                           Circle(posLeftTop,  15.0 )) )
          soc.makeCommand( CMD_MOVE,posRightTop.getX(),posRightTop.getY(),0.0);
        else
          soc.makeCommand( CMD_MOVE,posLeftTop.getX(), posLeftTop.getY(), 0.0);
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() > 28 )
      {
        soc = clearBall(CLEAR_BALL_OFFENSIVE ,WM->getSide(),NULL);
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() < 25 )
      {
        VecPosition posSide( 0.0, posAgent.getY() ); 
        if( fabs( (posSide - posAgent).getDirection() - angBody) > 10 )
        {
          soc = turnBodyToPoint( posSide );
          ACT->putCommandInQueue( soc );
        }
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
    else if( WM->isGoalKickUs()  )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  else
  {
     ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
     ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }

}

else
{
 
 int i;
  SoccerCommand soc;
  VecPosition   posAgent = WM->getAgentGlobalPosition();
  AngDeg        angBody  = WM->getAgentGlobalBodyAngle();

  // define the top and bottom position of a rectangle in which keeper moves
  static const VecPosition posLeftTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH/4.0 );
  static const VecPosition posRightTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, +PENALTY_AREA_WIDTH/4.0 );

  // define the borders of this rectangle using the two points.
  static Line  lineFront = Line::makeLineFromTwoPoints(posLeftTop,posRightTop);
  static Line  lineLeft  = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posLeftTop.getY()), posLeftTop );
  static Line  lineRight = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posRightTop.getY()),posRightTop );


  if( WM->isBeforeKickOff( ) )
  {
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos(WM->getStrategicPosition()) );
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc = turnBodyToPoint( VecPosition( 0, 0 ), 0 ));
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
    return soc;
  }

  if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
  {                                                // confidence ball too  low
    ACT->putCommandInQueue( searchBall() );        // search ball
    ACT->putCommandInQueue( alignNeckWithBody( ) );
  }
  else if( WM->getPlayMode() == PM_PLAY_ON || WM->isFreeKickThem() ||
           WM->isCornerKickThem() )               
  {
    if( WM->isBallCatchable() )
    {
      ACT->putCommandInQueue( soc = catchBall() );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
     else if( WM->isBallKickable() )
    {
       soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );    
       ACT->putCommandInQueue( soc );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else if( WM->isInOwnPenaltyArea( getInterceptionPointBall( &i, true ) ) &&
             WM->getFastestInSetTo( OBJECT_SET_PLAYERS, OBJECT_BALL, &i ) == 
                                               WM->getAgentObjectType() )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
    {
      // make line between own goal and the ball
      VecPosition posMyGoal = ( WM->getSide() == SIDE_LEFT )
             ? SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_L, SIDE_LEFT )
             : SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_R, SIDE_RIGHT);
      Line lineBall = Line::makeLineFromTwoPoints( WM->getBallPos(),posMyGoal);

      // determine where your front line intersects with the line from ball
      VecPosition posIntersect = lineFront.getIntersection( lineBall );

      // outside rectangle, use line at side to get intersection
      if (posIntersect.isRightOf( posRightTop ) )
        posIntersect = lineRight.getIntersection( lineBall );
      else if (posIntersect.isLeftOf( posLeftTop )  )
        posIntersect = lineLeft.getIntersection( lineBall );

      if( posIntersect.getX() < -49.0 )
        posIntersect.setX( -49.0 );
        
      // and move to this position
      if( posIntersect.getDistanceTo( WM->getAgentGlobalPosition() ) > 0.5 )
      {
        soc = moveToPos( posIntersect, PS->getPlayerWhenToTurnAngle() );
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else
      {
        ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
  }
  else if( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true )
  {
    if( WM->isBallKickable() )
    {
      if( WM->getTimeSinceLastCatch() == 25 && WM->isFreeKickUs() )
      {
        // move to position with lesser opponents.
        if( WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                          Circle(posRightTop, 15.0 )) <
            WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                           Circle(posLeftTop,  15.0 )) )
          soc.makeCommand( CMD_MOVE,posRightTop.getX(),posRightTop.getY(),0.0);
        else
          soc.makeCommand( CMD_MOVE,posLeftTop.getX(), posLeftTop.getY(), 0.0);
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() > 28 )
      {

	if (posAgent.getY()<0.0)
	{
		soc.makeCommand( CMD_MOVE,posLeftTop.getX()+9.0, posLeftTop.getY()-13.0, -30.0);
	
		if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_5)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_5),PASS_FAST);
	
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_9)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_9),PASS_FAST);


		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posRightTop 							,WM->getGlobalPosition(OBJECT_TEAMMATE_2)  )==0)
			soc.makeCommand( CMD_MOVE,posRightTop.getX()+9.0,posRightTop.getY()+13.0,30.0);

		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_4)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_4),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_3)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_3),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_2)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_2),PASS_FAST);
		else 
			soc=clearBall(CLEAR_BALL_DEFENSIVE,SIDE_LEFT,NULL );
	}
	else if (posAgent.getY()>=0.0)
	
	{

		soc.makeCommand( CMD_MOVE,posLeftTop.getX()+9.0, posLeftTop.getY()-13.0, -30.0);
		if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_2)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_2),PASS_FAST);
		
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_6)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_6),PASS_FAST);


		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(15.0) , posLeftTop 							,WM->getGlobalPosition(OBJECT_TEAMMATE_5)  )==0)
			soc.makeCommand( CMD_MOVE,posLeftTop.getX()+9.0, posLeftTop.getY()-13.0, -30.0);

		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_3)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_3),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_4)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_4),PASS_NORMAL);
		else if (WM->getNrInSetInCone(OBJECT_SET_OPPONENTS,tanDeg(30.0) , posAgent 							,WM->getGlobalPosition(OBJECT_TEAMMATE_5)  )==0)
			soc = directPass(WM->getGlobalPosition(OBJECT_TEAMMATE_5),PASS_FAST);
		else 
			soc=clearBall(CLEAR_BALL_DEFENSIVE,SIDE_RIGHT,NULL );
	}
		
        //soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );    
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() < 25 )
      {
        VecPosition posSide( 0.0, posAgent.getY() ); 
        if( fabs( (posSide - posAgent).getDirection() - angBody) > 10 )
        {
          soc = turnBodyToPoint( posSide );
          ACT->putCommandInQueue( soc );
        }
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
    else if( WM->isGoalKickUs()  )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  else
  {
     ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
     ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }

	}
  return soc;
}

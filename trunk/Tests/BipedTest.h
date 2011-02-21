/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BIPED_TEST_H
#define BIPED_TEST_H

#include "Biped.h"
#include<iostream>

using namespace std;

class BipedTest : public Test
{
public:

    BipedTest()
	{

		const float32 k_restitution = 1.4f;

		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 5.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.density = 0.0f;
			sd.restitution = k_restitution;

//			sd.SetAsBox(0.1f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0f);
//			body->CreateShape(&sd);

//			sd.SetAsBox(0.1f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0f);
//			body->CreateShape(&sd);

			sd.SetAsBox(0.5f, 100.0f, b2Vec2(0.0f, -10.0f), 0.5f * b2_pi);
			body->CreateShape(&sd);

			//sd.SetAsBox(0.1f, 10.0f, b2Vec2(0.0f, 10.0f), -0.5f * b2_pi);
			//body->CreateShape(&sd);
		}

		m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
		InitQtable();
		intToBinary(10);
		printf("\n");
		printQtable();


	}

	~BipedTest()
	{
		delete m_biped;
	}

	void Keyboard(unsigned char key){

		switch(key){
			case 'q':
                action[0] = 1;
				cout << "Q" << endl;
				break;
			case 'w':
                action[1] = 1;
				cout << "W" << endl;
				break;
			case 'o':
                action[2] = 1;
                cout << "O" << endl;
				break;
			case 'p':
                action[3 ] = 1;
				cout << "P" << endl;
				break;
		}
	}

	void KeyboardUp(unsigned char key){
		B2_NOT_USED(key);
        switch(key){
			case 'q':
                action[0] = 0;
				cout << "-Q" << endl;
				break;
			case 'w':
                action[1] = 0;
				cout << "-W" << endl;
				break;
			case 'o':
                action[2] = 0;
                cout << "-O" << endl;
				break;
			case 'p':
                action[3] = 0;
				cout << "-P" << endl;
				break;
		}
    }

	void Step(Settings* settings)
	{
		Test::Step(settings);
		//DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += 15;
	}

    int action [4];
    int qTable [16][16];
	float loop(){
	    //printf("%f \n",m_biped->Head->GetPosition().x);
	    if(action[0]== 0 && action[1] == 0){
            m_biped->UnSetMotorQW();
	    }else{
            if(action[0] == 1)
                m_biped->SetMotorQ();
            else
                m_biped->SetMotorW();
	    }
	    if(action[2]== 0 && action[3] == 0){
            m_biped->UnSetMotorOP();
	    }else{
            if(action[2] == 1)
                m_biped->SetMotorO();
            else
                m_biped->SetMotorP();
	    }
	    int fall = 0;
	    for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			b2Body* body1 = point->shape1->GetBody();
			b2Body* body2 = point->shape2->GetBody();
			int bp1 = (int)body1->GetUserData();
			int bp2 = (int)body2->GetUserData();

			if (bp1 > 6 || bp2 > 6)
			{
			    fall = 1;
			    break;
            }
		}
		if(fall == 1){
		    m_biped->~Biped();
            m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
        }
		fall = 0;
		return m_biped->Head->GetPosition().x;
	}

    void  InitQtable(){
        for(int i = 0; i < 16; i++){
            for(int j = 0; j < 16; j++){
                if(i == 3 || i == 7 || i > 10 || j == 3 || j == 7 || j > 10)
                    qTable[i][j] = -1000;
                else
                    qTable[i][j] = 0;
            }
	    }
    }

    void printQtable(){
        for(int i = 0; i < 16; i++){
            printf("|");
            for(int j = 0; j < 16; j++){
                printf("%i ",qTable[i][j]);
            }
            printf("|\n");
	    }
    }
    // Returns an integer array with the integer in binary
	int* intToBinary(int integer){
	    int Q,W,O,P;
	    Q = W = O = P = 1;
	    for(int i = 0; i <= integer; i++){
            if(i%2 == 0){
                P = 0;
            }
            else{
                P = 1;
            }

            if(i%2 == 0){
                if(O == 1)
                    O = 0;
                else
                    O = 1;
            }
            if(i%4 == 0){
                if(W == 1)
                    W = 0;
                else
                    W = 1;
            }
            if(i%8 == 0){
                if(Q == 1)
                    Q = 0;
                else
                    Q = 1;
            }
	    }
	    int temp [4];// = new int [4];
	    temp[0] = Q;
	    temp[1] = W;
	    temp[2] = O;
	    temp[3] = P;
	    return temp;
	}

	static Test* Create()
	{
		return new BipedTest;
	}
	Biped* m_biped;
};

#endif

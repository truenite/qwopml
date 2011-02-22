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

    int distancia;
    int action [4];
    int actions[16][4];
    float w0,w1,w2;
    int x1,x2,currAction;
    int inicio;

    struct state{
        state *next;
        int x1, x2, currAction;
    };

    state *head;

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

			sd.SetAsBox(0.5f, 100.0f, b2Vec2(0.0f, -10.0f), 0.5f * b2_pi);
			body->CreateShape(&sd);
		}

		m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
		distancia = m_biped->Head->GetPosition().x;
		initActions();
		//printActionsTable();
		w0 = w1 = w2 = 1;
		x1 = xOne();
		x2 = xTwo();
		currAction = 0;
		head = new state;
		UnityTest();
		addState();
		inicio = 0;
		//setAction();
		//updateWeights();
	}

	~BipedTest()
	{
        state* temp = head;
        state* del = head;
        if(temp){
            if(!temp->next){
                //printf("Aqui %d\n",temp->value);
                free(del);
            }
            else{
                //printf("aca\n");
                temp=temp->next;
                for(;temp;temp=temp->next){
                    free(del);
                    del=temp;
                }
                free(del);
            }
        }
        head=NULL;
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
            case 'j': m_biped->~Biped();
                        m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
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
		distancia = m_biped->Head->GetPosition().x;
		DrawString(5, m_textLine, "distancia = %d", (int)distancia);
		m_textLine += 15;
		if(inicio == 0)
            setAction();
        inicio=1;
	}

    void initActions(){
        int Q,W,O,P;
        Q = W = O = P = 1;
        for(int i = 0; i < 16; i++){
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
            actions[i][0] = Q;
            actions[i][1] = W;
            actions[i][2] = O;
            actions[i][3] = P;
        }

    }

    void printActionsTable(){
        for(int i = 0; i < 16; i++){
            printf("|");
            for(int j = 0; j < 4; j++){
                printf("%i ",actions[i][j]);
            }
            printf("|\n");
        }
    }

    void addState(){
        state *newOne = (state *)malloc(sizeof(state));

        if(!newOne){
            printf("Error al alojar memoria");
            exit(1);
        }
        memset(newOne,0,sizeof(newOne));
        state *temp = head;
        newOne -> next = NULL;
        while(temp->next != NULL)
            temp = temp->next;
        temp->next = newOne;
        newOne->x1 = x1;
        newOne->x2 = x2;
        newOne->currAction = currAction;
        newOne->next = NULL;
    }

    float Vb(int w0, int w1, int w2, int x1, int x2){
        if(x1 >= 100)
            return 1000;
        if(checkFall())
            return -1000;
        return (w0 + (w1*x1) + (w2*x2));
    }

    int xOne(){
        return (int)distancia;
    }

    int xTwo(){
        int slope;
        int x = (int)m_biped->Head->GetPosition().x - (int)m_biped->Pelvis->GetPosition().x;
        int y = (int)m_biped->Head->GetPosition().y - (int)m_biped->Pelvis->GetPosition().y;
        if(y == 0)
            return 90;
        slope = x/y;
        return slope;
    }

    void setAction(){
        // We are going to get the best state
        // and its x's so we are going to try
        // so we are going to try this with all
        // 16 possible states
        printf("Juan");
        for(int i = 0; i < 16; i++){
            printf("juanito\n");
            // First, we reset our biped
            m_biped->~Biped();
            m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
            state *temp = head;
            // We go through all the other states so we can try a new one
            while(temp->next != NULL){
                int* temp2 = intToBinary(temp->currAction);//temp.currAction);
                for(int j = 0; j < 4; j++)
                    action[j]=(int)temp2++;
                doAction();
                temp = temp->next;
            }
            // Here we set our todoaction to the value of i
            // so it tries this posible action
            int* temp2 = intToBinary(i);//temp.currAction);
            for(int j = 0; j < 4; j++)
                action[j]=(int)temp2++;
            doAction();
            int x1Temp = xOne();
            int x2Temp = xTwo();
            // If the result is better, then we save it
            // so later we can introduce it to our linkedlist
            if(checkFall() == 1){
                x1 = x1Temp;
                x2 = x2Temp;
                currAction = i;
                addState();
                printf("pedro");
                //updateWeights();
                return;
            }
            else{
                if(Vb(w0,w1,w2,x1,x2) < Vb(w0,w1,w2,x1Temp,x2Temp)){
                    x1 = x1Temp;
                    x2 = x2Temp;
                    currAction = i;
                }
            }
        }
        addState();
        setAction();
    }

    void  updateWeights(){
        state * temp = head;
        if(temp->next == NULL){
            printf("Error 404");
            return;
        }
        printf("intentando update");

        float newW1 = 0;
        float newW2 = 0;
        while(head->next != NULL){
            printf("aki1");
            if(temp->next->next == NULL){
                newW1 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x1,w1);
                newW2 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x2,w2);
                w1 = newW1;
                w2 = newW2;
                state * temp2 = temp->next;
                temp->next = NULL;
                free(temp2);
                printf("\n\n\n\naki2\n\n\n");
            }
            else{
                temp = head;
                while(temp->next->next != NULL){
                    temp = temp->next;
                }
                state * temp2 = temp->next;
                newW1 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x1,w1);
                newW2 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x2,w2);
                w1 = newW1;
                w2 = newW2;
                temp->next = NULL;
                free(temp2);
                printf("aki3");
            }
        }
        printf("updateados");
    }

    float updateWeight(float Vtrain, float Vb, int xi, float wi){
        float newWi = 0;

        newWi = (wi + (0.1 * (Vtrain - Vb) * xi));

        return newWi;
    }

    void UnityTest(){

        /*int temp2 = 0;// temp->currAction;
        printf("temp2: %i\n",temp2);
        int* temp = intToBinary(temp2);//temp.currAction);
        printf("ArregloI: ");
        for(int j = 0; j < 4; j++){
            action[j]=temp[j];
            printf("%i ",temp[j]);
        }
        temp = intToBinary(temp2);
        printf("\nArreglo: ");
        for(int j = 0; j < 4; j++){
            printf("%i ",action[j]);
        }
        printf("X1 = %i   X2= %i\n",xOne(),xTwo());
        printf("headx = %i  hipx= %i  headx - hipx: %i\n",(int)m_biped->Head->GetPosition().x , (int)m_biped->Pelvis->GetPosition().x,(int)m_biped->Head->GetPosition().x - (int)m_biped->Pelvis->GetPosition().x);
        printf("heady = %i  hipy= %i  heady - hipy: %i\n",(int)m_biped->Head->GetPosition().y , (int)m_biped->Pelvis->GetPosition().y,(int)m_biped->Head->GetPosition().y - (int)m_biped->Pelvis->GetPosition().y);
        printf("m = %f\n",(m_biped->Head->GetPosition().x - m_biped->Pelvis->GetPosition().x)/(m_biped->Head->GetPosition().y - m_biped->Pelvis->GetPosition().y));
        printf("\n");*/

    }

    void doAction(){
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
    }

    int checkFall(){
        int bol = 0;
        for (int32 i = 0; i < m_pointCount; ++i){
			ContactPoint* point = m_points + i;

			b2Body* body1 = point->shape1->GetBody();
			b2Body* body2 = point->shape2->GetBody();
			int bp1 = (int)body1->GetUserData();
			int bp2 = (int)body2->GetUserData();
            printf("bp1 = %i   bp2 %i",bp1,bp2);
			if (bp1 > 6 || bp2 > 6){
			    bol = 1;
			    return bol;
            }
		}
		return bol;
    }

	float loop(){
	    return m_biped->Head->GetPosition().x;
	}

    // Returns an integer array with the integer in binary
	int* intToBinary(int integer){
	    if(integer > 15)
            return NULL;
	    int *temp = new int[4];
	    for(int i = 3; i >= 0; i--){
            temp[i] = integer%2;
            integer = integer/2;
	    }
        return temp;
	}

	static Test* Create()
	{
		return new BipedTest;
	}
	Biped* m_biped;
};

#endif

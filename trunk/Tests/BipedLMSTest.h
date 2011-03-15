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

#ifndef BIPEDLMS_TEST_H
#define BIPEDLMS_TEST_H

#include "Biped.h"
#include<iostream>

using namespace std;

class BipedLMSTest : public Test
{
public:

    int distancia;
    int action [4];
    int actions[16][4];
    float w0,w1,w2;
    float x1,x2;
    int currAction;
    int inicio;
    timer t;
    int ite, ite2, strucLenght, stage, terminal;
    double wait;
    Settings* set;

    struct state{
        state *next;
        float x1, x2;
        int currAction, terminal;
    };

    state *head;

    state *temporal;

    BipedLMSTest()
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
		w0 = 1;
		w1 = 7;
		w2 = 10;
		x1 = xOne();
		x2 = xTwo();
		currAction = 0;
		head = new state;
		head->x1 = x1;
		head->x2 = x2;
		head->currAction = 0;
		head->next = NULL;
		temporal = head;
		UnityTest();
        inicio = 0;
		ite = 0;
		ite2 = 0;
		strucLenght = 0;
		stage = 0;
		currAction = 0;
		addState();
		wait = 1.5;
		t.start();
		terminal = 0;
	}

	~BipedLMSTest()
	{
        state* temp = head;
        state* del = head;
        if(temp){
            if(!temp->next){
                free(del);
            }
            else{
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

	void Step(Settings* settings)
	{
        set = settings;
		Test::Step(settings);
		distancia = m_biped->Head->GetPosition().x;
		DrawString(5, m_textLine, "distancia = %d", (int)distancia);
		m_textLine += 15;
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
        state *newOne = new state;
        state *temp = head;
        newOne -> next = NULL;
        while(temp->next != NULL)
            temp = temp->next;
        temp->next = newOne;
        newOne->x1 = x1;
        newOne->x2 = x2;
        newOne->currAction = currAction;
        newOne->next = NULL;
        newOne->terminal = terminal;
        printf("curr action %i  x1 %f, x2  %f  V(b)= %f\n",currAction,x1,x2,Vb(w0,w1,w2,x1,x2));
        printf("curr action del nodo %i\n",newOne->currAction);
        strucLenght++;
    }

    float Vb(float w0, float w1, float w2, float x1, float x2){
        if(x1 >= 100)
            return 1000;
        if(x1 <= -1000 && x2 <= -1000){
            x1+=1000;
            x2+=1000;
            x1*=-1;
            x2*=-1;
            //printf("x1 = %f  x2 = %f\n",x1,x2);
            return 1;
        }
        return (w0 + (w1*x1) + (w2*x2));
    }

    float xOne(){
        return (int)distancia;
    }

    float xTwo(){
        float slope;
        float x = m_biped->Head->GetPosition().x - m_biped->Pelvis->GetPosition().x;
        float y = m_biped->Head->GetPosition().y - m_biped->Pelvis->GetPosition().y;
        if(x == 0)
            return 99999999 ;
        slope = y/x;
        //slope = sqrt((slope*slope));
        //printf("y1 = %f   y2= %f\n",m_biped->Head->GetPosition().y , m_biped->Pelvis->GetPosition().y);
        //printf("slope %f   y = %f   x= %f\n",slope,y,x);
        return slope;
    }

    void getToLastState(){
        if(stage == 0){
            printf("ite %i\n",ite);
            if(ite == 0){
                m_biped->~Biped();
                m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
                temporal = head->next;
                for(int k = 0; k < 4; k++){
                    action[k]=actions[0][k];
                }
                t.start();
                ite++;
                return;
            }
            if(temporal->next == NULL){
                ite = 0;
                stage = 1;
                t.start();
                return;
            }
            for(int j = 0; j < ite; j++){
                if(temporal->next!=NULL){
                    temporal = temporal->next;
                }
            }
            int current = temporal->currAction;
            for(int k = 0; k < 4; k++){
                action[k]=actions[current][k];
            }
            ite++;
            t.start();
        }
    }

    void calculateBest(){
        if(ite2 > 0 && stage == 2){
            float x1Temp = xOne();
            float x2Temp = xTwo();
            /*if(checkFall() == 1){
                addState();
                x1 *= -1;
                x2 *= -1;
                x1 -= 1000;
                x2 -= 1000;
                currAction = ite2-1;
                stage = 3;
                ite2 = 0;
                updateWeights2();
                currAction = 0;
                x1,x2 = 0;
                return;
            }
            else{*/
                if(x1Temp>100)
                {
                    currAction = ite2-1;
                    addState();
                    stage = 0;
                    ite2 = 0;
                    updateWeights2();
                    stage = 3;
                    currAction = 0;
                    x1 = 0;
                    x2 = 0;
                    return;
                }
                else{
                    printf("Vb pas = %f  VbNueva = %f\n",Vb(w0,w1,w2,x1,x2),Vb(w0,w1,w2,x1Temp,x2Temp));
                    if(Vb(w0,w1,w2,x1,x2) < Vb(w0,w1,w2,x1Temp,x2Temp)){
                        //printf("si\n");
                        x1 = x1Temp;
                        x2 = x2Temp;
                        currAction = ite2-1;
                    }
                }
                if(ite2 == 16)
                    ite2 = 0;
                stage = 0;
            //}
        }
    }

    void doLastState(){
        if(stage == 1){
            printf("dolast %i\n",ite2);
            for(int k = 0; k < 4; k++){
                action[k] = actions[ite2][k];
            }
            if(ite2==15){
                for(int k = 0; k < 4; k++){
                    action[k] = 0;
                }
                stage = 2;
                ite2++;
                printf("stage 0! ite:%i  TERMINO\n",ite);
                addState();
                currAction = 0;
                x1 = 0;
                x2 = 0;
                t.start();
                return;
            }
            else{
                stage = 2;
                ite = 0;
                ite2++;
                t.start();
            }
        }
    }
    void loopEd(){
        if(t.elapsedTime() >= wait){
            switch(stage){
                case 0:
                    getToLastState();
                    break;
                case 1:
                    doLastState();
                    break;
                case 2:
                    calculateBest();
                    break;
                default:
                    break;
            }
        }
    }
/*
    void setAction(){
        if(timer!= 0 && 1 >= ((unsigned long) clock() - timer) / CLOCKS_PER_SEC){
            return;
        }
        x1 = 0;
        x2 = 0;
        int stateSize = 0;
        for(int i = 0; i < 16; i++){
            // First, we reset our biped
            m_biped->~Biped();
            m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
            state * temp = head;
            if(temp->next == NULL){
                printf("Error 404");
                exit(1);
            }
            temp = temp->next;
            // We go through all the other states so we can try a new one
            while(temp->next != NULL){
                stateSize ++;
                //printf("juan3 %i\n",temp->currAction);
                for(int j = 0; j < 4; j++){
                    action[j]=actions[temp->currAction][j];
                }
                float x1Temp = xOne();
                float x2Temp = xTwo();
                printf("xs: %f   %f\n",x1Temp,x2Temp);
                doAction();
                temp = temp->next;
                x1Temp = xOne();
                x2Temp = xTwo();
                printf("accion: %i%i%i%i\n",action[0],action[1],action[2],action[3]);
                printf("xs: %f   %f\n",x1,x2);
                printf("xs: %f   %f\n",x1Temp,x2Temp);
                printf("actual: %f\n",Vb(w0,w1,w2,x1,x2));
                printf("temp: %f\n",Vb(w0,w1,w2,x1Temp,x2Temp));
            }
            for(int j = 0; j < 4; j++){
                action[j]=actions[temp->currAction][j];
            }
            doAction();
            // Here we set our todoaction to the value of i
            // so it tries this posible action
            for(int j = 0; j < 4; j++)
                action[j]=actions[i][j];
            doAction();
            float x1Temp = xOne();
            float x2Temp = xTwo();
            // If the result is better, then we save it
            // so later we can introduce it to our linkedlist
            printf("accion: %i%i%i%i\n",action[0],action[1],action[2],action[3]);
            printf("xs: %f   %f\n",x1,x2);
            printf("xs: %f   %f\n",x1Temp,x2Temp);
            printf("actual: %f\n",Vb(w0,w1,w2,x1,x2));
            printf("temp: %f\n",Vb(w0,w1,w2,x1Temp,x2Temp));
            if(checkFall() == 1){
                x1 = x1Temp;
                x2 = x2Temp;
                currAction = i;
                addState();
                printf("No por favor no");
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
        printf("\n Vb = %f",Vb(w0,w1,w2,x1,x2));
        addState();
        iterationsAdjust --;
        iterationsAdjust = iterationsMax;
        if(iterationsAdjust != 0){
            timer = clock();
        }
        else{
            iterationsAdjust = iterationsMax;
            updateWeights();
        }
    }
*/
    void  updateWeights(){
        state * temp = head;
        if(temp->next == NULL){
            printf("Error 404");
            return;
        }
        printf("intentando update");
        temp = temp->next;
        float newW1 = 0;
        float newW2 = 0;
        while(head->next != NULL){
            if(temp->next->next == NULL){
                newW1 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x1,w1);
                newW2 = updateWeight(Vb(w0,w1,w2,temp->next->x1,temp->next->x2),Vb(w0,w1,w2,x1,x2),x2,w2);
                w1 = newW1;
                w2 = newW2;
                state * temp2 = temp->next;
                temp->next = NULL;
                free(temp2);
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
            }
        }
    }

    float updateWeight(float Vtrain, float Vb, int xi, float wi){
        float newWi = 0;

        newWi = (wi + (0.01 * (Vtrain - Vb) * xi));

        printf("=====updateWeight NEW w%f =====\n",newWi);
        return newWi;
    }

    void UnityTest(){
        /*
        int temp1 = 1;// temp->currAction;
        printf("temp2: %i\n",temp1);
        int* temp2 = intToBinary(temp1);//temp.currAction);
        printf("ArregloI: ");
        for(int j = 0; j < 4; j++){
            action[j]=temp2[j];
            printf("%i ",temp2[j]);
        }
        temp2 = intToBinary(temp1);
        printf("\nArreglo: ");
        for(int j = 0; j < 4; j++){
            printf("%i ",action[j]);
        }/*
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
	    //Step(set);
    }

    int checkFall(){
        int bol = 0;
        for (int32 i = 0; i < m_pointCount; ++i){
			ContactPoint* point = m_points + i;

			b2Body* body1 = point->shape1->GetBody();
			b2Body* body2 = point->shape2->GetBody();
			int bp1 = (int)body1->GetUserData();
			int bp2 = (int)body2->GetUserData();
			if (bp1 > 6 || bp2 > 6){
			    bol = 1;
			    return bol;
            }
		}
		return bol;
    }

	float loop(){
	    doAction();
	    loopEd();
	    return m_biped->Head->GetPosition().x;
	}

    void reset(){
        state* temp = head;
        state* del = head;
        if(temp){
            if(!temp->next){
                free(del);
            }
            else{
                temp=temp->next;
                for(;temp;temp=temp->next){
                    free(del);
                    del=temp;
                }
                free(del);
            }
        }
        currAction = 0;
        head = new state;
		head->x1 = 0;
		head->x2 = 0;
		head->currAction = 0;
		head->next = NULL;
		x1 = 0;
		x2 = 0;
		addState();
	}

	void updateWeights2(){
	    state *tempLast = head->next;
        while(tempLast->next != NULL)
            tempLast = tempLast->next;
        while(head->next != tempLast){
            state *tempUpdate = head->next;
            while(tempUpdate->next != tempLast && tempUpdate != tempLast)
                tempUpdate=tempUpdate->next;
            float VbNext = Vb(w0,w1,w2,tempLast->x1,tempLast->x2);
            float VbThis = Vb(w0,w1,w2,tempUpdate->x1,tempUpdate->x2);
            printf("\n\n\nupdateWheights2   w1 = %f   w2= %f\n",w1,w2);
            w1 = updateWeight(VbNext,VbThis,x1,w1);
            w2 = updateWeight(VbNext,VbThis,x2,w2);
            printf("nuevos   w1 = %f   w2= %f   wbNext %f  wbThis %f \n",w1,w2,VbNext,VbThis,x1,x2);
            tempLast = tempUpdate;

        }
        printf("\n\n\n\nnuevos   w1 = %f   w2= %f\n\n\n\n",w1,w2);
        reset();
        stage = 0;
        t.start();
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

    int binaryToInt(int* binary){
        int integer = 0;
        integer += binary[0]*8;
        integer += binary[1]*4;
        integer += binary[2]*2;
        integer += binary[3]*1;
        return 0;
    }
	static Test* Create()
	{
		return new BipedLMSTest;
	}
	Biped* m_biped;
};

#endif

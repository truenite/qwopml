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
#include "timer.h"
#include <time.h>

using namespace std;

class BipedTest : public Test
{
public:

    int distancia;
    int action [2];
    int qTable [3][3];
    int estadoActual;
    int posActual;
    int estadoAnt;
    int acciones[3][3];
    int listaAcciones[100];
    timer t;
    int contador;
    float gamma;
    float temperatura;

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

		temperatura = 0.9f;
        gamma = 0.5f;
		estadoActual = 0;
		estadoAnt = 0;
		posActual = 0;
		contador = 0;
		acciones = {{'s','q','w'},{'s','q','w'},{'s','q','w'}};
//		QLearning();
		escogerAccion(estadoActual);
		m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
		distancia = m_biped->Head->GetPosition().x;
        t.start();
		InitQtable();
		intToBinary(10);
		printf("\n");
		printQtable();
		ejecuta(contador);

	}

	~BipedTest()
	{
		delete m_biped;
	}

    inline double closed_interval_rand(double x0, double x1){
        return x0 + (x1 - x0) * rand() / ((double) RAND_MAX);
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
                        break;
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
		distancia = m_biped->Head->GetPosition().x;
		DrawString(5, m_textLine, "distancia = %d", (int)distancia);
		m_textLine += 15;
	}

	void escogerAccion(int estadoAct){
	    int max = 0;
	    int posTemp = 0;
        for(int i = 0; i< 3;i++){
            if(qTable[estadoAct][i] >= max){
                max = qTable[estadoAct][i];
                posTemp = i;
            }
        }
//        cout << "estadoActual: " << estadoAct << "POS: " << pos << "ACCION: " << acciones[estadoAct][pos] << endl;

        float randy = closed_interval_rand(0,1);
        int posRand = (int)closed_interval_rand(0,3);

        if(temperatura >= randy){
            posActual = posRand;
            cout << "random " << "posRand: " << posRand << "temp:" << temperatura <<  endl;
        }
        else{
            posActual = posTemp;
            cout << "mejor " << "posMejor: " << posTemp << endl;
        }

        listaAcciones[contador] = acciones[estadoAct][posActual];


	}

    //Prende motores dependiendo de la accion
    void doAction(){
        if(action[0]== 0 && action[1] == 0){
            m_biped->UnSetMotorQW();
            }else{
            if(action[0] == 1)
                m_biped->SetMotorQ();
            else
                m_biped->SetMotorW();
            }
//            if(action[2]== 0 && action[3] == 0){
//            m_biped->UnSetMotorOP();
//            }else{
//            if(action[2] == 1)
//                m_biped->SetMotorO();
//            else
//                m_biped->SetMotorP();
//            }
    }

    //estados finales y su respectivo procesamiento
    void episodio(){
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
		//SE CAE
		if(fall == 1 ){
		    //ciclo final
		    estadoAnt = estadoActual;
            estadoActual = 0;
            QupdateFinal(0);
            //crear nueva instancia
		    m_biped->~Biped();
            m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
            contador= 0;
            ejecuta(contador);
            temperatura = temperatura * 0.99;
            cout << "CAIDA..........................\n" << endl;
        //GANO!!!!!!!!!!
        }else if(distancia == 40){
//              //ciclo final
                estadoAnt = estadoActual;
                estadoActual = 0;
                QupdateFinal(100);
                //crear nueva instancia
                m_biped->~Biped();
                m_biped = new Biped(m_world, b2Vec2(0.0f, 4.0f));
                contador = 0;
                ejecuta(contador);
                temperatura = temperatura * 0.99;
                cout << "GANO.....................\n " << endl;
                }


		fall = 0;
    }

    void ejecuta(int ilocal){
        //listaAcciones = {'s', 's', 'q', 'q', 'w', 'w', 'w', 'q', 'q','w','w','q','q'};

        cout << "TECLA: " << listaAcciones[contador] << endl;

        switch(listaAcciones[ilocal]){
            case 'q': action[1] = 0;
                        action[0] = 1;
                        break;
            case 'w': action[0] = 0;
                        action[1] = 1;
                        break;
            case 's': action[0] = 0;
                        action[1] = 0;
                        break;
            default: cout << "ERROR SWITCH POSICIONES " <<  listaAcciones[ilocal] << " iLocal: " << ilocal << endl;


        }

        contador = (contador+1)%((sizeof(listaAcciones) / sizeof(int))-1);


    }

    void Qupdate(int reward){
        qTable[estadoAnt][posActual] = reward + (gamma * max(estadoActual));
    }

    void QupdateFinal(int reward){
        qTable[estadoAnt][posActual] = reward;
    }

    //realiza las acciones en la lista de acciones
    void modAction(){
        unsigned long int seconds = 1;
        if(t.elapsedTime() >= seconds){
            estadoAnt = estadoActual;
            estadoActual = posActual;
            Qupdate(distancia);
            escogerAccion(estadoActual);
            printQtable();
            ejecuta(contador);
            cout << "ESTADO ANTERIOR:  " <<  estadoAnt <<  endl;
            cout << "ESTADO ACTUAL:  " <<  estadoActual <<  endl;
            cout << "POSICION ACTUAL:  " <<  posActual <<  endl;
            t.start();
        }
    }

	float loop(){

        modAction();
        doAction();

	    episodio();

//        cout << (int)closed_interval_rand(0,3) << endl;

		return m_biped->Head->GetPosition().x;
	}

    void  InitQtable(){
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                qTable[i][j] = 0;
            }
	    }
    }

    void printQtable(){
        for(int i = 0; i < 3; i++){
            printf("|");
            for(int j = 0; j < 3; j++){
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

    //max valor de la tabla de las acciones del estado actual
	int max(int EA){
        int max = 0;

        for(int i = 0; i < 3;i++){
            if(qTable[EA][i] >= max)
                max = qTable[EA][i];
        }

        return max;
	}

	static Test* Create()
	{
		return new BipedTest;
	}
	Biped* m_biped;
};

#endif

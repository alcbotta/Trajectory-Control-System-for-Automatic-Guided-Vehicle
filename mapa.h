#pragma once

#include <iostream>
#include <time.h> 
#include <vector>

using namespace std;

typedef struct point_t
{
	int x;
	int y;
}point_t;

typedef struct node_t
{
	int g, h;
	bool acessivel;
	point_t posicao;

}node_t;

	class Mapa
	{
	public:

		
		int custo(point_t* origem, point_t* destino);

		
		int heuristica(point_t* origem, point_t* destino);

		vector <point_t>* a_star();

		void print_caminho(vector <point_t>* trajeto);

		point_t getDimensao();
		point_t getInicio();
		point_t getObjetivo();
		node_t** getMapa();
		vector <point_t> getObstaculos();

		bool setDimensao(point_t dimensao);
		bool setInicio(point_t inicio);
		bool setObjetivo(point_t objetivo);
		bool setMapa(node_t** mapa);
		bool setObstaculos(vector <point_t>* obstaculos);

		Mapa(point_t dimensoes_mapa);
		Mapa(point_t dimensoes_mapa, vector <point_t>* obstaculos, point_t inicio, point_t fim);
		~Mapa();

	private:
		point_t dimensao;
		point_t inicio;
		point_t objetivo;
		vector <point_t> obstaculos;
		vector <point_t>* vizinhos;
		node_t** mapa;
		
		void cria_grid();

		
		int menor_f(vector <node_t> &nodes);
		
		void nos_vizinhos(point_t* atual);

		
		int pertence_a(point_t* p, vector <node_t> &vector_array);

		
		vector <point_t>* caminho(point_t** trajeto, point_t* atual);
	};

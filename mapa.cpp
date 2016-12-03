#include "mapa.h"
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

Mapa::Mapa(point_t dimensoes_mapa)
{
	if (dimensoes_mapa.x <= 0 || dimensoes_mapa.y <= 0)
	{
		printf("Dimensoes invalidas\n");
		return;
	}
	this->dimensao = dimensoes_mapa;
	this->inicio = {0, 0};
	this->objetivo = { 0, 0};
	this->obstaculos.clear();
	this->vizinhos = new vector<point_t>;
	cria_grid();

}
Mapa::Mapa(point_t dimensoes_mapa, vector <point_t>* obstaculos, point_t inicio, point_t fim)
{
	if (dimensoes_mapa.x <= 0 || dimensoes_mapa.y <= 0)
	{
		printf("Dimensoes invalidas\n");
		return;
	}	
	if (inicio.x < 0 || inicio.x > dimensoes_mapa.x || inicio.y < 0 || inicio.y > dimensoes_mapa.y)
	{
		printf("Ponto inicio invalido");
		return;
	}
	if (fim.x < 0 || fim.x > dimensoes_mapa.x || fim.y < 0 || fim.y > dimensoes_mapa.y)
	{
		printf("Ponto fim invalido");
		return;
	}

	printf("Executando construtor...\n");
	this->dimensao = dimensoes_mapa;
	this->inicio = inicio;
	this->objetivo = fim;

	if (obstaculos != NULL)
		this->obstaculos = *obstaculos;
	else
		this->obstaculos.clear();
	this->vizinhos = new vector<point_t>;

	cria_grid();
}

Mapa::~Mapa()
{
}

int Mapa::custo(point_t* origem, point_t* destino)
{
	return abs(origem->x - destino->x) + abs(origem->y - destino->y);
}

int Mapa::heuristica(point_t* origem, point_t* destino)
{
	return (int)(10 * sqrt(pow((origem->x - destino->x), 2.0) + pow((origem->y - destino->y), 2.0)));
}

void Mapa::cria_grid()
{
	mapa = new node_t*[this->dimensao.x]; //Cria uma matriz de tamanho x, y
	
	point_t valor; //struct auxiliar

	//printf("Custos:\n");

	for (int i = 0; i < this->dimensao.x; i++)
	{
		mapa[i] = new node_t[this->dimensao.y]; 
		valor.x = i;
		for (int j = 0; j < this->dimensao.y; j++)
		{
			valor.y = j;

			mapa[i][j].posicao.x = i;
			mapa[i][j].posicao.y = j;
			mapa[i][j].acessivel = true;
			mapa[i][j].g = custo(&valor, &this->inicio);
			mapa[i][j].h = heuristica(&this->inicio, &this->objetivo);
			/*
			if(start->x == i && start->y == j)//Imprime os valores de custo na tela, para debug
			printf("STR ");
			else if(goal->x == i && goal->y == j)
			printf("FIM ");
			else
			printf("%3d ", node[i][j].g);
			*/
		}
		//printf("\n");
	}

	for (unsigned int i = 0; i < obstaculos.size(); i++)
	{
		if (!((this->inicio.x == obstaculos.at(i).x && this->inicio.y == obstaculos.at(i).y) ||
			  (this->objetivo.x == obstaculos.at(i).x && this->objetivo.y == obstaculos.at(i).y)))
		{
			mapa[obstaculos.at(i).x][obstaculos.at(i).y].acessivel = false;
			mapa[obstaculos.at(i).x][obstaculos.at(i).y].g = -1;
			mapa[obstaculos.at(i).x][obstaculos.at(i).y].h = -1;
		}
	}
}

int Mapa::menor_f(vector <node_t> &nodes)
{
	int menor_valor = 10000;
	int pos = 0;
	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		if (menor_valor > nodes.at(i).g + nodes.at(i).h)
		{
			menor_valor = nodes.at(i).g + nodes.at(i).h;
			pos = i;
		}
	}
	return pos; 
}

void Mapa::nos_vizinhos(point_t* atual)
{
	vizinhos->clear();
	if (atual->x > 0)
	{
		if (mapa[atual->x - 1][atual->y].acessivel)
		{
			point_t ponto = { atual->x - 1, atual->y };//Cria a struct com o ponto vizinho
			vizinhos->push_back(ponto); //E adiciona ao vetor de vizinhos
		}
	}

	if (atual->x < dimensao.x - 1)// Verifica o limite de x
	{
		if (mapa[atual->x + 1][atual->y].acessivel)
		{
			point_t ponto = { atual->x + 1, atual->y };
			vizinhos->push_back(ponto);
		}
	}

	if (atual->y > 0)//Limite inicial de y
	{
		if (mapa[atual->x][atual->y - 1].acessivel)
		{
			point_t ponto = { atual->x, atual->y - 1 };
			vizinhos->push_back(ponto);
		}
	}

	if (atual->y < dimensao.y - 1)//Limite final de y
	{
		if (mapa[atual->x][atual->y + 1].acessivel)
		{
			point_t ponto = { atual->x, atual->y + 1 };
			vizinhos->push_back(ponto);
		}
	}
}

int Mapa::pertence_a(point_t * p, vector <node_t> &vector_array)
{

	for (unsigned int i = 0; i < vector_array.size(); i++)
	if (vector_array.at(i).posicao.x == p->x && vector_array.at(i).posicao.y == p->y)
		return 1;
	return 0;
}

vector <point_t>* Mapa::caminho(point_t** trajeto, point_t* atual)
{
	
	point_t p;
	p.x = atual->x;
	p.y = atual->y;
	vector <point_t>* pontos = new vector<point_t>;
	while (trajeto[p.x][p.y].x != -1)
	{
		pontos->push_back(p);//Adiciona o ponto ao vetor
		//printf("%3d %3d\n", p.x, p.y);
		int aux = p.x;

		p.x = trajeto[p.x][p.y].x;
		p.y = trajeto[aux][p.y].y;

	}
	return pontos;
}

void Mapa::print_caminho(vector <point_t>* trajeto) 
{
	int ponto = 0;

	if (mapa == NULL)
	{
		printf("Mapa nao inicializado\n");
		return;
	}
	if (trajeto == NULL)
	{
		printf("======================================\n==========CAMINHO IMPOSSIVEL==========\n======================================\n");
		trajeto = new vector <point_t>;
		trajeto->clear();
	}
	/*
	printf("Custos:\n");

	for (int i = 0; i < dimensao.x; i++)
	{
		for (int j = 0; j <dimensao.y; j++)
		{
			if (inicio.x == i && inicio.y == j)//Imprime os valores de custo na tela, para debug
				printf("STR ");
			else if (objetivo.x == i && objetivo.y == j)
				printf("FIM ");
			else
				printf("%3d ", mapa[i][j].g);
		}
		printf("\n");
	}
	*/
	printf("legenda:\n");
	printf("%c -> barreira\nS ->Ponto inicial\nF ->Ponto final\n%c ->trajeto\n\n", 0xB0, 0xDB);
	for (int i = 0; i < dimensao.x; i++)
	{

		for (int j = 0; j < dimensao.y; j++)
		{
			ponto = 0;
			if (!mapa[i][j].acessivel)
			{
                printf("|%c", 'b'); continue;
			}
			else if (i == inicio.x && j == inicio.y)
			{
				printf("|S"); continue;
			}
			else if (i == objetivo.x && j == objetivo.y)
			{
				printf("|F"); continue;
			}

			for (unsigned int k = 0; k < trajeto->size(); k++)
			{
				if ((trajeto->at(k).x == i) && (trajeto->at(k).y == j))
				{
                    printf("|%c", 't');
					ponto = 1;
					break;
				}
			}
			if (ponto) continue;
			printf("| ");
		}
		printf("|\n");
	}
}

vector <point_t>* Mapa::a_star()
{
	cria_grid();
	vector <node_t> abertos; 
	vector <node_t> fechados; 
	
	point_t** trajeto; // Armazena o trajeto encontrado	

	trajeto = new point_t*[dimensao.x]; 
	for (int i = 0; i < dimensao.x; i++)
	{
		trajeto[i] = new point_t[dimensao.y];
		for (int j = 0; j < dimensao.y; j++)
		{
			trajeto[i][j].x = -1;
			trajeto[i][j].y = -1;
		}
	}
	
	mapa[inicio.x][inicio.y].g = 0;
	mapa[inicio.x][inicio.y].h = heuristica(&inicio, &objetivo); 
	abertos.push_back(mapa[inicio.x][inicio.y]); 

	while (abertos.size()) 
	{
		int menor = menor_f(abertos); 
		node_t atual = abertos.at(menor);

		if (atual.posicao.x == objetivo.x && atual.posicao.y == objetivo.y)
			return caminho(trajeto, &atual.posicao);

		abertos.erase(abertos.begin() + menor); 
		fechados.push_back(atual); 

		nos_vizinhos(&atual.posicao); 

		for (unsigned int i = 0; i < vizinhos->size(); i++)
		{
			if (pertence_a(&vizinhos->at(i), fechados))
				continue;

			int g = mapa[atual.posicao.x][atual.posicao.y].g + 10;

			
			if (!pertence_a(&atual.posicao, abertos) || g < custo(&vizinhos->at(i), &objetivo))
			{
				trajeto[vizinhos->at(i).x][vizinhos->at(i).y].x = atual.posicao.x;
				trajeto[vizinhos->at(i).x][vizinhos->at(i).y].y = atual.posicao.y;

				mapa[vizinhos->at(i).x][vizinhos->at(i).y].g = g; 

				if (!pertence_a(&atual.posicao, abertos))
				{
					abertos.push_back(mapa[vizinhos->at(i).x][vizinhos->at(i).y]);
				}
			}
		}
	}

	return NULL;
}

point_t Mapa::getDimensao()
{
	return this->dimensao;
}
point_t Mapa::getInicio()
{
	return this->inicio;
}
point_t Mapa::getObjetivo()
{
	return this->objetivo;
}
node_t** Mapa::getMapa()
{
	return this->mapa;
}
vector <point_t> Mapa::getObstaculos()
{
	return this->obstaculos;
}

bool Mapa::setDimensao(point_t dimensao)
{
	if (dimensao.x < 0 || dimensao.y < 0)
		return false;
	this->dimensao = dimensao;
	mapa = NULL;
	return true;
}

bool Mapa::setInicio(point_t inicio)
{
	if (inicio.x < 0 || inicio.x > dimensao.x)
		return false;
	if (inicio.y < 0 || inicio.y > dimensao.y)
		return false;
	this->inicio = inicio;
	return true;
}
bool Mapa::setObjetivo(point_t objetivo)
{
	if (objetivo.x < 0 || objetivo.x > dimensao.x)
		return false;
	if (objetivo.y < 0 || objetivo.y > dimensao.y)
		return false;
	this->objetivo = objetivo;
	return true;
}

bool Mapa::setMapa(node_t** mapa)
{
	if (!mapa)
		return false;
	this->mapa = mapa;
	return true;
}

bool Mapa::setObstaculos(vector <point_t>* obstaculos)
{
	if (!obstaculos)
		this->obstaculos.clear();
	else
		this->obstaculos = *obstaculos;
	return true;
}

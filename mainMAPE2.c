#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define _ISOC99_SOURCE
#include <math.h>

#include "parser.h"
#include "graph.h"
#include "yen.h"
#include "prefixTree.h"
#include "list.h"
#include "array.h"
#include "simulationh2.h"
#include "memory.h"
#include "heuristics.h"
#include "dijkstra.h"

#include <time.h>


int REF = 0;
int INST=0;

void printCurrentPaths(t_array * paths, int numberOfPairs, int numberOfDescriptors){  
	printf("currentPaths\n");
    for (int i = 0; i < numberOfPairs*numberOfDescriptors; i++) { //
   		for (int j = 0; j < arrayLength(arrayGet(paths, i)); j++) {
            printf("%lu ",(unsigned long) arrayGet(arrayGet(paths, i), j));
     }
     printf("\n");
    }    
}


double maxCost(t_graph * graph, t_array * paths, t_array * flowTimes){
	//Avaliação prévia do custo máximo da solução, com base na probabilidade de entrega. Sem considerar as perdas por enfileiramento e interferência.
	int tmp;
	float deliveryProbability, maxInterval, maxCost, flowTime;
	for (int i = 0; i < arrayLength(paths); i++) {
		deliveryProbability = 1.0;
		for (int j = 1; j < arrayLength(arrayGet(paths, i)); j++) {
			tmp = 1 - sqrt((double) GRAPH_MULTIPLIER / (double) graphGetCost(graph, (long) arrayGet(arrayGet(paths, i), j - 1), (long) arrayGet(arrayGet(paths, i), j)));
			//printf("hopCost: %.2f ", tmp);
			deliveryProbability  *= (1 - tmp * tmp * tmp * tmp);
		}
		maxInterval = (int) arrayGet(flowTimes, i) * deliveryProbability;
		flowTime=(int) arrayGet(flowTimes, i);
		maxCost = maxCost + ((maxInterval-flowTime)/maxInterval);			

		//printf("\npathDeliveryProbability: %.2f\n", deliveryProbability);
		//printf("maxInterval: %.2f\n", maxInterval);
	}
	
	return maxCost;

}


void printDSR(t_array * paths[], int numberOfPairs, int numberOfDescriptors, t_return * rf ){  
    printf("DSR Routes\n");
    //INST++;
    FILE *arq_ns3, *arq_result;
    char arq_name[25];
	sprintf(arq_name,"../inst/route/newILS/2s-route300_30_ref");
  
    arq_ns3 = fopen(arq_name, "a+");
    
	 
		fprintf(arq_ns3,"if(INST == %d){\n",INST);	
		for (int i = 0; i < numberOfPairs*numberOfDescriptors; i++) { // Unir os caminhos para simular 
			fprintf(arq_ns3,"if(flowId == %ld){",10+i+1);	
			for (int j = arrayLength(arrayGet(paths, i))-1; j >=0 ; j--) {
				fprintf(arq_ns3,"	pathILS.push_back (Ipv4Address (\"10.0.0.%d\"));", (unsigned long) arrayGet(arrayGet(paths, i), j)+1);
			}
			
			fprintf(arq_ns3,"}\n");
		
		}    
		
		for (int i = 0; i < numberOfPairs*numberOfDescriptors; i++) { // Unir os caminhos para simular 
			fprintf(arq_ns3,"if(flowId == %ld){",i+1);	
			for (int j = 0; j < arrayLength(arrayGet(paths, i)); j++) {
				fprintf(arq_ns3,"	pathILS.push_back (Ipv4Address (\"10.0.0.%d\"));", (unsigned long) arrayGet(arrayGet(paths, i), j)+1);
			}
			fprintf(arq_ns3,"}\n");
		
		}    
		fprintf(arq_ns3,"}\n");
		/* 
		sprintf(arq_name,"ns3/ScenarioRandom3-ILS_Result_2");
		arq_result = fopen(arq_name, "a+");
		for (int f = 0; f <  numberOfPairs*numberOfDescriptors; f++) {

			//fprintf(arq_result, "%d	%.2f	%d	%.2f\n",INST, 1800000/rf->meanInterval,  f+1, 1800000/rf->meanIntervalFlow[f]);
			fprintf(arq_result, "%.2f	%.2f\n",1800000/rf->meanInterval,  1800000/rf->meanIntervalFlow[f]);
			
		}
		fclose(arq_result);
		*/
	
    fclose(arq_ns3);
    
}



void printSolution(t_array * paths[], int numberOfPairs, int numberOfDescriptors, t_graph * graph ){  
    FILE *arq_ns3, *arq_result;
    char arq_name[25];
	sprintf(arq_name,"../inst/delay/4s-pathCost300_30_ref");
  
    arq_ns3 = fopen(arq_name, "a+");
    

		//fprintf(arq_ns3,"INST == %d\n",INST);	
		for (int i = 0; i < numberOfPairs*numberOfDescriptors; i++) { // Unir os caminhos para simular 
			double pathCost=0;
			for (int j = 0; j < arrayLength(arrayGet(paths, i)); j++) {
				//fprintf(arq_ns3,"%d	", (unsigned long) arrayGet(arrayGet(paths, i), j)+1);
				if(j>0){
					//printf("%.2f	", (double) graphGetCost(graph, arrayGet(arrayGet(bestPathsILS[d], i), j-1), arrayGet(arrayGet(bestPathsILS[d], i), j)));
					pathCost +=  (double) graphGetCost(graph, arrayGet(arrayGet(paths, i), j-1), arrayGet(arrayGet(paths, i), j));
				}
			}
			fprintf(arq_ns3,"%.2f\n", pathCost/(double) GRAPH_MULTIPLIER);
		
		}    
		//fprintf(arq_ns3,"\n");
		
    fclose(arq_ns3);
    
}

int comparePaths (t_array * paths[], int numberOfPairs, int numberOfDescriptors ){
for (int i = 0; i < numberOfPairs; i++) {	
	for (int d1 = 0; d1 < numberOfDescriptors-1; d1++) {
		for (int d2 = d1+1; d2 < numberOfDescriptors; d2++) {
			if(arrayGet(paths[d1], i)==arrayGet(paths[d2], i)){
			return 1;	
		}
		}
	}
}
return 0;
}

int main(int argc, char ** argv) {

	int * currentSrc, * currentDst, * currentFlt, * randSrc, * randDst;
	int i, j, c, numberOfPairs, numberOfNodes;
	int numhist, numPaths;
    int *num;

	int numberOfDescriptors = 2; //quantidade de descritores
	int numberOfPathsPerFlow = 10000; // S = conjunto de soluções para cada fluxo
   
	t_graph * graph;
	t_list * pathList;
	t_list * src, * dst, ** flt;
	t_prefixTreeNode * path;
	t_array * nodePairs, * flowTime, * simFlowTime, * txDurations;
	t_array * currentPaths;
	t_array * currentAuxPaths, * neighborPaths, * bestPaths, * histPaths;
	float  bestCost, bestDelay, currentCost, pertCost, bestTime, currentTime;
	clock_t t; //variável para armazenar tempo
    t_return * r, * rf;
	FILE *arq;
	
    /* s = FxP
       s = nodePairs
       F = numberOfPairs
       P = currentPaths1[F] e currentPaths2[F]
     
     s = F1    | currentPaths1 currentPaths2 |
         F2    | currentPaths1 currentPaths2 |
     */
    

	INST = atoi(argv[3]);

	graph = parserParse(argv[1], & src, & dst, &flt);
    //graphPrint(graph);
	/*
	 * Compute paths and place them in an array.
	 * Each array entry corresponds to a pair of
	 * source and destination. Each entry is a 
	 * handler for a list containing the paths.
	 */
	nodePairs = arrayNew(listLength(src)); // Vetor que armazenar todos caminhos do conjunto de soluções em cada fluxo
	flowTime = arrayNew(listLength(flt)); // Vetor que armazenar todos intevalos de tempo de cada fluxo
	txDurations = arrayNew(listLength(flt)); // vetro que armazena as taxas do backoff.

	numberOfNodes = graphSize(graph); //Quantidade de nós
	

//********* Avaliar um caminho específico *****************
//*****Para Determinar uma quantidade específica de fontes e fluxos
numberOfPairs = 1;
numberOfDescriptors =1;
numPaths = numberOfDescriptors*numberOfPairs;


	//Novos valores para os fluxos
	arraySet(flowTime, 0,145513); 
	arraySet(flowTime, 1,145513); 
	arraySet(flowTime, 2,145513); 
	arraySet(flowTime, 3,145513); 

	arraySet(flowTime, 4,145513); 
	arraySet(flowTime, 5,145513); 
	arraySet(flowTime, 6,145513); 
	arraySet(flowTime, 7,145513); 


	currentPaths = arrayNew(4);
	t_array * pathEval;
	pathEval = arrayNew(6);
	arraySet(pathEval, 0, 1);
	arraySet(pathEval, 1, 10);
	arraySet(pathEval, 2, 34);
	arraySet(pathEval, 3, 20);
	arraySet(pathEval, 4, 11);
	arraySet(pathEval, 5, 0);

	arraySet(currentPaths, 0, pathEval);

	/*
	pathEval = arrayNew(6);
	arraySet(pathEval, 0, 1);
	arraySet(pathEval, 1, 10);
	arraySet(pathEval, 2, 34);
	arraySet(pathEval, 3, 20);
	arraySet(pathEval, 4, 11);
	arraySet(pathEval, 5, 0);
	arraySet(currentPaths, 1, pathEval);

	pathEval = arrayNew(5);
	arraySet(pathEval, 0, 2);
	arraySet(pathEval, 1, 7);
	arraySet(pathEval, 2, 16);
	arraySet(pathEval, 3, 33);
	arraySet(pathEval, 4, 0);
	arraySet(currentPaths, 2, pathEval);

	pathEval = arrayNew(5);
	arraySet(pathEval, 0, 2);
	arraySet(pathEval, 1, 7);
	arraySet(pathEval, 2, 16);
	arraySet(pathEval, 3, 33);
	arraySet(pathEval, 4, 0);
	arraySet(currentPaths, 3, pathEval);
*/
	simFlowTime = arrayNew(4); 
	arraySet(simFlowTime, 0, arrayGet(flowTime, 0));
	
	/*
	arraySet(simFlowTime, 1, arrayGet(flowTime, 1));
	arraySet(simFlowTime, 2, arrayGet(flowTime, 2));
	arraySet(simFlowTime, 3, arrayGet(flowTime, 3));
   */

	printf("currentPaths\n");
    for (i = 0; i < 4; i++) { // Unir os caminhos para simular 
   		for (j = 0; j < arrayLength(arrayGet(currentPaths, i)); j++) {
            printf("%lu ",(unsigned long) arrayGet(arrayGet(currentPaths, i), j));
     }
     printf(" - FLT %d \n", arrayGet(simFlowTime, i));
    }    

	MALLOC(r, sizeof(t_return));
	MALLOC(rf, sizeof(t_return));
    r = simulationSimulate(graph, currentPaths, simFlowTime, txDurations ); //função objetivo
    bestCost = r->meanInterval;
    currentTime = ((float)clock() - t)/((CLOCKS_PER_SEC/1000));
    bestTime = currentTime;
    rf=r; // melhor fluxo retornado
    printf(" EvalPath Cost = %.2f\n", bestCost);



    arrayFree(currentPaths);
    free(currentPaths);
	
	
    free(r);
	free(rf);
	
	listFreeWithData(dst);
	free(dst);
	listFreeWithData(src);
	free(src);
	listFreeWithData(flt);
	free(flt);
	graphFree(graph);
	free(graph);
	
 	for (i = 0; i < numberOfPairs; i++) {

		pathList = arrayGet(nodePairs, i);
		for (path = listBegin(pathList); path; path = listNext(pathList)) prefixTreePrune(path);
		listFree(pathList);
		free(pathList);
	}

	arrayFree(nodePairs);
	free(nodePairs);

	arrayFree(flowTime);
	free(flowTime);

	arrayFree(simFlowTime);
	free(simFlowTime);

	arrayFree(txDurations);
	free(txDurations);

	arrayFree(currentAuxPaths);
	free(currentAuxPaths);

	arrayFree(neighborPaths);
	free(neighborPaths);

	arrayFree(bestPaths);
	free(bestPaths);

	arrayFree(histPaths);
	free(histPaths);

	
	
	return(0);
}

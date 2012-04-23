#include "evolutionharness.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#include "base/system.h"
#include "graphics/glstuff.h"
#include "physics.h"
#include "graphics/graphics.h"
#include <fstream>
#define cout STD_COUT

EvolutionHarness::EvolutionHarness()
{
	m_ga=NULL;
	stopEvolution=false;
	//visualize=true;
	m_timestep=1.0f/60.0f;
}

EvolutionHarness::~EvolutionHarness()
{
	if(m_ga)
		delete m_ga;
	if(gPhysicsSDK)
		terminatePhysics();
}
 
void EvolutionHarness::setSimulatorTimeStep(double timeStep)
{
	m_timestep=timeStep;
}

void EvolutionHarness::evolve(bool continueFromFile)
{
	if(NULL==m_ga) 
		systemError("evolve: ga not alloced");
	stopEvolution=false;

	//m_ga->initialize(); //needed?

	//load continue data from file here? This could possibly have been in some evolution setup phase
	//but need to create that first
	//man må enten constructe ga med en pop
	//eller sette pop senere med population(pop) men kanskje enklest å constructe??

	if(continueFromFile) {
		std::ifstream inFile("log_lastgen.txt",std::ios::in);
		if(!inFile)
			systemError("could not open continue file!");

		int lastGen;
		inFile >> lastGen;
		printf("CONTINUE FROM GENERATION %d:\n",lastGen);
		//FUNKER DETTE NÅ? Kan gjøres mer elegant senere ved å loade før man konstruerer GA
		//det fungerer ikke 100%(?) men ihvertfall ser det ut som hovedessensen i genomene kommer med, og eliten føres videre
		GAPopulation pop=m_ga->population(); //get copy of population
		for(int i=0;i<pop.size();i++) {
			//pop.individual(i).read(inFile);
			GABin2DecGenome* ind=(GABin2DecGenome*)&pop.individual(i);
			ind->GA1DBinaryStringGenome::read(inFile);

			//debug out
			pop.individual(i).write(cout);
			cout << std::endl;

		}
		inFile.close();
		m_ga->population(pop);


	}
	else {
		//is it ok to not do this above??
		//otherwise, do an init before loading new inds.. or manually reset stuff
		m_ga->initialize(); //this creates and evaluates all individuals (ie. it is "generation 0")
	}
	
	std::ofstream eliteGenomeLog("log_elites.txt",std::ios::out);
	std::ofstream timeLog("log_eval_times.txt",std::ios::out);
	double globalStartTime=glfwGetTime();
	while(!m_ga->done() && !stopEvolution){
		int lastGenNumber=generationNumber();
		printf("previous gen: %d, saving.\n",lastGenNumber);
			
		/*//hack: if(m_ga->generation() == 20) printStatistics("bestind_gen20.txt");*/

		//one generation is already evaluated
		//saving generation genomes
		std::ofstream outFile("log_lastgen.txt",std::ios::out);
		outFile << lastGenNumber << std::endl; //first write generation number
		GAPopulation pop=m_ga->population();
		for(int i=0;i<pop.size();i++) {
			GABin2DecGenome* ind=(GABin2DecGenome*)&pop.individual(i);
			ind->GA1DBinaryStringGenome::write(outFile);
			outFile << std::endl;
		}
		//once again in float values for human readability
		for(int i=0;i<pop.size();i++) {
			pop.individual(i).write(outFile);
			outFile << std::endl;
			outFile << pop.individual(i).score();
			outFile << std::endl;
		}
		outFile.close();

		//dump elite genomes for each generation (incl fitness in order to able to make a resume)
		eliteGenomeLog << lastGenNumber << " " << m_ga->statistics().bestIndividual().score() << std::endl;
		//now print the genome (float only?)
		//GABin2DecGenome* ind=(GABin2DecGenome*)&pop.best();
		//ind->GA1DBinaryStringGenome::write(eliteGenomeLog);
		//eliteGenomeLog << std::endl;
		pop.best().write(eliteGenomeLog);
		eliteGenomeLog << std::endl;

		printf("\ncreating next generation and evaluating:\n");
		double startTime=glfwGetTime();
		m_ga->step(); //next generation
		double timeElapsed=glfwGetTime()-startTime;
		printf("generation time: %f\n",timeElapsed);

		timeLog << (lastGenNumber+1) << " " << timeElapsed << std::endl;

		m_ga->flushScores();
	}
	double totalEvolutionTime=glfwGetTime()-globalStartTime;
	timeLog << "total evolution time: " << totalEvolutionTime << std::endl;
	printf("total evolution time: %.2f",totalEvolutionTime);
	timeLog.close();
	m_ga->flushScores();
}

void EvolutionHarness::printStatistics(char *fileName)
{
	cout << "The GA found:\n" << m_ga->statistics().bestIndividual() << "\n" << "fitness: " << m_ga->statistics().bestIndividual().score() << "\n";
	cout << "seed:" << GAGetRandomSeed() << "\n";

	printf("writing best individual to file\n");
	//std::ofstream outFile("bestind.txt",std::ios::out);
	std::ofstream outFile(fileName,std::ios::out);
	GABin2DecGenome* ind=(GABin2DecGenome*)&(m_ga->statistics().bestIndividual());
	ind->write(outFile);
	outFile << std::endl;
	ind->GA1DBinaryStringGenome::write(outFile);
	outFile << std::endl;
}



void EvolutionHarness::updateGraphicsAndPhysics(bool freeze)
{
	updateFPS();
	updateWindowSize();

	if(!freeze)
		gScene->simulate((NxReal)m_timestep);
	if(g_visualize)
		updateGraphics();

	gScene->flushStream();
	gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

	if(g_visualize) {
		if(glfwGetKey(GLFW_KEY_LCTRL)) 
			renderDebugInfo(); //hvorfor kan ikke dette også rendres før fetchresults?
		glfwSwapBuffers(); //screen update  
	}
	else
		glfwPollEvents(); //this must be called if swapbuffers is not called

}


void forcedPopEvaluator(GAPopulation & p)
{
	for(int i=0; i<p.size(); i++)
		p.individual(i).evaluate(gaTrue);
}

int EvolutionHarness::generationNumber()
{
	return m_ga->generation();
}

void EvolutionHarness::createGA(const FloatParamMultiValList& p,GAGenome::Evaluator evaluator,const char* paramFileName)
{
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	params.read(paramFileName);

	//initialization of mapping, bits to decimals
	int nBits=0;
	GABin2DecPhenotype mapping;
	std::map<std::string, std::vector<FloatParam> >::const_iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		std::vector<FloatParam>::const_iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),jt->val,jt->minVal,jt->maxVal,jt->precision);
			mapping.add(jt->precision, jt->minVal, jt->maxVal); //specified precision
			nBits+=jt->precision;
		}
	}
	printf("total number of bits: %d\n",nBits);

	GABin2DecGenome genome(mapping, evaluator, (void *)this); //pass a pointer to this object

	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time (?)
	m_ga = new GASimpleGA(pop);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
}

void EvolutionHarness::convertParams( GAGenome& g, FloatParamMultiValList &p )
{
	GABin2DecGenome& genome = (GABin2DecGenome &)g;
	int i=0;
	std::map<std::string, std::vector<FloatParam> >::iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		std::vector<FloatParam>::iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			jt->val=genome.phenotype(i);
			//printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),jt->val,jt->minVal,jt->maxVal,jt->precision);
			i++;
		}
	}
}

#include "mocap.h"
#include "base/system.h"

#include <stdio.h>
#include <float.h>


enum MarkerNames          {RFHD,LSHO,RSHO,RELB,RWRI,LWRI,LELB,RASI,LASI,LKNE,RKNE,RANK,LANK,RTOE,LTOE};
const int connections[15]={RSHO,RSHO,NULL,RSHO,RELB,LELB,LSHO,RSHO,RSHO,LASI,RASI,RKNE,LKNE,RANK,LANK}; //RFHD->RSHO,...

void Mocap::processDataForIntro()
{
	printf("temp test: processing data for compression\n");
	//scaleData(0.05f); //obs!
	Vec3 min(FLT_MAX,FLT_MAX,FLT_MAX);
	Vec3 max(FLT_MIN,FLT_MIN,FLT_MIN);
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			Vec3 v=m_positions[f][m];
			if(v.x<min.x) min.x=v.x;
			if(v.y<min.y) min.y=v.y;
			if(v.z<min.z) min.z=v.z;
			if(v.x>max.x) max.x=v.x;
			if(v.y>max.y) max.y=v.y;
			if(v.z>max.z) max.z=v.z;
		}
	}
	printf("extents: min: %.2f %.2f %.2f  max: %.2f %.2f %.2f\n",min.x,min.y,min.z,max.x,max.y,max.z);
	
	printf("realigning\n");
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			m_positions[f][m]-=min;
		}
	}

	/*
	//kan scale med forskjellige faktorer i forskjellige dimensjoner hvis man bare kjenner disse når man scaler opp igjen
	printf("scaling\n");
	//double maxExtent=__max(max.x-min.x,__max(max.y-min.y,max.z-min.z));
	printf("max extent: %f\n",maxExtent);
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			m_positions[f][m]=m_positions[f][m]*(float)(1.0/maxExtent*255.0);
		}
	}
	*/

	printf("scaling\n");
	Vec3 scale(255,255,255);
	scale/=(max-min);
	printf("scale amounts: %f %f %f\n",scale.x,scale.y,scale.z);
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			m_positions[f][m]=m_positions[f][m]*scale;
		}
	}
	//scaleFactors=max-min;
	scaleFactors=Vec3(1/scale.x,1/scale.y,1/scale.z);

	min.set(FLT_MAX,FLT_MAX,FLT_MAX);
	max.set(FLT_MIN,FLT_MIN,FLT_MIN);
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			Vec3 v=m_positions[f][m];
			if(v.x<min.x) min.x=v.x;
			if(v.y<min.y) min.y=v.y;
			if(v.z<min.z) min.z=v.z;
			if(v.x>max.x) max.x=v.x;
			if(v.y>max.y) max.y=v.y;
			if(v.z>max.z) max.z=v.z;
		}
	}
	printf("extents: min: %.2f %.2f %.2f  max: %.2f %.2f %.2f\n",min.x,min.y,min.z,max.x,max.y,max.z);
	

	saveBinary("mocap.bin");
	//saveIncludeFile("markers.h");
	saveIncludeFile("markers.h",1); //store bytes
}


void Mocap::saveBinary(const char* fileName)
{
	FILE* mcFile;

	mcFile=fopen(fileName,"wb");
	printf("saving mocap data to %s\n",fileName);

	/*for(int f=0; f<m_nFrames; f++) {
		for(int m=0; m<m_nMarkers; m++ ) {
			Vec3 v=m_positions[f][m];
			unsigned char conv[3];
			conv[0]=(unsigned char)v.x;
			conv[1]=(unsigned char)v.y;
			conv[2]=(unsigned char)v.z;
			//fwrite(&v,sizeof(v),1,mcFile);
			fwrite(conv,3*sizeof(unsigned char),1,mcFile);
		}
	}
	fclose(mcFile);*/

	/*//one marker stream at a time compresses better
	for(int m=0; m<m_nMarkers; m++ ) {
		for(int f=0; f<m_nFrames; f++) {
			Vec3 v=m_positions[f][m];
			unsigned char conv[3];
			conv[0]=(unsigned char)v.x;
			conv[1]=(unsigned char)v.y;
			conv[2]=(unsigned char)v.z;
			fwrite(conv,3*sizeof(unsigned char),1,mcFile);
		}
	}
	fclose(mcFile);*/


	for(int m=0; m<m_nMarkers; m++ ) {
		for(int i=0;i<3;i++) {
			for(int f=0; f<m_nFrames; f++) {
				Vec3 v=m_positions[f][m];
				unsigned char comp;
				if(0==i) comp=(unsigned char)v.x;
				if(1==i) comp=(unsigned char)v.y;
				if(2==i) comp=(unsigned char)v.z;
				fwrite(&comp,sizeof(unsigned char),1,mcFile);
			}
		}
	}
	fclose(mcFile);


}

void Mocap::saveIncludeFile(const char* fileName,int dataType)
{
	FILE* mcFile;

	mcFile=fopen(fileName,"w");
	printf("saving mocap data to %s\n",fileName);

	fprintf(mcFile,"#pragma once\n");	
	fprintf(mcFile,"//total time: %f\n",m_endTime);
	fprintf(mcFile,"#define N_MC_FRAMES %d\n#define N_MC_MARKERS %d\n",m_nFrames,m_nMarkers);
	fprintf(mcFile,"#define MC_SAMPLERATE %d\n",m_frequency);
	fprintf(mcFile,"#define MC_SCALEX %ff\n",scaleFactors.x);
	fprintf(mcFile,"#define MC_SCALEY %ff\n",scaleFactors.y);
	fprintf(mcFile,"#define MC_SCALEZ %ff\n",scaleFactors.z);
	if(1==dataType)
		fprintf(mcFile,"unsigned char ");
	else 
		fprintf(mcFile,"float ");


	/*
	fprintf(mcFile,"markers[N_MC_FRAMES][N_MC_MARKERS][3]={\n",m_nFrames,m_nMarkers);	
	//frame streams
	for(int f=0; f<m_nFrames; f++) {
		fprintf(mcFile,"{ ");
		for(int m=0; m<m_nMarkers; m++ ) {
			Vec3 v=m_positions[f][m];
			//fwrite(&v,sizeof(v),1,mcFile);
			if(1==dataType)
				fprintf(mcFile,"{%d,%d,%d}, ",(unsigned char)v.x,(unsigned char)v.y,(unsigned char)v.z);
			else
				fprintf(mcFile,"{%.2ff,%.2ff,%.2ff}, ",v.x,v.y,v.z);
		}
		fprintf(mcFile," },\n");
	}
	fprintf(mcFile,"};\n");
	fclose(mcFile);*/

	/*//marker streams
	fprintf(mcFile,"markers[N_MC_MARKERS][N_MC_FRAMES][3]={\n",m_nFrames,m_nMarkers);	
	for(int m=0; m<m_nMarkers; m++ ) {
		fprintf(mcFile,"{ ");
		for(int f=0; f<m_nFrames; f++) {
			Vec3 v=m_positions[f][m];
			//fwrite(&v,sizeof(v),1,mcFile);
			if(1==dataType)
				fprintf(mcFile,"{%d,%d,%d}, ",(unsigned char)v.x,(unsigned char)v.y,(unsigned char)v.z);
			else
				fprintf(mcFile,"{%.2ff,%.2ff,%.2ff}, ",v.x,v.y,v.z);
		}
		fprintf(mcFile," },\n");
	}
	fprintf(mcFile,"};\n");
	fclose(mcFile);*/


	//x,y,z streams, only for char
	fprintf(mcFile,"markers[3][N_MC_MARKERS][N_MC_FRAMES]={\n",m_nFrames,m_nMarkers);	
	for(int i=0;i<3;i++) {
		fprintf(mcFile,"{ ");
		for(int m=0; m<m_nMarkers; m++ ) {
			fprintf(mcFile,"{ ");
			for(int f=0; f<m_nFrames; f++) {
				Vec3 v=m_positions[f][m];
				unsigned char val;
				if(0==i) val=(unsigned char)v.x;
				if(1==i) val=(unsigned char)v.y;
				if(2==i) val=(unsigned char)v.z;
				fprintf(mcFile,"%d,",val);
			}
			fprintf(mcFile," },\n");
		}
		fprintf(mcFile," },\n");
	}
	fprintf(mcFile,"};\n");
	fclose(mcFile);



}



//note: this is very hacked together and not very flexible
void Mocap::loadFromFile(const char* fileName)
{
	FILE* mcFile;

	mcFile=fopen(fileName,"r");
	printf("loading mocap data from %s\n",fileName);

	fscanf(mcFile,"nMarkers %d\n",&m_nMarkers);
	fscanf(mcFile,"nFrames %d\n",&m_nFrames);
	fscanf(mcFile,"freq %d\n",&m_frequency);
	printf("markers: %d  frames: %d  frequency: %d\n",m_nMarkers,m_nFrames,m_frequency);
	m_endTime=float(m_nFrames)/m_frequency;

	//could resize vectors now to possibly improve efficiency in inner loop?
	m_positions.resize(m_nFrames);
	m_absVel.resize(m_nFrames);
	m_absAcc.resize(m_nFrames);


	//FIX:: sjekk at data er rikitge!

	//read position vectors
	char id[200];
	fscanf(mcFile,"\n");
	fscanf(mcFile,"%s\n",id);
	if(strcmp("POSITION",id) != 0)
		systemError("could not find position data\n");
	int f;
	float lowestY=FLT_MAX;
	for(f=0; f<m_nFrames; f++) {
		for(int m=0; m<m_nMarkers; m++ ) {
			Vec3 v;
			//NB!! think I have to swap elements here
			fscanf(mcFile,"%e %e %e ",&v.x,&v.z,&v.y);
			if(v.y<lowestY) 
				lowestY=v.y;
			//printf("%f %f %f    %e %e %e\n",v.x,v.y,v.z,v.x,v.y,v.z);
			m_positions[f].push_back(v);
		}
	}
	printf("read %d position vectors\n",f);

	//autotransform to ground level
	for(f=0; f<m_nFrames; f++) {
		for(int m=0; m<m_nMarkers; m++ ) {
			m_positions[f][m]-=Vec3(0,lowestY,0);
		}
	}
/*
	//read absolute velocities
	fscanf(mcFile,"\n");
	fscanf(mcFile,"%s\n",id);
	if(strcmp("ABSVEL",id) != 0)
		systemError("could not find velocity data\n");
	float val;
	for(f=0; f<m_nFrames; f++) {
		for(int m=0; m<m_nMarkers; m++ ) {
			fscanf(mcFile,"%e ",&val);
			//printf("%f ",val);
			m_absVel[f].push_back(val);
		}
		//printf("\n");
	}
	printf("read %d velocity values\n",f);

	//read absolute accelerations
	fscanf(mcFile,"\n");
	fscanf(mcFile,"%s\n",id);
	if(strcmp("ABSACC",id) != 0)
		systemError("could not find acceleration data\n");
	for(f=0; f<m_nFrames; f++) {
		for(int m=0; m<m_nMarkers; m++ ) {
			fscanf(mcFile,"%e ",&val);
			//printf("%f ",val);
			m_absAcc[f].push_back(val);
		}
		//printf("\n");
	}
	printf("read %d acceleration values\n",f);
*/


	printf("done loading\n");
	fclose(mcFile);
}

Vec3 Mocap::getPosition(float time, int marker)
{
	//time in seconds
	//for now don't interpolate just get nearest sample
	Vec3 res;
	int sample=int(time*m_frequency);
	int s=__min(__max(sample,0),m_nFrames-1);
	res=m_positions[s][marker];
	return res;
}

void Mocap::scaleData(float val)
{
	for(int f=0;f<m_nFrames;f++) {
		for(int m=0;m<m_nMarkers;m++) {
			m_positions[f][m].scale(val);
			//m_absVel[f][m]*=val;
			//m_absAcc[f][m]*=val;
		}
	}
}
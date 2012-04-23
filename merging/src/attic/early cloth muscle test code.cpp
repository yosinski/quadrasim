MyCloth* mus1=NULL;
MyCloth* mus2=NULL;
MyCloth* mus3=NULL;


MyCloth* generateMuscle(NxVec3& pos, float height,float radius,int heightDivs,int aroundDivs)
{
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = pos;
	clothDesc.dampingCoefficient = 0.7f; //hva er mye? 1 eller 0?
	//clothDesc.density=5.0f;
	clothDesc.friction = 0.5f;
	clothDesc.pressure = 1.0f;
	clothDesc.solverIterations = 10; //default: 5
	//clothDesc.solverIterations = 8; //default: 5
	clothDesc.flags |= NX_CLF_PRESSURE;
	clothDesc.flags |= NX_CLF_GRAVITY;
	clothDesc.flags |= NX_CLF_BENDING;
	clothDesc.flags |= NX_CLF_BENDING_ORTHO; //more realistic
	clothDesc.flags |= NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY; //cloth can also pull other things
	if(gHardwareSimulation)
		clothDesc.flags |= NX_CLF_HARDWARE;
	MyCloth *objCloth = new MyCloth(gScene,clothDesc,height,radius,heightDivs,aroundDivs); //use handmade muscle constructor
	cloths.push_back(objCloth);
	return objCloth;
}



void updateMuscleSetup2()
{
	if(!mus1 || !mus2 || !mus3) return;

	static int frameCount=0; //hack
	frameCount++;
	float maxPressure=3.0f;
	//float pressure=(sin(frameCount/20.0f)*0.5f+0.5f)*3.0f + 1.0f; //1-4 atm
	float p=(sin(frameCount/20.0f)*0.5f+0.5f);
	mus1->getNxCloth()->setPressure(p*maxPressure+1);
	mus2->getNxCloth()->setPressure(p*maxPressure+1);
	mus3->getNxCloth()->setPressure(p*maxPressure+1);
}



void createMuscleSetup2()
{
	float startHeight=4.0f;
	float height=10;
	int heightDivs=8;
	int aroundDivs=16;
	float cubeSize=0.6f;
	int endPoint=(heightDivs+1)*aroundDivs-1;
	float xOffs=-8.0f;
	Part* cube;
	
	mus1=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	mus1->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus1->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);


	heightDivs=32;
	aroundDivs=32;
	xOffs=0;
	mus2=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	endPoint=(heightDivs+1)*aroundDivs-1;
	mus2->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize+0.1f,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus2->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);

	heightDivs=8;
	aroundDivs=64;
	xOffs=8;
	mus3=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	endPoint=(heightDivs+1)*aroundDivs-1;
	mus3->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize+0.1f,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus3->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);
}




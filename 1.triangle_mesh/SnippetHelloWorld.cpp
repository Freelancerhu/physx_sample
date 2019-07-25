
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>
#include <iostream>
#include "PxPhysicsAPI.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene = NULL;

PxMaterial*				gMaterial = NULL;

PxPvd*                  gPvd = NULL;

PxReal stackZ = 10.0f;

PxRigidDynamic* ball = NULL;

PxRigidStatic* triangleMesh = NULL;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	if(ball)
		gScene->removeActor(*ball);
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 100.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	dynamic->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*dynamic);
	ball = dynamic;
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

float rand(float loVal, float hiVal)
{
	return loVal + (float(rand()) / RAND_MAX)*(hiVal - loVal);
}

void createRandomTerrain(const PxVec3& origin, PxU32 numRows, PxU32 numColumns,
	PxReal cellSizeRow, PxReal cellSizeCol, PxReal heightScale,
	PxVec3*& vertices, PxU32*& indices)
{
	PxU32 numX = (numColumns + 1);
	PxU32 numZ = (numRows + 1);
	PxU32 numVertices = numX * numZ;
	PxU32 numTriangles = numRows * numColumns * 2;

	if (vertices == NULL)
		vertices = new PxVec3[numVertices];
	if (indices == NULL)
		indices = new PxU32[numTriangles * 3];
	PxU32 currentIdx = 0;
	for (PxU32 i = 0; i <= numRows; i++)
	{
		for (PxU32 j = 0; j <= numColumns; j++)
		{
			PxVec3 v(origin.x + PxReal(j)*cellSizeRow, origin.y, origin.z + PxReal(i)*cellSizeCol);
			vertices[currentIdx++] = v;
		}
	}
	currentIdx = 0;
	for (PxU32 i = 0; i < numRows; i++)
	{
		for (PxU32 j = 0; j < numColumns; j++)
		{
			PxU32 base = (numColumns + 1)*i + j;
			indices[currentIdx++] = base + 1;
			indices[currentIdx++] = base;
			indices[currentIdx++] = base + numColumns + 1;
			indices[currentIdx++] = base + numColumns + 2;
			indices[currentIdx++] = base + 1;
			indices[currentIdx++] = base + numColumns + 1;
		}
	}
	for (PxU32 i = 0; i < numVertices; i++)
	{
		PxVec3& v = vertices[i];
		v.y += heightScale * rand(-1.0f, 1.0f);
	}
}

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
	params.suppressTriangleMeshRemapTable = true;
	if (!skipMeshCleanup)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	if (!skipEdgeData)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

PxTriangleMesh* createBVH33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff)
{
	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);
	physx::PxCooking* gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	PxCookingParams params = gCooking->getParams();
	params.buildTriangleAdjacencies = true;

	// Create BVH33 midphase
	params.midphaseDesc = PxMeshMidPhase::eBVH33;

	// setup common cooking params
	setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);
	
	if (cookingPerformance)
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eCOOKING_PERFORMANCE;
	else
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;

	if (meshSizePerfTradeoff)
	{
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.0f;
	}
	else
	{
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.55f;
	}

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (skipMeshCleanup)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG
	PxTriangleMesh* triMesh = NULL;
	PxU32 meshSize = 0;

	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (inserted)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);

		PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);

		meshSize = outBuffer.getSize();
	}

	// Print the elapsed time for comparison
	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
	cookingPerformance ? printf("\t\t Cooking performance on\n") : printf("\t\t Cooking performance off\n");
	inserted ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Mesh size/performance trade-off: %f \n", double(params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff));
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!inserted)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}
	return triMesh;
}

// Creates a triangle mesh using BVH34 midphase with different settings.
PxTriangleMesh* createBVH34TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, const PxU32 numTrisPerLeaf)
{
	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);
	physx::PxCooking* gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	PxCookingParams params = gCooking->getParams();

	// Create BVH34 midphase
	params.midphaseDesc = PxMeshMidPhase::eBVH34;

	// setup common cooking params
	setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

	// Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
	// and worse cooking performance. Cooking time is better when more triangles per leaf are used.
	params.midphaseDesc.mBVH34Desc.numTrisPerLeaf = numTrisPerLeaf;

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (skipMeshCleanup)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG


	PxTriangleMesh* triMesh = NULL;
	PxU32 meshSize = 0;

	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (inserted)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);

		PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);

		meshSize = outBuffer.getSize();
	}

	// Print the elapsed time for comparison
	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
	inserted ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Num triangles per leaf: %d \n", numTrisPerLeaf);
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!inserted)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}

	return triMesh;
}

PxTriangleMesh* createTriangleMeshes(int createType)
{
	const PxU32 numColumns = 128;
	const PxU32 numRows = 128;
	const PxU32 numVertices = (numColumns + 1)*(numRows + 1);
	const PxU32 numTriangles = numColumns * numRows * 2;

	PxVec3* vertices = new PxVec3[numVertices];
	PxU32* indices = new PxU32[numTriangles * 3];

	createRandomTerrain(PxVec3(0.0f, 0.0f, 0.0f), numRows, numColumns, 30.0f, 20.0f, 20.0f, vertices, indices);
	
	if (createType / 10 == 33) {
		// Create triangle mesh using BVH33 midphase with different settings
		printf("-----------------------------------------------\n");
		printf("Create triangles mesh using BVH33 midphase: \n\n");
	}
	if (createType == 331)
		// 倾向于runtime speed，进行mesh cleanup和precomputing active edges，将mesh存入stream，这是默认设置，适用于离线cooking。
		return createBVH33TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, false, false);
	if (createType == 332)
		// 倾向于mesh的尺寸，进行mesh cleanup和precomputing active edges，将mesh存入stream。
		return createBVH33TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, false, true);
	if (createType == 333)
		// 倾向于cooking的速度，跳过mesh cleanup，但是进行precompute active edges，将mesh存入PxPhysics。
		// 这些设置适用于runtime cooking，即使选择fast cooking会降低运行时模拟和查询的表现。		
		// 我们仍旧需要保证triangle是有效的。
		return createBVH33TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, true, false);
	if (createType == 334)
		// 倾向于cooking的速度，不跳过mesh cleanup，而是跳过precompute active edges，将mesh存入PxPhysics。
		// 对于runtime cooking，这一种是最快的设置，但是所有的边都设置为active，这样会降低runtime的表现并且影响行为。
		return createBVH33TriangleMesh(numVertices, vertices, numTriangles, indices, false, true, true, true, false);
	if (createType / 10 == 34) {
		// Create triangle mesh using BVH34 midphase with different settings
		printf("-----------------------------------------------\n");
		printf("Create triangles mesh using BVH34 midphase: \n\n");
	}
	if (createType == 341)
		// 倾向于runtime speed, 不跳过mesh  cleanup和precomputing active edges。把mesh存在steam中，这是默认设置，适用于离线cooking
		createBVH34TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, 4);
	if (createType == 342)
		// 倾向于mesh的尺寸，进行mesh cleanup和precomputing active edges，将mesh存入stream。
		createBVH34TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, 15);
	if (createType == 343)
		// 倾向于cooking的速度，跳过mesh cleanup，但是进行precompute active edges，将mesh存入PxPhysics。
		// 这些设置适用于runtime cooking，即使每片叶上有更多的triangles会降低运行时模拟和查询的表现。
		// 我们仍旧需要保证triangle是有效的。
		createBVH34TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, 15);
	if (createType == 344)
		// 倾向于cooking的速度，不跳过mesh cleanup，而是跳过precompute active edges，将mesh存入PxPhysics。
		// 对于runtime cooking，这一种是最快的设置，但是所有的边都设置为active，这样会降低runtime的表现并且影响行为。
		createBVH34TriangleMesh(numVertices, vertices, numTriangles, indices, false, true, true, 15);
}

PxHeightField* createHeightField() {
	PxHeightFieldSample samples1[2500];
	for (PxU32 i = 0; i < 2500; ++i) {
		samples1[i].height = 100;
		samples1[i].materialIndex0 = 2;
		samples1[i].materialIndex1 = 3;
	}

	PxHeightFieldDesc heightFieldDesc;
	heightFieldDesc.nbColumns = 50;
	heightFieldDesc.nbRows = 50;
	heightFieldDesc.thickness = -10;
	heightFieldDesc.convexEdgeThreshold = 3;
	heightFieldDesc.samples.data = samples1;
	heightFieldDesc.samples.stride = sizeof(PxHeightFieldSample);

	physx::PxCooking* gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	PxCookingParams params = gCooking->getParams();
	PxHeightField* pHeightField = gCooking->createHeightField(heightFieldDesc, gPhysics->getPhysicsInsertionCallback());

	// create modified HF samples, this 10-sample strip will be used as a modified row
	// Source samples that are out of range of target heightfield will be clipped with no error.
	PxHeightFieldSample samplesM[100];
	for (PxU32 i = 0; i < 100; i++)
	{
		samplesM[i].height = 50;
		samplesM[i].materialIndex0 = 1;
		samplesM[i].materialIndex1 = 1;
	}

	PxHeightFieldDesc desc10Rows;
	desc10Rows.nbColumns = 15;
	desc10Rows.nbRows = 15;
	desc10Rows.samples.data = samplesM;
	desc10Rows.samples.stride = sizeof(PxHeightFieldSample);

	pHeightField->modifySamples(1, 0, desc10Rows); // modify row 1 with new sample data

	return pHeightField;
}

PxTriangleMesh* createCookedMeshes(int createType) {
	// this is meshgeo
	if (triangleMesh) {
		triangleMesh->release();
	}
	PxMeshScale scale(PxVec3(0.1f, 0.1f, 0.1f), PxQuat::PxQuat(PxIdentity));
	PxTriangleMesh* tm = createTriangleMeshes(createType);
	PxTriangleMeshGeometry geom(tm, scale);
	PxTransform transform(PxVec3(-250.0f, 0.0f, -250.0f), PxQuat::PxQuat(0.0f, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic *actor = gPhysics->createRigidStatic(transform);
	//PxRigidActorExt::createExclusiveShape(*actor, geom, *gMaterial);
	PxShape* meshShape;
	meshShape = actor->createShape(geom, *gMaterial);
	triangleMesh = actor;
	gScene->addActor(*actor);
	return tm;
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);


	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);



	PxTriangleMesh* tm = createCookedMeshes(331);
	// this part gets Vertex and Face Data
	const PxU32 nbVerts = tm->getNbVertices();
	const PxVec3* verts = tm->getVertices();
	const PxU32 nbTris = tm->getNbTriangles();
	const void* tris = tm->getTriangles();
	bool is16Bit = tm->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	std::cout << "has 16 bit indices = " << is16Bit << std::endl;
	std::cout << "number of vertices = =" << nbVerts << std::endl;
	int num = 0;
	//for (PxU32 i = 0; i < 50; ++i) {
	//	for (PxU32 j = 0; j < 50; ++j) {
	//		const PxU16* triIndices = (const PxU16*)tris;
	//		const PxU16 index = triIndices[3 * i + j];
	//		const PxVec3& vertex = verts[index];
	//		num++;
	//		std::cout << vertex.x << ' ' << vertex.y << ' ' << vertex.z << ' ' << num << std::endl;
	//	}
	//}


	// height field
	//PxHeightField* hf = createHeightField();
	//PxHeightFieldGeometry hfGeom(hf, PxMeshGeometryFlags(), PxReal(1), PxReal(1), PxReal(1));
	//PxRigidStatic *hfActor = gPhysics->createRigidStatic(transform);
	//PxRigidActorExt::createExclusiveShape(*actor, hfGeom, *gMaterial);
	//gScene->addActor(*hfActor);


	for (PxU32 i = 0; i < 5; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(500), PxVec3(500, 50, 500));
}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
	gPhysics->release();
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();

	gFoundation->release();

	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	case '1':	createCookedMeshes(331);	break;
	case '2':	createCookedMeshes(332);	break;
	case '3':	createCookedMeshes(333);	break;
	case '4':	createCookedMeshes(334);	break;
	case '5':	createCookedMeshes(341);	break;
	case '6':	createCookedMeshes(342);	break;
	case '7':	createCookedMeshes(343);	break;
	case '8':	createCookedMeshes(344);	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 10000;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}


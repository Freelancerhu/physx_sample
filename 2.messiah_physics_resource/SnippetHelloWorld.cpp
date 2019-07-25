// ****************************************************************************
// Convert binary data to collection then add items of collection to scene
// ****************************************************************************

#include <ctype.h>
#include <iostream>
#include <fstream>
#include "PxPhysicsAPI.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxVisualDebuggerConnection*		
						gConnection	= NULL;

PxReal stackZ = 10.0f;

class PhysxFileHeader
{
public:
	std::uint64_t mMagic;
	std::uint32_t mVersion;
	std::uint16_t mShapeid;
	std::uint64_t mFilesize;

	PxU32 mPxVersion;
	PxU32 mPxMajor;
	PxU32 mPxMinor;
	std::size_t GetHeaderSize();
	void Decode(void * memory, bool isBinary = false);
	void *Encode();
};

std::size_t PhysxFileHeader::GetHeaderSize()
{
	std::cout << " major = " << PX_PHYSICS_VERSION_MAJOR << " minor = " << PX_PHYSICS_VERSION_MINOR << std::endl;
	std::cout << " pxmajor = " << mPxMajor << " pxminor = " << mPxMinor << std::endl;
	return sizeof(uint64_t) + sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint64_t);
}

void PhysxFileHeader::Decode(void * memory, bool isBinary)
{
	size_t ptr = 0;
	char * mem = static_cast<char *>(memory);
	auto func = [&ptr, mem](void *dest, size_t siz)
	{
		memcpy(dest, mem + ptr, siz);
		ptr += siz;
	};
	func(&mMagic, sizeof(mMagic));
	func(&mVersion, sizeof(mVersion));
	func(&mShapeid, sizeof(mShapeid));
	func(&mFilesize, sizeof(mFilesize));
	if (isBinary)
	{
		PxU32 pxHeader;
		func(&pxHeader, sizeof(pxHeader));
		func(&mPxVersion, sizeof(mPxVersion));
		mPxMajor = mPxVersion >> 24;
		mPxMinor = (mPxVersion&((1 << 24) - 1)) >> 16;
	}
}

void *PhysxFileHeader::Encode()
{
	char *mem = static_cast<char *>(malloc(GetHeaderSize()));
	size_t ptr = 0;
	auto func = [mem, &ptr](void *src, size_t siz)
	{
		memcpy(mem + ptr, src, siz);
		ptr += siz;
	};
	func(&mMagic, sizeof(mMagic));
	func(&mVersion, sizeof(mVersion));
	func(&mShapeid, sizeof(mShapeid));
	func(&mFilesize, sizeof(mFilesize));
	return mem;
}

template<size_t ALIGN>
class AlignedMemory128
{
private:
	void *mPtr;
public:
	size_t mSize;
	void *ptr;
	AlignedMemory128(size_t siz);
	~AlignedMemory128();
};

template<size_t ALIGN>
AlignedMemory128<ALIGN>::AlignedMemory128(size_t siz)
{
	this->mSize = siz;
	if (siz == 0)
	{
		this->mPtr = NULL;
		this->ptr = NULL;
		return;
	}
	this->mPtr = malloc(siz + ALIGN);
	this->ptr = (void*)((size_t(this->mPtr) + ALIGN)&~(ALIGN - 1));
}

template<size_t ALIGN>
AlignedMemory128<ALIGN>::~AlignedMemory128()
{
	if (this->mPtr != NULL)
		free(this->mPtr);
}

void ConvertMetaData(){
	std::string srcSt = "E:\\PhysX-3.3-master\\PhysXSDK\\Tools\\BinaryMetaData\\win64.metaData";
	std::string dstSt = "E:\\PhysX-3.3-master\\PhysXSDK\\Tools\\BinaryMetaData\\win64.metaData";
	std::string path = "E:\\PhysX-3.3-master\\PhysXSDK\\Snippets\\SnippetHelloWorld\\resource_og.win64";
	std::string dstBinFile = "E:\\PhysX-3.3-master\\PhysXSDK\\Snippets\\SnippetHelloWorld\\resource_win64.win64";
	char * srcIt = new char[srcSt.length() + 1];
	strcpy(srcIt, srcSt.c_str());
	char * dstIt = new char[dstSt.length() + 1];
	strcpy(dstIt, dstSt.c_str());

	PxDefaultFileInputData srcMetaDataStream(srcIt);
	PxDefaultFileInputData dstMetaDataStream(dstIt);
	PxBinaryConverter* binaryConverter = PxSerialization::createBinaryConverter();
	binaryConverter->setReportMode(physx::PxConverterReportMode::eNORMAL);
	if (!binaryConverter->setMetaData(srcMetaDataStream, dstMetaDataStream))
	{
		binaryConverter->release();
		std::cout << "converter failed." << std::endl;
		return;
	}

	PxDefaultFileInputData  srcBinaryDataStream(path.c_str());
	PxDefaultFileOutputStream dstBinaryDataStream(dstBinFile.c_str());
	binaryConverter->convert(srcBinaryDataStream, srcBinaryDataStream.getLength(), dstBinaryDataStream);
	binaryConverter->release();
	return;
}


PxCollection *ConvertFile(const char *filename, PhysxFileHeader *header, AlignedMemory128<PX_SERIAL_FILE_ALIGN> * &aMemory, bool isBinary, PxCooking * mCooking)
{
	PxSerializationRegistry* registry = PxSerialization::createSerializationRegistry(PxGetPhysics());
	using namespace std;
	if (isBinary) {
		// Open file and get file size
		FILE* fp = fopen(filename, "rb");
		fseek(fp, 0, SEEK_END);
		unsigned fileSize = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		// Allocate aligned memory, load data and deserialize
		void* memory = malloc(fileSize + PX_SERIAL_FILE_ALIGN);
		//void* memory128 = (void*)((size_t(memory) + PX_SERIAL_FILE_ALIGN - 1)&~(PX_SERIAL_FILE_ALIGN - 1));
		// memory128 是文件中的完整原始数据
		size_t s = fread(memory, 1, fileSize, fp);
		std::cout << "memory128 = " << (char*)memory << std::endl;
		fclose(fp);
		header->Decode(memory, true);
		if (header->mPxMajor == 3 && header->mPxMinor == 4)
		{
			registry->release();
			return NULL;
		}
		size_t headerSize = header->GetHeaderSize();
		aMemory = new AlignedMemory128<PX_SERIAL_FILE_ALIGN>(fileSize - headerSize);
		std::cout << "======== fileSize - headerSize = " << fileSize - headerSize << std::endl;
		memcpy(aMemory->ptr, (char*)memory + headerSize, fileSize - headerSize);

		FILE * pFile;
		pFile = fopen("E:\\PhysX-3.3-master\\PhysXSDK\\Snippets\\SnippetHelloWorld\\resource_og.win64", "wb");
		fwrite((char*)memory + headerSize, sizeof(char), fileSize - headerSize, pFile);
		fclose(pFile);

		ConvertMetaData();
		std::cout << "========================== collection am = " << (char*)aMemory->ptr << std::endl;
		PxCollection* collection = PxSerialization::createCollectionFromBinary(aMemory->ptr, *registry);
		registry->release();
		std::cout << "========================== collection = " << collection << std::endl;
		return collection;
	}
	else {
		std::string headername(filename);
		size_t ptr = headername.rfind('\\');
		headername = headername.substr(0, ptr);
		headername = headername + std::string("\\repx.header");
		std::ifstream ifs(headername);
		char *headerMem = static_cast<char *>(malloc(header->GetHeaderSize()));
		ifs.read(headerMem, header->GetHeaderSize());
		ifs.close();
		header->Decode(headerMem, false);
		PxDefaultFileInputData inputData(filename);
		PxCollection* collection = PxSerialization::createCollectionFromXml(inputData, *mCooking, *registry);
		return collection;
	}
}


PxCollection *HsjConvertFile(const char *filename, PhysxFileHeader *header, AlignedMemory128<PX_SERIAL_FILE_ALIGN> * &aMemory, bool isBinary, PxCooking * mCooking)
{
	PxSerializationRegistry* registry = PxSerialization::createSerializationRegistry(PxGetPhysics());
	using namespace std;
	if (isBinary) {
		// Open file and get file size
		FILE* fp = fopen(filename, "rb");
		fseek(fp, 0, SEEK_END);
		unsigned fileSize = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		// Allocate aligned memory, load data and deserialize
		char* memory = static_cast<char *>(malloc(fileSize + PX_SERIAL_FILE_ALIGN));
		char* memory128 = (char*)((size_t(memory) + PX_SERIAL_FILE_ALIGN)&~(PX_SERIAL_FILE_ALIGN - 1));
		size_t s = fread(memory128, 1, fileSize, fp);
		fclose(fp);
		header->Decode(memory128, true);
		if (header->mPxMajor == 3 && header->mPxMinor == 4)
		{
			//Statistics::GetInstance().TakeANote("Warning", STR("Unmatch Version of file : ", filename));
			registry->release();
			return NULL;
		}
		size_t headerSize = header->GetHeaderSize();
		aMemory = new AlignedMemory128<PX_SERIAL_FILE_ALIGN>(fileSize - headerSize);
		//void* tempMemory = malloc(fileSize - headerSize + PX_SERIAL_FILE_ALIGN);
		//void* aMemory = (void*)((size_t(tempMemory) + PX_SERIAL_FILE_ALIGN)&~(PX_SERIAL_FILE_ALIGN - 1));
		memcpy(aMemory->ptr, memory128 + headerSize, fileSize - headerSize);
		PxCollection* collection = PxSerialization::createCollectionFromBinary(aMemory->ptr, *registry);
		registry->release();

		gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

		auto nObj = collection->getNbObjects();
		for (physx::PxU32 i = 0; i < nObj; i++)
		{
			auto objType = collection->getObject(i).getConcreteType();
			std::cout << "===== objtype = " << objType << std::endl; // 3 8 7 	eTRIANGLE_MESH, eMATERIAL, eSHAPE
			if (objType == physx::PxConcreteType::eTRIANGLE_MESH)
			{
				std::cout << "===== physx::PxConcreteType::eTRIANGLE_MESH =====" << std::endl;
				auto& triangle_mesh = collection->getObject(i);
				auto* tm = static_cast<physx::PxTriangleMesh*>(&triangle_mesh);
				PxMeshScale scale(PxVec3(10.1f, 10.1f, 10.1f), PxQuat::PxQuat(PxIdentity));
				PxTriangleMeshGeometry geom(tm, scale);
				PxTransform transform(PxVec3(100.0f, 200.0f, 100.0f), PxQuat::PxQuat(0.0f, PxVec3(0.0f, 0.0f, 1.0f)));
				PxRigidStatic *actor = gPhysics->createRigidStatic(transform);
				actor->createShape(geom, *gMaterial);
				gScene->addActor(*actor);
			}

			//if (objType == physx::PxConcreteType::eMATERIAL)
			//{
			//	std::cout << "===== physx::PxConcreteType::eMATERIAL =====" << std::endl;
			//	auto& material = collection->getObject(i);
			//	auto* mt = static_cast<physx::PxMaterial*>(&material);
			//	std::cout << "getRestitution = " << mt->getRestitution() << std::endl;
			//	std::cout << "getDynamicFriction = " << mt->getDynamicFriction() << std::endl;
			//	std::cout << "getStaticFriction = " << mt->getStaticFriction() << std::endl;
			//	PxReal halfExtent = 2.0f;
			//	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *mt);
			//	const PxTransform& t = PxTransform(PxVec3(0, 0, stackZ -= 10.0f));
			//	PxTransform localTm(PxVec3(PxReal(2 * 2) - PxReal(12 - 2), PxReal(2 * 2 + 1), 0) * halfExtent);
			//	PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			//	body->attachShape(*shape);
			//	gScene->addActor(*body);
			//}


			if (objType == physx::PxConcreteType::eSHAPE)
			{
				std::cout << "===== physx::PxConcreteType::eSHAPE =====" << std::endl;
				auto& shape = static_cast<physx::PxShape&>(collection->getObject(i));
				//PxTransform localTm(PxVec3(10.0f, 100.0f, 10.0f), PxQuat::PxQuat(0.0f, PxVec3(0.0f, 0.0f, 1.0f)));
				//PxRigidStatic *static_actor = gPhysics->createRigidStatic(localTm);
				//static_actor->attachShape(shape);
				//gScene->addActor(*static_actor);
				PxTriangleMeshGeometry tmgeometry;
				if (shape.getTriangleMeshGeometry(tmgeometry) && tmgeometry.isValid()) {
					PxTriangleMesh* tm = tmgeometry.triangleMesh;
					PxMeshScale tmscale = tmgeometry.scale;
					const PxVec3 scaleVec3 = PxVec3(tmscale.scale.x *30, tmscale.scale.y * 30, tmscale.scale.z + 10.0f);
					PxMeshScale new_scale(scaleVec3, PxQuat::PxQuat(0.0f, PxVec3(0.0f, 0.0f, 1.0f)));
					PxMeshGeometryFlags	meshFlags = tmgeometry.meshFlags;
					PxTriangleMeshGeometry geom(tm, new_scale);
					PxTransform transform(PxVec3(20.0f, 200.0f, -200.0f), PxQuat::PxQuat(0.0f, PxVec3(0.0f, 0.0f, 1.0f)));
					PxRigidStatic *actor = gPhysics->createRigidStatic(transform);
					actor->createShape(geom, *gMaterial);
					gScene->addActor(*actor);
				}
			}
			else
			{
			collection->getObject(i).release();
			}
		}

		return collection;
	}
	else {
		std::string headername(filename);
		size_t ptr = headername.rfind('\\');
		headername = headername.substr(0, ptr);
		headername = headername + std::string("\\repx.header");
		std::ifstream ifs(headername);
		char *headerMem = static_cast<char *>(malloc(header->GetHeaderSize()));
		ifs.read(headerMem, header->GetHeaderSize());
		ifs.close();
		header->Decode(headerMem, false);
		PxDefaultFileInputData inputData(filename);
		PxCollection* collection = PxSerialization::createCollectionFromXml(inputData, *mCooking, *registry);
		return collection;
	}
}

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,profileZoneManager);
	physx::PxCooking* gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	if(gPhysics->getPvdConnectionManager())
	{
		gPhysics->getVisualDebugger()->setVisualizeConstraints(true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
		gConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), PVD_HOST, 5425, 10);
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	//===================================================================================================
	PxSerializationRegistry* registry = PxSerialization::createSerializationRegistry(PxGetPhysics());

	PhysxFileHeader physxHeader;
	AlignedMemory128<PX_SERIAL_FILE_ALIGN> *alignMem = NULL;
	std::string path = "E:\\PhysX-3.3-master\\PhysXSDK\\Snippets\\SnippetHelloWorld\\resource.win64";
	PxCollection *collection1 = ConvertFile(path.c_str(), &physxHeader, alignMem, true, gCooking);
	//gScene->addCollection(*collection1);

	std::string newPath = "E:\\PhysX-3.3-master\\PhysXSDK\\Snippets\\SnippetHelloWorld\\resource_win64.win64";
	PxCollection *collection3 = HsjConvertFile(path.c_str(), &physxHeader, alignMem, true, gCooking);
	gScene->addCollection(*collection3);

	//===================================================================================================

	PxSphereGeometry geometry(10);
	PxMaterial* material = PxGetPhysics().createMaterial(0.0f, 0.0f, 0.0f);
	PxShape* shape = PxGetPhysics().createShape(geometry, *material);
	PxTransform t = PxTransform(PxIdentity);
	PxRigidDynamic* dynamic = PxCreateDynamic(PxGetPhysics(), t, geometry, *material, 1.0f);
	gScene->addActor(*dynamic);

	//for(PxU32 i=0;i<5;i++)
	//	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

	//if(!interactive)
	//	createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
	PxProfileZoneManager* profileZoneManager = gPhysics->getProfileZoneManager();
	if(gConnection != NULL)
		gConnection->release();
	gPhysics->release();	
	profileZoneManager->release();
	gFoundation->release();
	
	printf("SnippetHelloWorld done.\n");
}

void keyPress(const char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

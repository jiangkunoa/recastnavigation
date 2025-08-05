#pragma once

// 平台无关的导出宏定义
#if defined(_WIN32) || defined(_WIN64)
	#ifdef RECASTBRIDGE_EXPORTS
		#define RECAST_API __declspec(dllexport)
	#else
		#define RECAST_API __declspec(dllimport)
	#endif
#else
	#define RECAST_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

	// 函数声明使用导出宏
	RECAST_API int load(const char* filePath);
	RECAST_API void release(int navMeshId);
	RECAST_API bool isPointNavigable(int navMeshId, const float* point, float searchRadius);
	RECAST_API float distanceToObstacle(int navMeshId, const float* point, float maxSearchRadius);
	RECAST_API int findPathOptimized(int navMeshId, 
									const float* start, 
									const float* end,
									float* outPoints,
									int maxPoints,
									int* outPointCount);
	RECAST_API bool getTerrainBounds(int navMeshId, float* minx, float* miny, float* maxx, float* maxy);
	RECAST_API bool getRandomPoint(int navMeshId, float* outPoint);
#ifdef __cplusplus
}
#endif
#include "RecastBridge.h"
#include "DetourNavMesh.h"
#include "NavMeshCreator.h"
#include "DetourNavMeshQuery.h"
#include <unordered_map>
#include <atomic>
#include <mutex>

// 缓存管理结构
struct NavMeshCache
{
	std::atomic<int> nextId{1};
	std::unordered_map<int, dtNavMesh*> cache;
	std::mutex cacheMutex; // 互斥锁保护并发访问
};

static NavMeshCache s_navMeshCache;

RECAST_API int load(const char* filePath)
{
	// 1. 创建导航网格对象
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh) return -1; // 内存分配失败

	// 2. 加载文件
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
	{
		dtFreeNavMesh(navMesh);
		return -1; // 文件打开失败
	}

	// 3. 读取文件头（Recast标准格式）
	NavMeshSetHeader header;
	if (fread(&header, sizeof(header), 1, fp) != 1)
	{
		fclose(fp);
		dtFreeNavMesh(navMesh);
		return -1; // 头读取失败
	}

	// 4. 验证文件格式（魔数和版本）
	if (header.magic != NAVMESHSET_MAGIC || header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		dtFreeNavMesh(navMesh);
		return -1; // 无效文件格式
	}

	// 5. 初始化导航网格
	if (dtStatusFailed(navMesh->init(&header.params)))
	{
		fclose(fp);
		dtFreeNavMesh(navMesh);
		return -1; // 初始化失败
	}

	// 6. 逐个加载导航块(tiles)
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		if (fread(&tileHeader, sizeof(tileHeader), 1, fp) != 1)
		{
			fclose(fp);
			dtFreeNavMesh(navMesh);
			return -1; // 分块头读取失败
		}

		auto tileData = static_cast<unsigned char*>(dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM));
		if (!tileData)
		{
			fclose(fp);
			dtFreeNavMesh(navMesh);
			return -1; // 内存分配失败
		}

		if (fread(tileData, tileHeader.dataSize, 1, fp) != 1)
		{
			dtFree(tileData);
			fclose(fp);
			dtFreeNavMesh(navMesh);
			return -1; // 分块数据读取失败
		}

		// 7. 添加导航块到网格
		navMesh->addTile(
			tileData,
			tileHeader.dataSize,
			DT_TILE_FREE_DATA, // 自动管理内存
			tileHeader.tileRef,
			nullptr
		);
	}

	fclose(fp);

	// 8. 缓存并返回ID
	std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);

	int cacheId = s_navMeshCache.nextId++;
	s_navMeshCache.cache[cacheId] = navMesh;

	return cacheId;
}

RECAST_API void release(int navMeshId)
{
	std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
	auto it = s_navMeshCache.cache.find(navMeshId);
	if (it != s_navMeshCache.cache.end())
	{
		dtNavMesh* navMesh = it->second;
		if (navMesh)
		{
			// 2. 释放导航网格本体
			dtFreeNavMesh(navMesh);
		}
		// 3. 从缓存中移除
		s_navMeshCache.cache.erase(it);
	}
}


RECAST_API bool isPointNavigable(int navMeshId, const float* point, float searchRadius)
{
	std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
	auto it = s_navMeshCache.cache.find(navMeshId);
	if (it == s_navMeshCache.cache.end()) return false;

	dtNavMesh* navMesh = it->second;
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	if (!query) return false;

	// 初始化查询器
	if (dtStatusFailed(query->init(navMesh, 1024)))
	{
		dtFreeNavMeshQuery(query);
		return false;
	}

	// 配置搜索参数
	const float halfExtents[3] = {searchRadius, searchRadius, searchRadius};
	dtQueryFilter filter;
	filter.setIncludeFlags(0xFFFF); // 包含所有多边形类型
	filter.setExcludeFlags(0); // 无排除标志

	dtPolyRef nearestRef;
	float nearestPoint[3];

	// 执行最近多边形搜索
	dtStatus status = query->findNearestPoly(
		point, halfExtents, &filter, &nearestRef, nearestPoint
	);

	dtFreeNavMeshQuery(query);

	// 判断结果有效性：需返回有效多边形且距离在阈值内
	return dtStatusSucceed(status) && nearestRef != 0;
}

RECAST_API float distanceToObstacle(int navMeshId, const float* point, float maxSearchRadius)
{
	std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
	auto it = s_navMeshCache.cache.find(navMeshId);
	if (it == s_navMeshCache.cache.end())
	{
		return -1.0f; // 导航网格未找到
	}

	dtNavMesh* navMesh = it->second;
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	if (!query)
	{
		return -2.0f; // 查询对象分配失败
	}

	// 初始化查询器
	if (dtStatusFailed(query->init(navMesh, 2048)))
	{
		dtFreeNavMeshQuery(query);
		return -2.0f; // 初始化失败
	}

	// 配置搜索参数
	const float halfExtents[3] = {maxSearchRadius, maxSearchRadius, maxSearchRadius};
	dtQueryFilter filter;
	filter.setIncludeFlags(0xFFFF); // 包含所有多边形类型

	// 1. 先找到最近的多边形作为起点
	dtPolyRef startRef;
	float nearestPoint[3];
	dtStatus findPolyStatus = query->findNearestPoly(
		point, halfExtents, &filter, &startRef, nearestPoint
	);

	if (dtStatusFailed(findPolyStatus) || startRef == 0)
	{
		dtFreeNavMeshQuery(query);
		return -3.0f; // 未找到起始多边形
	}

	// 2. 执行障碍物距离查询
	float distance = 0.0f;
	float hitPos[3]; // 障碍物位置
	float hitNormal[3]; // 障碍物法线

	dtStatus wallStatus = query->findDistanceToWall(
		startRef, // 起始多边形引用
		point, // 待检测点
		maxSearchRadius, // 最大搜索半径
		&filter, // 查询过滤器
		&distance, // 返回的距离值
		hitPos, // 障碍物位置（输出）
		hitNormal // 障碍物法线（输出）
	);

	dtFreeNavMeshQuery(query);

	// 处理查询结果
	if (dtStatusFailed(wallStatus))
	{
		return -4.0f; // 距离查询失败
	}

	return distance;
}

RECAST_API int findPathOptimized(int navMeshId,
                      const float* start,
                      const float* end,
                      float* outPoints,
                      int maxPoints,
                      int* outPointCount)
{
	// 初始化输出
	*outPointCount = 0;

	// 1. 获取导航网格
	std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
	auto it = s_navMeshCache.cache.find(navMeshId);
	if (it == s_navMeshCache.cache.end())
	{
		return -1; // 导航网格未找到
	}
	dtNavMesh* navMesh = it->second;

	// 2. 创建导航网格查询对象
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	if (!query || dtStatusFailed(query->init(navMesh, 2048)))
	{
		if (query) dtFreeNavMeshQuery(query);
		return -2; // 查询初始化失败
	}

	// 3. 查找起点和终点所在的多边形
	dtQueryFilter filter;
	filter.setIncludeFlags(0xFFFF); // 包含所有可行走区域

	constexpr float halfExtents[3] = {2.0f, 4.0f, 2.0f}; // 搜索范围

	dtPolyRef startRef, endRef;
	float startNearest[3], endNearest[3];

	// 查找起点多边形
	if (dtStatusFailed(query->findNearestPoly(start, halfExtents, &filter, &startRef, startNearest)))
	{
		dtFreeNavMeshQuery(query);
		return -3; // 起点不可达
	}

	// 查找终点多边形
	if (dtStatusFailed(query->findNearestPoly(end, halfExtents, &filter, &endRef, endNearest)))
	{
		dtFreeNavMeshQuery(query);
		return -4; // 终点不可达
	}

	// 4. 准备路径查找
	dtPolyRef pathPolys[512]; // 多边形路径缓冲区
	int pathPolyCount = 0;

	// 5. 执行路径查找
	if (dtStatusFailed(query->findPath(
		startRef, endRef, // 起点和终点多边形
		startNearest, endNearest, // 起点和终点位置
		&filter, // 查询过滤器
		pathPolys, // 多边形路径输出
		&pathPolyCount, // 多边形数量输出
		sizeof(pathPolys) / sizeof(dtPolyRef) // 最大多边形数
	)))
	{
		dtFreeNavMeshQuery(query);
		return -5; // 路径查找失败
	}

	// 6. 将多边形路径转换为点路径
	if (pathPolyCount > 0)
	{
		// 准备临时缓冲区
		auto straightPath = new float[maxPoints * 3];
		unsigned char straightPathFlags[512]; // 路径点标志
		dtPolyRef straightPathPolys[512]; // 路径多边形

		// 执行路径优化
		int straightPathCount = 0;
		query->findStraightPath(
			startNearest, endNearest, // 起点和终点
			pathPolys, pathPolyCount, // 多边形路径
			straightPath, // 点路径输出
			straightPathFlags, // 点标志
			straightPathPolys, // 点所在多边形
			&straightPathCount, // 点数量输出
			maxPoints, // 最大点数
			0 // 选项
		);

		// 7. 复制结果到输出数组
		if (straightPathCount > 0)
		{
			// 确保不超过最大点数
			int pointsToCopy = (straightPathCount < maxPoints) ? straightPathCount : maxPoints;

			// 复制点数据
			memcpy(outPoints, straightPath, sizeof(float) * pointsToCopy * 3);
			*outPointCount = pointsToCopy;
		}

		delete[] straightPath;
	}

	dtFreeNavMeshQuery(query);
	return 0; // 成功
}


RECAST_API bool getTerrainBounds(int navMeshId, float* minx, float* miny, float* maxx, float* maxy)
{
    std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
    auto it = s_navMeshCache.cache.find(navMeshId);
    if (it == s_navMeshCache.cache.end()) return false;

    dtNavMesh* navMesh = it->second;
    if (!navMesh) return false;

    const dtNavMeshParams* params = navMesh->getParams();
    if (!params) return false;

    // 初始化边界为导航网格原点
    float minX = params->orig[0];
    float maxX = params->orig[0];
    float minZ = params->orig[2];
    float maxZ = params->orig[2];

    // 修复1：使用正确的遍历方式获取tile
    for (int z = 0; z < params->maxTiles; ++z) {
        for (int x = 0; x < params->maxTiles; ++x) {
            const dtMeshTile* tile = navMesh->getTileAt(x, z, 0); // 使用公共API获取tile
            if (!tile || !tile->header) continue;

            // 修复2：使用正确的字段名 (x/z 而不是 tileX/tileZ)
            const dtMeshHeader* header = tile->header;
            float tileMinX = params->orig[0] + header->x * params->tileWidth;
            float tileMaxX = tileMinX + params->tileWidth;
            float tileMinZ = params->orig[2] + header->y * params->tileHeight;
            float tileMaxZ = tileMinZ + params->tileHeight;

            // 更新整体边界
            minX = std::min(minX, tileMinX);
            maxX = std::max(maxX, tileMaxX);
            minZ = std::min(minZ, tileMinZ);
            maxZ = std::max(maxZ, tileMaxZ);
        }
    }

    // 输出结果（注意：这里将Z轴范围映射到Y参数，符合用户要求的miny/maxy）
    *minx = minX;
    *miny = minZ;
    *maxx = maxX;
    *maxy = maxZ;

    return true;
}


// Returns a random number [0..1]
static float frand()
{
	//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand()/(float)RAND_MAX;
}

RECAST_API bool getRandomPoint(int navMeshId, float* outPoint)
{
    std::lock_guard<std::mutex> lock(s_navMeshCache.cacheMutex);
    auto it = s_navMeshCache.cache.find(navMeshId);
    if (it == s_navMeshCache.cache.end()) return false;

    dtNavMesh* navMesh = it->second;
    dtNavMeshQuery* query = dtAllocNavMeshQuery();
    if (!query || dtStatusFailed(query->init(navMesh, 2048))) {
        if (query) dtFreeNavMeshQuery(query);
        return false;
    }

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);  // 包含所有可行走区域
    
	dtPolyRef randomRef;
	float randomPt[3];
	// 修复：使用nullptr代替lambda，并调整参数顺序
	if (dtStatusSucceed(query->findRandomPoint(&filter, frand, &randomRef, randomPt))) {
		memcpy(outPoint, randomPt, sizeof(float) * 3);
		dtFreeNavMeshQuery(query);
		return true;
	}

    dtFreeNavMeshQuery(query);
    return false;
}
